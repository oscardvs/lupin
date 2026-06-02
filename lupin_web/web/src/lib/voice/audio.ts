/**
 * Browser audio I/O for the Gemini Live pipeline.
 *
 * Input (mic → server):  16 kHz mono PCM-16 LE, base64-encoded, 50 ms frames.
 * Output (server → spk): 24 kHz mono PCM-16 LE, base64-encoded, variable size,
 *                        scheduled gap-free against AudioContext.currentTime.
 *
 * AudioWorklet is loaded from a blob URL so we don't need a separate static
 * asset path served at runtime.
 */

const INPUT_SAMPLE_RATE = 16000
const OUTPUT_SAMPLE_RATE = 24000
const FRAME_MS = 50

/** Worklet that pipes mono Float32 frames to the main thread at FRAME_MS cadence. */
const MIC_WORKLET_SOURCE = /* js */ `
class LupinMicWorklet extends AudioWorkletProcessor {
  constructor() {
    super()
    this.frameSize = Math.round(sampleRate * ${FRAME_MS} / 1000)
    this.buf = new Float32Array(this.frameSize)
    this.fill = 0
  }
  process(inputs) {
    const ch = inputs[0] && inputs[0][0]
    if (!ch) return true
    for (let i = 0; i < ch.length; i++) {
      this.buf[this.fill++] = ch[i]
      if (this.fill >= this.frameSize) {
        this.port.postMessage(this.buf.slice(0))
        this.fill = 0
      }
    }
    return true
  }
}
registerProcessor('lupin-mic', LupinMicWorklet)
`

function float32ToPcm16(float: Float32Array): Int16Array {
  const out = new Int16Array(float.length)
  for (let i = 0; i < float.length; i++) {
    const s = Math.max(-1, Math.min(1, float[i]))
    out[i] = s < 0 ? s * 0x8000 : s * 0x7fff
  }
  return out
}

function int16ToBase64(buf: Int16Array): string {
  const bytes = new Uint8Array(buf.buffer, buf.byteOffset, buf.byteLength)
  // chunked to avoid arg-list blow-ups on long buffers
  let bin = ''
  const CHUNK = 0x8000
  for (let i = 0; i < bytes.length; i += CHUNK) {
    bin += String.fromCharCode.apply(null, Array.from(bytes.subarray(i, i + CHUNK)))
  }
  return btoa(bin)
}

function base64ToPcm16(b64: string): Int16Array {
  const bin = atob(b64)
  const bytes = new Uint8Array(bin.length)
  for (let i = 0; i < bin.length; i++) bytes[i] = bin.charCodeAt(i)
  return new Int16Array(bytes.buffer, bytes.byteOffset, bytes.byteLength / 2)
}

function pcm16ToFloat32(pcm: Int16Array): Float32Array {
  const out = new Float32Array(pcm.length)
  for (let i = 0; i < pcm.length; i++) out[i] = pcm[i] / 32768
  return out
}

function downsample(input: Float32Array, fromRate: number, toRate: number): Float32Array {
  if (fromRate === toRate) return input
  const ratio = fromRate / toRate
  const outLen = Math.floor(input.length / ratio)
  const out = new Float32Array(outLen)
  // Simple linear interpolation — good enough for speech at these rates.
  for (let i = 0; i < outLen; i++) {
    const src = i * ratio
    const i0 = Math.floor(src)
    const i1 = Math.min(i0 + 1, input.length - 1)
    const t = src - i0
    out[i] = input[i0] * (1 - t) + input[i1] * t
  }
  return out
}

/**
 * Drains time-domain samples from an AnalyserNode and returns RMS in [0, 1].
 * Cheap enough to call every animation frame.
 */
export function readAnalyserRms(analyser: AnalyserNode, scratch: Float32Array): number {
  // Cast to satisfy the lib.dom Float32Array<ArrayBuffer> constraint without
  // forcing a copy.
  analyser.getFloatTimeDomainData(scratch as Float32Array<ArrayBuffer>)
  let sum = 0
  for (let i = 0; i < scratch.length; i++) sum += scratch[i] * scratch[i]
  return Math.sqrt(sum / scratch.length)
}

export class MicCapture {
  private ctx: AudioContext | null = null
  private stream: MediaStream | null = null
  private workletNode: AudioWorkletNode | null = null
  private analyser: AnalyserNode | null = null
  private scratch = new Float32Array(1024)

  /**
   * Request mic, set up the worklet, and start delivering base64 PCM-16 frames at 16 kHz.
   *
   * `onLevel` is optional and called once per audio frame (~every FRAME_MS) with
   * the frame's RMS in [0, 1] and the frame duration in ms. The voice session
   * uses it to drive the speech endpointer so the mic auto-closes on silence.
   */
  async start(
    onFrame: (b64Pcm16: string) => void,
    onLevel?: (rms: number, frameMs: number) => void,
  ): Promise<void> {
    if (this.workletNode) return // already running
    this.stream = await navigator.mediaDevices.getUserMedia({
      audio: {
        echoCancellation: true,
        noiseSuppression: true,
        autoGainControl: true,
        channelCount: 1,
      },
    })
    // Try to get a 16 kHz context directly — if the platform refuses, we
    // resample on the way out.
    let ctx: AudioContext
    try {
      ctx = new AudioContext({ sampleRate: INPUT_SAMPLE_RATE })
    } catch {
      ctx = new AudioContext()
    }
    this.ctx = ctx
    const blob = new Blob([MIC_WORKLET_SOURCE], { type: 'application/javascript' })
    const url = URL.createObjectURL(blob)
    try {
      await ctx.audioWorklet.addModule(url)
    } finally {
      URL.revokeObjectURL(url)
    }
    const source = ctx.createMediaStreamSource(this.stream)
    const node = new AudioWorkletNode(ctx, 'lupin-mic')
    node.port.onmessage = (ev: MessageEvent<Float32Array>) => {
      const float = ev.data
      const at16k =
        ctx.sampleRate === INPUT_SAMPLE_RATE
          ? float
          : downsample(float, ctx.sampleRate, INPUT_SAMPLE_RATE)
      onFrame(int16ToBase64(float32ToPcm16(at16k)))
      if (onLevel) {
        let sum = 0
        for (let i = 0; i < at16k.length; i++) sum += at16k[i] * at16k[i]
        const rms = Math.sqrt(sum / at16k.length)
        const frameMs = (at16k.length / INPUT_SAMPLE_RATE) * 1000
        onLevel(rms, frameMs)
      }
    }
    source.connect(node)
    // Tap the same source into an analyser for the UI orb. The analyser is a
    // pull-based sink — the consumer reads it on the animation clock, so it
    // doesn't need to be connected to destination.
    const analyser = ctx.createAnalyser()
    analyser.fftSize = 2048
    analyser.smoothingTimeConstant = 0.4
    source.connect(analyser)
    this.scratch = new Float32Array(analyser.fftSize)
    this.analyser = analyser
    // Worklet is a sink — don't connect to destination, we don't want the user
    // to hear themselves.
    this.workletNode = node
  }

  async stop(): Promise<void> {
    this.workletNode?.disconnect()
    this.workletNode = null
    this.analyser?.disconnect()
    this.analyser = null
    this.stream?.getTracks().forEach((t) => t.stop())
    this.stream = null
    if (this.ctx && this.ctx.state !== 'closed') {
      try {
        await this.ctx.close()
      } catch {
        /* swallow */
      }
    }
    this.ctx = null
  }

  /** Live mic level in [0, 1]. Returns 0 when not capturing. */
  getLevel(): number {
    if (!this.analyser) return 0
    return readAnalyserRms(this.analyser, this.scratch)
  }

  get running(): boolean {
    return this.workletNode != null
  }
}

/**
 * Plays a stream of base64 PCM-16 (24 kHz mono) chunks gap-free. Schedules each
 * buffer at the tail of the previous one against the AudioContext clock so the
 * model's voice doesn't gap-glitch on chunk boundaries.
 */
export class AudioPlayer {
  private ctx: AudioContext | null = null
  private nextStartTime = 0
  private active = new Set<AudioBufferSourceNode>()
  private gain: GainNode | null = null
  private analyser: AnalyserNode | null = null
  private scratch = new Float32Array(1024)
  private muted = false

  private ensureCtx(): AudioContext {
    if (!this.ctx || this.ctx.state === 'closed') {
      this.ctx = new AudioContext({ sampleRate: OUTPUT_SAMPLE_RATE })
      this.gain = this.ctx.createGain()
      this.gain.gain.value = this.muted ? 0 : 1
      // Insert an analyser between gain and destination so we can read the
      // model's voice level for the reactive orb.
      this.analyser = this.ctx.createAnalyser()
      this.analyser.fftSize = 2048
      this.analyser.smoothingTimeConstant = 0.5
      this.scratch = new Float32Array(this.analyser.fftSize)
      this.gain.connect(this.analyser)
      this.analyser.connect(this.ctx.destination)
      this.nextStartTime = 0
    }
    return this.ctx
  }

  /**
   * Eagerly create and resume the AudioContext. Must be called from inside a
   * user-gesture handler (e.g. the click that starts the voice session).
   *
   * Why: ensureCtx is otherwise lazy — it would only create the context when
   * the first audio chunk arrives from the server, which is well after the
   * user's tap. Browsers (Chrome/Safari autoplay policy) suspend a context
   * created outside a gesture, and the model's voice queues silently until
   * the next gesture happens to resume it. The user-visible symptom is "the
   * answer is in but gated by the next button press". Calling prepare() in
   * the same call stack as the start-button click avoids that.
   */
  async prepare(): Promise<void> {
    const ctx = this.ensureCtx()
    if (ctx.state === 'suspended') {
      try {
        await ctx.resume()
      } catch {
        /* swallow — gesture may have already lapsed; ensureCtx will retry on enqueue */
      }
    }
  }

  /** Live model-voice level in [0, 1]. Returns 0 when nothing is playing. */
  getLevel(): number {
    if (!this.analyser) return 0
    return readAnalyserRms(this.analyser, this.scratch)
  }

  enqueue(b64Pcm16: string): void {
    const ctx = this.ensureCtx()
    if (ctx.state === 'suspended') void ctx.resume()
    const pcm = base64ToPcm16(b64Pcm16)
    if (pcm.length === 0) return
    const float = pcm16ToFloat32(pcm)
    const buffer = ctx.createBuffer(1, float.length, OUTPUT_SAMPLE_RATE)
    // copyToChannel expects Float32Array<ArrayBuffer> in current lib.dom typings;
    // re-pack to drop the SharedArrayBuffer possibility from the buffer type.
    buffer.getChannelData(0).set(float)

    const src = ctx.createBufferSource()
    src.buffer = buffer
    src.connect(this.gain ?? ctx.destination)

    const startAt = Math.max(this.nextStartTime, ctx.currentTime + 0.02)
    src.start(startAt)
    this.nextStartTime = startAt + buffer.duration

    this.active.add(src)
    src.onended = () => this.active.delete(src)
  }

  /** Drop everything queued. Used on interrupted=true and on stop(). */
  flush(): void {
    this.active.forEach((src) => {
      try {
        src.stop()
      } catch {
        /* already stopped */
      }
    })
    this.active.clear()
    if (this.ctx) this.nextStartTime = this.ctx.currentTime
  }

  setMuted(muted: boolean): void {
    this.muted = muted
    if (this.gain) this.gain.gain.value = muted ? 0 : 1
  }

  async close(): Promise<void> {
    this.flush()
    this.analyser?.disconnect()
    this.analyser = null
    if (this.ctx && this.ctx.state !== 'closed') {
      try {
        await this.ctx.close()
      } catch {
        /* swallow */
      }
    }
    this.ctx = null
    this.gain = null
  }
}
