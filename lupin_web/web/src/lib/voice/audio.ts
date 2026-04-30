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

export class MicCapture {
  private ctx: AudioContext | null = null
  private stream: MediaStream | null = null
  private workletNode: AudioWorkletNode | null = null

  /** Request mic, set up the worklet, and start delivering base64 PCM-16 frames at 16 kHz. */
  async start(onFrame: (b64Pcm16: string) => void): Promise<void> {
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
    }
    source.connect(node)
    // Worklet is a sink — don't connect to destination, we don't want the user
    // to hear themselves.
    this.workletNode = node
  }

  async stop(): Promise<void> {
    this.workletNode?.disconnect()
    this.workletNode = null
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
  private muted = false

  private ensureCtx(): AudioContext {
    if (!this.ctx || this.ctx.state === 'closed') {
      this.ctx = new AudioContext({ sampleRate: OUTPUT_SAMPLE_RATE })
      this.gain = this.ctx.createGain()
      this.gain.gain.value = this.muted ? 0 : 1
      this.gain.connect(this.ctx.destination)
      this.nextStartTime = 0
    }
    return this.ctx
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
