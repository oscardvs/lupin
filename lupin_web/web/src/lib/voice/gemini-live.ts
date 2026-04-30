/**
 * Browser-side client for Google's Gemini Live API. Single bidirectional
 * WebSocket — we send the setup config + mic frames + tool responses, the
 * server streams text + audio + tool calls back.
 *
 * Wire reference: https://ai.google.dev/api/live
 *
 * Authentication: API key as `?key=` query param. This means the key is
 * exposed to the client — acceptable for the LAN-only Lupin demo, must be
 * traded for an ephemeral access token if this UI ever leaves the LAN.
 */

import type {
  BidiClientMessage,
  BidiServerMessage,
  BidiSetupConfig,
  FunctionDeclaration,
} from './types'

const WS_BASE =
  'wss://generativelanguage.googleapis.com/ws/google.ai.generativelanguage.v1beta.GenerativeService.BidiGenerateContent'

export interface GeminiLiveOptions {
  apiKey: string
  model: string
  systemInstruction: string
  languageCode: string
  tools: FunctionDeclaration[]
  /** Voice handler called for each base64 PCM-16 audio chunk from the model (24 kHz). */
  onAudio: (b64Pcm16: string) => void
  /** Streaming text from the model, may be partial. `final` flips on turn complete. */
  onModelText: (text: string, final: boolean) => void
  /** Streaming user transcript from server-side recognition. */
  onUserText: (text: string, final: boolean) => void
  /** Tool call dispatched by the model. Resolve with the JSON to send back. */
  onToolCall: (
    name: string,
    id: string,
    args: Record<string, unknown>,
  ) => Promise<Record<string, unknown>>
  /** Server signaled the model finished speaking. */
  onTurnComplete: () => void
  /** Server signaled the user interrupted — flush playback. */
  onInterrupted: () => void
  /** Status / error transitions, surfaced to the UI. */
  onStatus: (status: 'connecting' | 'open' | 'ready' | 'closed' | 'error', detail?: string) => void
}

export class GeminiLiveClient {
  private ws: WebSocket | null = null
  private ready = false
  private closed = false
  private opts: GeminiLiveOptions
  /** Accumulator for the current model turn's transcript text. */
  private modelTextBuf = ''
  /** Accumulator for the current user turn's transcript text. */
  private userTextBuf = ''

  constructor(opts: GeminiLiveOptions) {
    this.opts = opts
  }

  open(): void {
    if (this.ws) return
    this.opts.onStatus('connecting')
    const url = `${WS_BASE}?key=${encodeURIComponent(this.opts.apiKey)}`
    const ws = new WebSocket(url)
    this.ws = ws

    ws.onopen = () => {
      if (this.closed) return
      this.opts.onStatus('open')
      const setup: BidiClientMessage = {
        setup: { config: this.buildSetup() },
      }
      ws.send(JSON.stringify(setup))
    }

    ws.onmessage = async (ev) => {
      const text = typeof ev.data === 'string' ? ev.data : await (ev.data as Blob).text()
      let msg: BidiServerMessage
      try {
        msg = JSON.parse(text) as BidiServerMessage
      } catch {
        return
      }
      this.handleServer(msg)
    }

    ws.onerror = () => {
      this.opts.onStatus('error', 'WebSocket error')
    }

    ws.onclose = (ev) => {
      this.ready = false
      this.ws = null
      this.opts.onStatus('closed', ev.reason || `code ${ev.code}`)
    }
  }

  close(): void {
    this.closed = true
    this.ready = false
    try {
      this.ws?.close()
    } catch {
      /* swallow */
    }
    this.ws = null
  }

  /** Send one base64 PCM-16 mic frame at 16 kHz. No-op until setup is complete. */
  sendAudio(b64Pcm16: string): void {
    if (!this.ready || !this.ws || this.ws.readyState !== WebSocket.OPEN) return
    const msg: BidiClientMessage = {
      realtimeInput: {
        audio: { data: b64Pcm16, mimeType: 'audio/pcm;rate=16000' },
      },
    }
    this.ws.send(JSON.stringify(msg))
  }

  /** Send a text turn from the user. Useful for typed prompts and the mock harness. */
  sendUserText(text: string): void {
    if (!this.ready || !this.ws || this.ws.readyState !== WebSocket.OPEN) return
    const msg: BidiClientMessage = {
      clientContent: {
        turns: [{ role: 'user', parts: [{ text }] }],
        turnComplete: true,
      },
    }
    this.ws.send(JSON.stringify(msg))
  }

  private buildSetup(): BidiSetupConfig {
    return {
      model: this.opts.model,
      generationConfig: {
        responseModalities: ['AUDIO'],
        speechConfig: {
          languageCode: this.opts.languageCode,
        },
      },
      systemInstruction: { parts: [{ text: this.opts.systemInstruction }] },
      tools: this.opts.tools.length ? [{ functionDeclarations: this.opts.tools }] : undefined,
      inputAudioTranscription: {},
      outputAudioTranscription: {},
    }
  }

  private handleServer(msg: BidiServerMessage): void {
    if (msg.setupComplete) {
      this.ready = true
      this.opts.onStatus('ready')
      return
    }
    const sc = msg.serverContent
    if (sc) {
      if (sc.interrupted) {
        this.modelTextBuf = ''
        this.opts.onInterrupted()
      }
      if (sc.inputTranscription?.text) {
        this.userTextBuf += sc.inputTranscription.text
        this.opts.onUserText(this.userTextBuf, !!sc.inputTranscription.finished)
        if (sc.inputTranscription.finished) this.userTextBuf = ''
      }
      if (sc.outputTranscription?.text) {
        this.modelTextBuf += sc.outputTranscription.text
        this.opts.onModelText(this.modelTextBuf, !!sc.outputTranscription.finished)
        if (sc.outputTranscription.finished) this.modelTextBuf = ''
      }
      if (sc.modelTurn?.parts) {
        for (const p of sc.modelTurn.parts) {
          if (p.inlineData?.data && p.inlineData.mimeType?.startsWith('audio/')) {
            this.opts.onAudio(p.inlineData.data)
          }
          if (p.text) {
            // Some configurations stream text via modelTurn.parts[].text instead
            // of outputTranscription. Treat it the same.
            this.modelTextBuf += p.text
            this.opts.onModelText(this.modelTextBuf, false)
          }
        }
      }
      if (sc.turnComplete) {
        if (this.modelTextBuf) this.opts.onModelText(this.modelTextBuf, true)
        this.modelTextBuf = ''
        this.opts.onTurnComplete()
      }
    }
    if (msg.toolCall) {
      void this.handleToolCalls(msg.toolCall.functionCalls)
    }
  }

  private async handleToolCalls(
    calls: { id: string; name: string; args: Record<string, unknown> }[],
  ): Promise<void> {
    const responses = await Promise.all(
      calls.map(async (c) => {
        let response: Record<string, unknown>
        try {
          response = await this.opts.onToolCall(c.name, c.id, c.args ?? {})
        } catch (e) {
          response = { error: e instanceof Error ? e.message : String(e) }
        }
        return { id: c.id, name: c.name, response }
      }),
    )
    if (!this.ws || this.ws.readyState !== WebSocket.OPEN) return
    const msg: BidiClientMessage = { toolResponse: { functionResponses: responses } }
    this.ws.send(JSON.stringify(msg))
  }
}
