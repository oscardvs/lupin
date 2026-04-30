/**
 * Shared types for the voice (Gemini Live) layer. The wire-format types are
 * narrow subsets of the Gemini Live BidiGenerateContent protocol — only the
 * fields we read or send are typed here.
 *
 * Reference: https://ai.google.dev/api/live
 */

export type VoiceStatus =
  | 'idle' // disconnected, not running
  | 'connecting' // ws opening / setup pending
  | 'ready' // setupComplete received, awaiting input
  | 'listening' // user speaking, mic streaming
  | 'thinking' // model turn in progress, no audio yet
  | 'speaking' // model turn streaming audio out
  | 'error'

export type SpeakerRole = 'user' | 'model' | 'tool' | 'system'

export interface TranscriptTurn {
  id: string
  role: SpeakerRole
  text: string
  /** Wall-clock when this turn started, ms since epoch. */
  startedAt: number
  /** Whether the model marked this turn finalised (vs partial). */
  final: boolean
}

export interface ToolInvocation {
  id: string
  /** Tool name as declared in the schema (e.g. 'drive', 'nav_goto'). */
  name: string
  args: Record<string, unknown>
  /** Wall-clock when the tool call arrived, ms since epoch. */
  startedAt: number
  /** Result the tool handler returned, if any. */
  result?: Record<string, unknown>
  error?: string
  /** Whether the e-stop refused the call before it ran. */
  blocked?: boolean
}

/** ───────────────────── Wire-format (subset) ───────────────────── */

export interface BidiSetupConfig {
  model: string
  generationConfig?: {
    responseModalities?: ('AUDIO' | 'TEXT')[]
    speechConfig?: {
      voiceConfig?: {
        prebuiltVoiceConfig?: { voiceName: string }
      }
      languageCode?: string
    }
  }
  systemInstruction?: { parts: { text: string }[] }
  tools?: Array<{ functionDeclarations: FunctionDeclaration[] }>
  inputAudioTranscription?: Record<string, never>
  outputAudioTranscription?: Record<string, never>
}

export interface FunctionDeclaration {
  name: string
  description: string
  parameters: {
    type: 'object'
    properties: Record<string, unknown>
    required?: string[]
  }
}

export interface BidiClientMessage {
  setup?: { config: BidiSetupConfig }
  realtimeInput?: {
    audio?: { data: string; mimeType: string }
  }
  toolResponse?: {
    functionResponses: {
      id: string
      name: string
      response: Record<string, unknown>
    }[]
  }
  clientContent?: {
    turns?: { role: 'user' | 'model'; parts: { text: string }[] }[]
    turnComplete?: boolean
  }
}

export interface BidiServerMessage {
  setupComplete?: Record<string, unknown>
  serverContent?: {
    modelTurn?: {
      parts: {
        text?: string
        inlineData?: { data: string; mimeType: string }
      }[]
    }
    turnComplete?: boolean
    interrupted?: boolean
    inputTranscription?: { text: string; finished?: boolean }
    outputTranscription?: { text: string; finished?: boolean }
  }
  toolCall?: {
    functionCalls: { id: string; name: string; args: Record<string, unknown> }[]
  }
  goAway?: { timeLeft?: string }
}
