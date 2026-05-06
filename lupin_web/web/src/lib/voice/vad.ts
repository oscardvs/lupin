/**
 * Speech endpointer — detects start and end of an utterance from a stream of
 * RMS samples. Used by the voice session to auto-close the mic on silence so
 * the agent stops listening once the user is done speaking.
 *
 * Two-threshold hysteresis with a silence-hold timer, plus a min/max utterance
 * length so a single popped frame can't end (or start) a turn.
 */

export interface SpeechEndpointerOptions {
  /** RMS that must be sustained for preSpeechHoldMs to declare speechStart. */
  startThreshold: number
  /** RMS below which silence accumulates toward speechEnd. */
  endThreshold: number
  /** Continuous silence duration that ends an utterance. */
  endHoldMs: number
  /** Speech must persist this long before speechStart fires (rejects pops). */
  preSpeechHoldMs: number
  /** Ignore end-of-speech until at least this much speech has elapsed. */
  minUtteranceMs: number
  /** Hard cap so a stuck-open mic eventually closes. */
  maxUtteranceMs: number
  onSpeechStart?: () => void
  onSpeechEnd?: (reason: 'silence' | 'max-duration') => void
}

const DEFAULTS: SpeechEndpointerOptions = {
  startThreshold: 0.02,
  endThreshold: 0.012,
  endHoldMs: 800,
  preSpeechHoldMs: 80,
  minUtteranceMs: 250,
  maxUtteranceMs: 15000,
}

type State = 'idle' | 'arming' | 'speaking'

export class SpeechEndpointer {
  private opts: SpeechEndpointerOptions
  private state: State = 'idle'
  private armingMs = 0
  private silenceMs = 0
  private utteranceMs = 0

  constructor(opts: Partial<SpeechEndpointerOptions> = {}) {
    this.opts = { ...DEFAULTS, ...opts }
  }

  /** Drive the state machine with one frame's RMS and elapsed duration. */
  feed(rms: number, frameMs: number): void {
    if (frameMs <= 0) return

    switch (this.state) {
      case 'idle': {
        if (rms >= this.opts.startThreshold) {
          this.armingMs = frameMs
          this.state = 'arming'
        }
        return
      }

      case 'arming': {
        if (rms >= this.opts.startThreshold) {
          this.armingMs += frameMs
          if (this.armingMs >= this.opts.preSpeechHoldMs) {
            this.state = 'speaking'
            this.silenceMs = 0
            this.utteranceMs = this.armingMs
            this.armingMs = 0
            this.opts.onSpeechStart?.()
          }
        } else {
          this.armingMs = 0
          this.state = 'idle'
        }
        return
      }

      case 'speaking': {
        this.utteranceMs += frameMs
        if (rms < this.opts.endThreshold) {
          this.silenceMs += frameMs
        } else {
          this.silenceMs = 0
        }
        if (this.utteranceMs >= this.opts.maxUtteranceMs) {
          this.endUtterance('max-duration')
          return
        }
        if (
          this.silenceMs >= this.opts.endHoldMs &&
          this.utteranceMs >= this.opts.minUtteranceMs
        ) {
          this.endUtterance('silence')
        }
        return
      }
    }
  }

  /** Force-end if currently speaking. No-op otherwise. */
  flush(reason: 'silence' | 'max-duration' = 'silence'): void {
    if (this.state === 'speaking') this.endUtterance(reason)
    else this.reset()
  }

  reset(): void {
    this.state = 'idle'
    this.armingMs = 0
    this.silenceMs = 0
    this.utteranceMs = 0
  }

  get isSpeaking(): boolean {
    return this.state === 'speaking'
  }

  private endUtterance(reason: 'silence' | 'max-duration'): void {
    const cb = this.opts.onSpeechEnd
    this.reset()
    cb?.(reason)
  }
}
