/**
 * Scripted mock voice session. Used when ?mock=1 is set OR when no Gemini API
 * key is configured. Walks through a deterministic dialogue, fires synthetic
 * tool calls, and ends in a 'ready' state so the UI feels alive in demos.
 */

import type { ToolInvocation, TranscriptTurn, VoiceStatus } from './types'

interface MockOpts {
  onStatus: (s: VoiceStatus, detail?: string) => void
  onTranscript: (turn: TranscriptTurn) => void
  onToolCall: (
    name: string,
    id: string,
    args: Record<string, unknown>,
  ) => Promise<Record<string, unknown>>
}

export interface MockHandle {
  stop: () => void
  sendText: (text: string) => void
}

interface Step {
  delayMs: number
  run: () => void | Promise<void>
}

export function runMockSession(opts: MockOpts): MockHandle {
  let cancelled = false
  let timer: ReturnType<typeof setTimeout> | null = null

  const turn = (
    role: TranscriptTurn['role'],
    text: string,
    final = true,
    idSuffix = '',
  ): TranscriptTurn => ({
    id: `mock-${role}-${Date.now()}-${idSuffix || Math.random().toString(36).slice(2, 6)}`,
    role,
    text,
    startedAt: Date.now(),
    final,
  })

  const steps: Step[] = [
    { delayMs: 400, run: () => opts.onStatus('connecting') },
    { delayMs: 600, run: () => opts.onStatus('ready') },
    {
      delayMs: 800,
      run: () =>
        opts.onTranscript(
          turn(
            'system',
            'Mock session — no Gemini key set. Wire one in Settings to talk to the real model.',
          ),
        ),
    },
    { delayMs: 1500, run: () => opts.onStatus('listening') },
    { delayMs: 800, run: () => opts.onTranscript(turn('user', 'Hey Lupin, what\'s your battery?')) },
    { delayMs: 600, run: () => opts.onStatus('thinking') },
    {
      delayMs: 400,
      run: async () => {
        await opts.onToolCall('query_state', `mock-${Date.now()}`, { fields: ['battery', 'pose'] })
      },
    },
    { delayMs: 600, run: () => opts.onStatus('speaking') },
    {
      delayMs: 200,
      run: () =>
        opts.onTranscript(
          turn('model', 'Battery is around 78%, and I\'m parked roughly at the dock.'),
        ),
    },
    { delayMs: 1500, run: () => opts.onStatus('ready') },
  ]

  const runChain = async (idx: number) => {
    if (cancelled || idx >= steps.length) return
    const step = steps[idx]
    timer = setTimeout(async () => {
      if (cancelled) return
      try {
        await step.run()
      } catch {
        /* swallow — mock isn't supposed to crash */
      }
      void runChain(idx + 1)
    }, step.delayMs)
  }
  void runChain(0)

  return {
    stop: () => {
      cancelled = true
      if (timer) clearTimeout(timer)
      opts.onStatus('idle')
    },
    sendText: (text: string) => {
      opts.onTranscript(turn('user', text))
      setTimeout(() => {
        opts.onStatus('thinking')
        setTimeout(() => {
          opts.onStatus('speaking')
          opts.onTranscript(
            turn('model', `(mock) heard you say: "${text}". Connect a Gemini key for a real reply.`),
          )
          setTimeout(() => opts.onStatus('ready'), 600)
        }, 400)
      }, 200)
    },
  }
}

// re-exported for type convenience in session.tsx
export type { ToolInvocation }
