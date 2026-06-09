import {
  AlertCircle,
  KeyRound,
  Mic,
  MicOff,
  Power,
  Send,
  Square,
  Volume2,
  VolumeX,
  Wrench,
} from 'lucide-react'
import { useCallback, useEffect, useMemo, useRef, useState } from 'react'

import { ViewShell } from '@/components/system/ViewShell'
import { Badge } from '@/components/ui/badge'
import { Button } from '@/components/ui/button'
import { Input } from '@/components/ui/input'
import { ScrollArea } from '@/components/ui/scroll-area'
import { VoiceOrb } from '@/components/widgets/VoiceOrb'
import { useEStop, ESTOP_REASON_LABELS } from '@/lib/estop'
import { useIsPhone } from '@/lib/responsive'
import { isMockMode, useSettings } from '@/lib/settings'
import { useVoiceSession } from '@/lib/voice/session'
import type { ToolInvocation, TranscriptTurn, VoiceStatus } from '@/lib/voice/types'
import { cn } from '@/lib/utils'

const STATUS_LABEL: Record<VoiceStatus, string> = {
  idle: 'idle',
  connecting: 'connecting',
  ready: 'ready',
  listening: 'listening',
  thinking: 'thinking',
  speaking: 'speaking',
  error: 'error',
}

const STATUS_TONE: Record<VoiceStatus, string> = {
  idle: 'text-muted-foreground',
  connecting: 'text-sky-400',
  ready: 'text-lime-300',
  listening: 'text-emerald-400',
  thinking: 'text-amber-400',
  speaking: 'text-sky-400',
  error: 'text-destructive',
}

const STATUS_DOT: Record<VoiceStatus, string> = {
  idle: 'bg-muted-foreground',
  connecting: 'bg-sky-400 animate-pulse',
  ready: 'bg-lime-300 animate-pulse',
  listening: 'bg-emerald-400 animate-pulse',
  thinking: 'bg-amber-400 animate-pulse',
  speaking: 'bg-sky-400 animate-pulse',
  error: 'bg-destructive animate-pulse',
}

const STATUS_HINT: Record<VoiceStatus, string> = {
  idle: 'Press start to begin',
  connecting: 'Opening session…',
  ready: 'Tap to talk',
  listening: 'Listening — tap to stop',
  thinking: 'Thinking…',
  speaking: 'Speaking…',
  error: 'Session error',
}

export function VoiceView() {
  const [settings] = useSettings()
  const estop = useEStop()
  const session = useVoiceSession()
  const transcriptEndRef = useRef<HTMLDivElement | null>(null)
  const [textDraft, setTextDraft] = useState('')

  const mock = useMemo(() => isMockMode(), [])
  const hasKey = settings.geminiApiKey.trim().length > 0
  const sessionRunning = session.status !== 'idle' && session.status !== 'error'

  // Keep transcript pinned to bottom on new turns.
  useEffect(() => {
    transcriptEndRef.current?.scrollIntoView({ behavior: 'smooth', block: 'end' })
  }, [session.transcript.length, session.tools.length])

  const handleStart = useCallback(() => {
    void session.start()
  }, [session])

  const handleStop = useCallback(() => {
    void session.stop()
  }, [session])

  const handleSend = useCallback(() => {
    const t = textDraft.trim()
    if (!t) return
    session.sendText(t)
    setTextDraft('')
  }, [session, textDraft])

  const isPhone = useIsPhone()

  return (
    <ViewShell intent="fit">
      {/* E-stop banner — same chrome as Teleop */}
      {estop.active ? (
        <div className="reticle relative flex flex-wrap items-center gap-3 rounded-sm border-2 border-destructive bg-destructive/10 px-3 py-2.5 text-sm sm:px-4 sm:py-3">
          <span className="reticle-bl" aria-hidden />
          <span className="reticle-br" aria-hidden />
          <Square className="h-5 w-5 shrink-0 fill-destructive text-destructive" />
          <div className="flex-1 min-w-0">
            <div className="flex items-baseline gap-2">
              <span className="font-semibold uppercase tracking-[0.16em] text-destructive">
                E-stop engaged
              </span>
              <span className="tag">PNL-EMG-01</span>
            </div>
            <div className="text-xs text-muted-foreground">
              {estop.reason ? ESTOP_REASON_LABELS[estop.reason] : 'Unknown reason'} — voice agent
              can still chat and read state, but motion tools will refuse.
            </div>
          </div>
          <Button variant="default" size="sm" onClick={estop.reset} className="shrink-0">
            Reset E-stop
          </Button>
        </div>
      ) : null}

      {/* Loud mock-session banner — the #1 "I told it to drive and nothing
          happened" trap: with no Gemini key the session runs the scripted mock,
          which never calls a motion tool, so commands silently never reach the
          robot. Shown while a mock session is actually RUNNING (the pre-session
          key hint below covers the not-yet-started case). */}
      {sessionRunning && !session.isLive ? (
        <div className="reticle relative flex flex-wrap items-center gap-3 rounded-sm border-2 border-amber-500 bg-amber-500/15 px-3 py-2.5 text-sm sm:px-4 sm:py-3">
          <span className="reticle-bl" aria-hidden />
          <span className="reticle-br" aria-hidden />
          <AlertCircle className="h-5 w-5 shrink-0 text-amber-400" />
          <div className="flex-1 min-w-0">
            <div className="flex items-baseline gap-2">
              <span className="font-semibold uppercase tracking-[0.14em] text-amber-300">
                Mock session — robot will NOT move
              </span>
              <span className="tag">CFG-VOX-02</span>
            </div>
            <div className="text-xs text-muted-foreground">
              This is a scripted demo with no Gemini connection. Drive/nav/arm
              commands are never sent to the robot. Add a Gemini API key in
              Settings → Voice (and end + restart the session) to control Lupin.
            </div>
          </div>
        </div>
      ) : null}

      {/* Configuration warnings — only when likely to confuse the user. */}
      {!hasKey && !mock ? (
        <div className="reticle relative flex flex-wrap items-center gap-3 rounded-sm border border-amber-500/60 bg-amber-500/10 px-3 py-2.5 text-sm sm:px-4">
          <span className="reticle-bl" aria-hidden />
          <span className="reticle-br" aria-hidden />
          <KeyRound className="h-4 w-4 shrink-0 text-amber-400" />
          <div className="flex-1 min-w-0">
            <div className="flex items-baseline gap-2">
              <span className="font-semibold uppercase tracking-[0.12em] text-amber-300">
                No Gemini key
              </span>
              <span className="tag">CFG-VOX-01</span>
            </div>
            <div className="text-xs text-muted-foreground">
              Open Settings → Voice → API key. Without one, the tab runs the scripted mock session.
            </div>
          </div>
        </div>
      ) : null}

      {/* Console — opaque ink tile that flexes to fill; transcript scrolls inside. */}
      <div className="reticle relative flex min-h-0 flex-1 flex-col rounded-sm border border-hairline bg-ink-2 p-4 sm:p-6">
        <span className="reticle-bl" aria-hidden />
        <span className="reticle-br" aria-hidden />

        <div className="mb-4 flex flex-wrap items-baseline gap-2">
          <span className="tag tag-strong">voice console</span>
          <span className="tag tag-accent">PNL-VOX-01</span>
          {session.isLive ? (
            <span className="tag">gemini live</span>
          ) : (
            <span className="tag border-amber-500/60 text-amber-300">mock · no robot control</span>
          )}
          <span className="tag">{settings.voiceLanguage}</span>
          <span
            className={cn(
              'tag ml-auto inline-flex items-center gap-1.5 uppercase',
              STATUS_TONE[session.status],
            )}
          >
            <span className={cn('inline-block h-1.5 w-1.5 rounded-full', STATUS_DOT[session.status])} />
            {STATUS_LABEL[session.status]}
          </span>
          {session.errorDetail ? (
            <span className="tag text-destructive">· {session.errorDetail}</span>
          ) : null}
        </div>

        <div className="grid min-h-0 flex-1 gap-5 md:grid-cols-[minmax(0,240px)_minmax(0,1fr)]">
          {/* ───────────────── controls column ───────────────── */}
          {/* min-h-0 lets the column honour the (stretched) grid-row height on
              short viewports; the orb + buttons stay shrink-0 and only the tool
              timeline below compresses, so the column never overflows the tile
              and paints over the footer (the `.reticle` tile is position:relative
              and would otherwise stack above the static footer). */}
          <div className="flex min-h-0 flex-col items-center gap-4">
            <div className="flex shrink-0 flex-col items-center gap-2">
              <VoiceOrb
                status={session.status}
                getInputLevel={session.getInputLevel}
                getOutputLevel={session.getOutputLevel}
                isLive={session.isLive}
                disabled={!sessionRunning}
                micActive={session.micActive}
                onTap={() => {
                  if (!settings.voicePushToTalk) return
                  if (session.micActive) void session.endUtterance()
                  else void session.beginUtterance()
                }}
                size={isPhone ? 160 : 240}
                ariaLabel={
                  settings.voicePushToTalk
                    ? session.micActive
                      ? 'Listening — tap to stop'
                      : 'Tap to talk'
                    : 'Open mic indicator'
                }
              />
              <div className="text-[11px] uppercase tracking-[0.18em] text-muted-foreground">
                {STATUS_HINT[session.status]}
              </div>
            </div>

            <div className="flex w-full shrink-0 flex-col gap-2">
              {!sessionRunning ? (
                <Button onClick={handleStart} className="w-full" size="lg">
                  <Power className="mr-2 h-4 w-4" />
                  Start session
                </Button>
              ) : (
                <Button onClick={handleStop} variant="outline" className="w-full" size="lg">
                  <Power className="mr-2 h-4 w-4" />
                  End session
                </Button>
              )}

              <div className="grid grid-cols-2 gap-2">
                <Button
                  variant="outline"
                  size="sm"
                  onClick={session.toggleSpeakerMuted}
                  disabled={!sessionRunning}
                >
                  {session.speakerMuted ? (
                    <>
                      <VolumeX className="mr-1.5 h-3.5 w-3.5" /> Muted
                    </>
                  ) : (
                    <>
                      <Volume2 className="mr-1.5 h-3.5 w-3.5" /> Audio
                    </>
                  )}
                </Button>
                <Badge
                  variant="outline"
                  className={cn(
                    'flex h-8 items-center justify-center gap-1.5 px-2 font-mono text-[11px] uppercase tracking-wider',
                    session.micActive ? 'border-emerald-500 text-emerald-300' : 'text-muted-foreground',
                  )}
                >
                  {session.micActive ? <Mic className="h-3 w-3" /> : <MicOff className="h-3 w-3" />}
                  {session.micActive ? 'live' : 'idle'}
                </Badge>
              </div>
            </div>

            {/* Tools timeline — the flexible part of the column: keeps its 11rem
                height when the viewport is tall enough, but compresses (and
                scrolls internally) on short viewports so the column fits. */}
            <div className="flex min-h-0 w-full flex-col">
              <div className="mb-1.5 flex shrink-0 items-baseline gap-2">
                <Wrench className="h-3 w-3 text-primary/80" />
                <span className="tag tag-strong">tool calls</span>
                <span className="ml-auto tag">{session.tools.length}</span>
              </div>
              <ScrollArea className="h-44 min-h-0 rounded-sm border border-hairline bg-ink-1">
                <div className="flex flex-col gap-1.5 p-2">
                  {session.tools.length === 0 ? (
                    <div className="px-1 py-2 text-[11px] text-muted-foreground">
                      No tool calls yet.
                    </div>
                  ) : (
                    session.tools.slice().reverse().map((t) => <ToolRow key={t.id} t={t} />)
                  )}
                </div>
              </ScrollArea>
            </div>
          </div>

          {/* ───────────────── transcript column ───────────────── */}
          <div className="flex min-h-0 flex-col gap-2">
            <div className="flex items-baseline gap-2">
              <span className="tag tag-strong">transcript</span>
              <span className="ml-auto tag">{session.transcript.length} turns</span>
            </div>
            <ScrollArea className="min-h-[240px] rounded-sm border border-hairline bg-ink-1 md:min-h-0 md:flex-1">
              <div className="flex flex-col gap-2 p-3">
                {session.transcript.length === 0 ? (
                  <div className="rounded-sm border border-dashed border-hairline px-3 py-6 text-center text-xs text-muted-foreground">
                    {sessionRunning
                      ? 'Tap the mic and ask Lupin to drive, navigate, or report state.'
                      : 'Press “Start session” to begin.'}
                  </div>
                ) : (
                  session.transcript.map((t) => <TranscriptRow key={t.id} t={t} />)
                )}
                <div ref={transcriptEndRef} />
              </div>
            </ScrollArea>

            <div className="flex items-center gap-2">
              <Input
                placeholder={
                  sessionRunning
                    ? 'Type a prompt (or tap mic)…'
                    : 'Start a session, then type or speak'
                }
                value={textDraft}
                onChange={(e) => setTextDraft(e.target.value)}
                onKeyDown={(e) => {
                  if (e.key === 'Enter' && !e.shiftKey) {
                    e.preventDefault()
                    handleSend()
                  }
                }}
                disabled={!sessionRunning}
              />
              <Button
                onClick={handleSend}
                disabled={!sessionRunning || textDraft.trim().length === 0}
                size="icon"
                aria-label="Send"
              >
                <Send className="h-4 w-4" />
              </Button>
            </div>
          </div>
        </div>
      </div>

      {/* relative z-10 keeps this status line above the position:relative console
          tile, so any residual overflow on an extreme-short viewport can never
          paint over the footer (the original layering bug). */}
      <div className="relative z-10 flex shrink-0 flex-wrap items-center gap-2 text-[11px] text-muted-foreground">
        <AlertCircle className="h-3 w-3 text-primary/80" />
        <span className="tag">model</span>
        <span className="font-mono text-foreground/80">{settings.geminiModel}</span>
        <span className="tag">·</span>
        <span className="tag">caps</span>
        <span className="ticker">
          {settings.voiceMaxLinearMps}m/s · {settings.voiceMaxAngularRps}rad/s
        </span>
        <span className="tag">·</span>
        <span className="tag">{settings.voicePushToTalk ? 'tap-to-talk' : 'hands-free'}</span>
        {settings.voiceVadEnabled ? <span className="tag">vad</span> : null}
      </div>
    </ViewShell>
  )
}

function TranscriptRow({ t }: { t: TranscriptTurn }) {
  const isUser = t.role === 'user'
  const isModel = t.role === 'model'
  const isSystem = t.role === 'system'
  return (
    <div
      className={cn(
        'flex flex-col rounded-sm border px-3 py-2 text-sm shadow-sm',
        'animate-in fade-in-0 slide-in-from-bottom-2 duration-300',
        isUser && 'border-primary/30 bg-primary/5',
        isModel && 'border-sky-500/30 bg-sky-500/5',
        isSystem && 'border-dashed border-hairline bg-background/60 text-muted-foreground',
        !isUser && !isModel && !isSystem && 'border-hairline bg-card/50',
        !t.final && 'opacity-80',
      )}
    >
      <div className="flex items-baseline gap-2 text-[10px] uppercase tracking-[0.18em] text-muted-foreground">
        <span
          className={cn(
            'font-semibold',
            isUser && 'text-primary',
            isModel && 'text-sky-300',
            isSystem && 'text-muted-foreground',
          )}
        >
          {t.role}
        </span>
        {!t.final ? (
          <span className="inline-flex items-center gap-1 text-[10px] text-muted-foreground">
            <span className="inline-block h-1 w-1 animate-pulse rounded-full bg-current" />
            streaming
          </span>
        ) : null}
      </div>
      <div className="whitespace-pre-wrap break-words text-foreground">{t.text}</div>
    </div>
  )
}

function ToolRow({ t }: { t: ToolInvocation }) {
  const settled = !!t.result || !!t.error || !!t.blocked
  const ok = settled && t.result?.ok === true
  const err = settled && (t.error || t.result?.ok === false)
  return (
    <div
      className={cn(
        'rounded-sm border px-2 py-1.5 text-[11px]',
        'animate-in fade-in-0 slide-in-from-right-1 duration-200',
        ok && 'border-emerald-500/40 bg-emerald-500/5',
        err && 'border-destructive/40 bg-destructive/5',
        !settled && 'border-hairline bg-background/60 animate-pulse',
      )}
    >
      <div className="flex items-baseline gap-1.5">
        <span className="font-mono font-semibold text-foreground">{t.name}</span>
        {t.blocked ? <span className="tag text-destructive">e-stop</span> : null}
        <span className="ml-auto font-mono text-muted-foreground">
          {new Date(t.startedAt).toLocaleTimeString([], { hour12: false })}
        </span>
      </div>
      {Object.keys(t.args).length ? (
        <div className="mt-0.5 truncate font-mono text-[10px] text-muted-foreground">
          {JSON.stringify(t.args)}
        </div>
      ) : null}
      {settled ? (
        <div
          className={cn(
            'mt-0.5 truncate font-mono text-[10px]',
            err ? 'text-destructive' : 'text-emerald-300/90',
          )}
        >
          → {t.error ?? JSON.stringify(t.result)}
        </div>
      ) : (
        <div className="mt-0.5 font-mono text-[10px] text-muted-foreground">→ pending…</div>
      )}
    </div>
  )
}
