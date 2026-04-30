import {
  AlertCircle,
  KeyRound,
  Loader2,
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

import { Badge } from '@/components/ui/badge'
import { Button } from '@/components/ui/button'
import { Input } from '@/components/ui/input'
import { ScrollArea } from '@/components/ui/scroll-area'
import { useEStop, ESTOP_REASON_LABELS } from '@/lib/estop'
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
  connecting: 'text-primary',
  ready: 'text-primary',
  listening: 'text-emerald-400',
  thinking: 'text-amber-400',
  speaking: 'text-sky-400',
  error: 'text-destructive',
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

  return (
    <div className="flex w-full flex-col gap-3 p-3 sm:gap-4 sm:p-4">
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

      {/* Console */}
      <div className="reticle relative rounded-sm border border-hairline bg-card/35 p-4 sm:p-6 scanline">
        <span className="reticle-bl" aria-hidden />
        <span className="reticle-br" aria-hidden />

        <div className="mb-4 flex flex-wrap items-baseline gap-2">
          <span className="tag tag-strong">voice console</span>
          <span className="tag tag-accent">PNL-VOX-01</span>
          <span className="tag">{session.isLive ? 'gemini live' : 'mock'}</span>
          <span className="tag">{settings.voiceLanguage}</span>
          <span className={cn('tag ml-auto uppercase', STATUS_TONE[session.status])}>
            {STATUS_LABEL[session.status]}
          </span>
          {session.errorDetail ? (
            <span className="tag text-destructive">· {session.errorDetail}</span>
          ) : null}
        </div>

        <div className="grid gap-5 lg:grid-cols-[minmax(0,260px)_minmax(0,1fr)]">
          {/* ───────────────── controls column ───────────────── */}
          <div className="flex flex-col items-center gap-4">
            <PushToTalkButton session={session} disabled={!sessionRunning} />

            <div className="flex w-full flex-col gap-2">
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

            {/* Tools timeline */}
            <div className="w-full">
              <div className="mb-1.5 flex items-baseline gap-2">
                <Wrench className="h-3 w-3 text-primary/80" />
                <span className="tag tag-strong">tool calls</span>
                <span className="ml-auto tag">{session.tools.length}</span>
              </div>
              <ScrollArea className="h-44 rounded-sm border border-hairline bg-background/40">
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
          <div className="flex flex-col gap-2 min-h-0">
            <div className="flex items-baseline gap-2">
              <span className="tag tag-strong">transcript</span>
              <span className="ml-auto tag">{session.transcript.length} turns</span>
            </div>
            <ScrollArea className="h-[42vh] min-h-[280px] rounded-sm border border-hairline bg-background/40 lg:h-[48vh]">
              <div className="flex flex-col gap-2 p-3">
                {session.transcript.length === 0 ? (
                  <div className="rounded-sm border border-dashed border-hairline px-3 py-6 text-center text-xs text-muted-foreground">
                    {sessionRunning
                      ? 'Hold the mic and ask Lupin to drive, navigate, or report state.'
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
                    ? 'Type a prompt (or hold mic)…'
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

      <div className="flex flex-wrap items-center gap-2 text-[11px] text-muted-foreground">
        <AlertCircle className="h-3 w-3 text-primary/80" />
        <span className="tag">model</span>
        <span className="font-mono text-foreground/80">{settings.geminiModel}</span>
        <span className="tag">·</span>
        <span className="tag">caps</span>
        <span className="ticker">
          {settings.voiceMaxLinearMps}m/s · {settings.voiceMaxAngularRps}rad/s
        </span>
        <span className="tag">·</span>
        <span className="tag">{settings.voicePushToTalk ? 'push-to-talk' : 'open mic'}</span>
      </div>
    </div>
  )
}

function PushToTalkButton({
  session,
  disabled,
}: {
  session: ReturnType<typeof useVoiceSession>
  disabled: boolean
}) {
  const [{ voicePushToTalk }] = useSettings()
  const onPress = useCallback(() => {
    if (!voicePushToTalk) return
    void session.beginUtterance()
  }, [voicePushToTalk, session])
  const onRelease = useCallback(() => {
    if (!voicePushToTalk) return
    void session.endUtterance()
  }, [voicePushToTalk, session])

  const label = !voicePushToTalk
    ? session.micActive
      ? 'Mic open'
      : 'Mic'
    : session.micActive
      ? 'Listening…'
      : 'Hold to talk'

  const Icon =
    session.status === 'thinking' ? Loader2 : session.micActive ? Mic : MicOff

  return (
    <button
      type="button"
      disabled={disabled}
      onMouseDown={onPress}
      onMouseUp={onRelease}
      onMouseLeave={(e) => {
        // If they drag off the button while held, treat as release.
        if (e.buttons === 0) return
        onRelease()
      }}
      onTouchStart={(e) => {
        e.preventDefault()
        onPress()
      }}
      onTouchEnd={(e) => {
        e.preventDefault()
        onRelease()
      }}
      className={cn(
        'group relative flex h-44 w-44 select-none items-center justify-center rounded-full border-2 border-hairline bg-card/40 transition',
        'focus-visible:outline-none focus-visible:ring-2 focus-visible:ring-ring focus-visible:ring-offset-2',
        disabled
          ? 'opacity-50'
          : session.micActive
            ? 'border-emerald-500 bg-emerald-500/15 shadow-[0_0_0_4px_rgba(16,185,129,0.18)]'
            : 'hover:border-primary/60 hover:bg-card/70 active:scale-[0.98]',
      )}
      aria-label={label}
    >
      <span
        aria-hidden
        className={cn(
          'absolute inset-2 rounded-full border border-dashed',
          session.micActive ? 'border-emerald-400/50 animate-pulse' : 'border-hairline/60',
        )}
      />
      <Icon
        className={cn(
          'h-12 w-12',
          session.status === 'thinking' && 'animate-spin',
          session.micActive ? 'text-emerald-300' : 'text-foreground/80',
        )}
      />
      <span className="absolute -bottom-6 text-[11px] uppercase tracking-[0.18em] text-muted-foreground">
        {label}
      </span>
    </button>
  )
}

function TranscriptRow({ t }: { t: TranscriptTurn }) {
  const isUser = t.role === 'user'
  const isModel = t.role === 'model'
  const isSystem = t.role === 'system'
  return (
    <div
      className={cn(
        'flex flex-col rounded-sm border px-3 py-2 text-sm',
        isUser && 'border-primary/30 bg-primary/5',
        isModel && 'border-sky-500/30 bg-sky-500/5',
        isSystem && 'border-dashed border-hairline bg-background/60 text-muted-foreground',
        !isUser && !isModel && !isSystem && 'border-hairline bg-card/50',
        !t.final && 'opacity-80',
      )}
    >
      <div className="flex items-baseline gap-2 text-[10px] uppercase tracking-[0.18em] text-muted-foreground">
        <span>{t.role}</span>
        {!t.final ? <span className="tag">…streaming</span> : null}
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
        ok && 'border-emerald-500/40 bg-emerald-500/5',
        err && 'border-destructive/40 bg-destructive/5',
        !settled && 'border-hairline bg-background/60',
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
