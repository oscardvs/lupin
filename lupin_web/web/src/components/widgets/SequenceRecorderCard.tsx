import { useCallback, useEffect, useState } from 'react'
import { useRos, useTopic } from '@/lib/ros'
import { ROS_TYPE, type ArmLibraryState, type ArmLibrarySeqMeta } from '@/types/ros'
import {
  fetchLibrary, startRecording, saveRecording, cancelRecording,
  runSequence, stopSequence, deleteEntry,
} from '@/lib/armLibrary'

const EMPTY_STATE: ArmLibraryState = {
  recording: false, playing: false, name: '', mode: '',
  progress: 0, elapsed_s: 0, n_waypoints: 0, torque: true,
}

/** Record (kinesthetic / teleop) and replay arm sequences. Kinesthetic shows a
 * "support the arm" confirm reusing the calibrate-flow copy before torque is cut. */
export function SequenceRecorderCard({ disabled }: { disabled: boolean }) {
  const { callService } = useRos()
  const [seqs, setSeqs] = useState<ArmLibrarySeqMeta[]>([])
  const [mode, setMode] = useState<'kinesthetic' | 'teleop'>('teleop')
  const [includeGripper, setIncludeGripper] = useState(true)
  const [speed, setSpeed] = useState(1.0)
  const [name, setName] = useState('')
  const [error, setError] = useState<string | null>(null)
  const [confirming, setConfirming] = useState(false)
  // Label of the in-flight action (e.g. 'Disabling torque…') so Cancel/Stop/
  // Record aren't dead buttons while a multi-second torque round-trip runs.
  const [busy, setBusy] = useState<string | null>(null)

  // Live recorder/replay state from the node. useTopic returns a ref and fires
  // onMessage; mirror into React state so the card re-renders on each update
  // (same onMessage+state pattern as ArmJointsCard).
  const [state, setState] = useState<ArmLibraryState>(EMPTY_STATE)
  useTopic<{ data: string }>('/lupin/arm/library/state', ROS_TYPE.String, {
    onMessage: (msg) => {
      try { setState({ ...EMPTY_STATE, ...JSON.parse(msg.data) }) } catch { /* ignore */ }
    },
  })

  const refresh = useCallback(async () => {
    try { setSeqs((await fetchLibrary(callService)).sequences) }
    catch (e) { setError(e instanceof Error ? e.message : String(e)) }
  }, [callService])

  useEffect(() => { void refresh() }, [refresh])
  // Re-fetch when a recording finishes (recording transitions true->false).
  useEffect(() => { if (!state.recording) void refresh() }, [state.recording, refresh])

  const doStart = useCallback(async () => {
    setError(null); setConfirming(false)
    setBusy(mode === 'kinesthetic' ? 'Disabling torque…' : 'Starting…')
    try { const r = await startRecording(callService, mode, includeGripper); if (!r.success) setError(r.message) }
    catch (e) { setError(e instanceof Error ? e.message : String(e)) }
    finally { setBusy(null) }
  }, [callService, mode, includeGripper])

  const onRecord = useCallback(() => {
    if (mode === 'kinesthetic') setConfirming(true)  // show "support the arm" modal
    else void doStart()
  }, [mode, doStart])

  const onSave = useCallback(async () => {
    const n = name.trim()
    if (!n) { setError('name the sequence first'); return }
    setError(null); setBusy('Saving…')
    try {
      const r = await saveRecording(callService, n)
      if (!r.success) setError(r.message)
      else { setName(''); await refresh() }
    } catch (e) { setError(e instanceof Error ? e.message : String(e)) }
    finally { setBusy(null) }
  }, [callService, name, refresh])

  const onCancel = useCallback(async () => {
    setError(null); setBusy('Cancelling…')
    // Surface failures: a swallowed error here looks like a dead Cancel button.
    try { const r = await cancelRecording(callService); if (!r.success) setError(r.message) }
    catch (e) { setError(e instanceof Error ? e.message : String(e)) }
    finally { setBusy(null) }
  }, [callService])

  const onPlay = useCallback(async (n: string) => {
    setError(null)
    try { const r = await runSequence(callService, n, speed); if (!r.success) setError(r.message) }
    catch (e) { setError(e instanceof Error ? e.message : String(e)) }
  }, [callService, speed])

  const onStop = useCallback(async () => {
    setError(null); setBusy('Stopping…')
    try { const r = await stopSequence(callService); if (!r.success) setError(r.message) }
    catch (e) { setError(e instanceof Error ? e.message : String(e)) }
    finally { setBusy(null) }
  }, [callService])
  const onDelete = useCallback(async (n: string) => {
    if (!confirm(`Delete sequence "${n}"?`)) return
    try { await deleteEntry(callService, 'sequence', n); await refresh() }
    catch (e) { setError(e instanceof Error ? e.message : String(e)) }
  }, [callService, refresh])

  const recording = state.recording
  const playing = state.playing

  return (
    <section className="reticle relative rounded-sm border border-hairline bg-ink-2 p-3">
      <span className="reticle-bl" aria-hidden />
      <span className="reticle-br" aria-hidden />
      <h3 className="mb-2 text-sm font-semibold tracking-wide text-foreground">Sequence Recorder</h3>

      {!recording && (
        <div className="mb-2 flex items-center gap-3 text-sm">
          <label className="flex items-center gap-1">
            <input type="radio" checked={mode === 'teleop'} onChange={() => setMode('teleop')} /> Teleop
          </label>
          <label className="flex items-center gap-1">
            <input type="radio" checked={mode === 'kinesthetic'} onChange={() => setMode('kinesthetic')} /> Kinesthetic
          </label>
          <label className="ml-auto flex items-center gap-1 text-xs text-muted-foreground">
            <input type="checkbox" checked={includeGripper} onChange={(e) => setIncludeGripper(e.target.checked)} /> gripper
          </label>
        </div>
      )}

      {!recording ? (
        <button onClick={onRecord} disabled={disabled || playing || busy !== null}
          className="mb-3 w-full rounded-sm bg-destructive px-3 py-1.5 text-sm font-semibold text-destructive-foreground disabled:opacity-40">
          {busy ?? `● Record ${mode}`}
        </button>
      ) : (
        <div className="mb-3 rounded-sm border border-destructive/40 bg-destructive/10 p-2">
          <div className="mb-2 text-sm text-destructive">
            ● Recording {state.mode} — {state.elapsed_s.toFixed(1)}s · {state.n_waypoints} pts
            {/* Only claim the arm is limp once torque-off is actually confirmed. */}
            {state.mode === 'kinesthetic' &&
              (state.torque === false ? ' · arm is LIMP, support it' : ' · disabling torque…')}
          </div>
          <div className="flex gap-2">
            <input value={name} onChange={(e) => setName(e.target.value)} placeholder="name…"
              className="flex-1 rounded-sm border border-hairline bg-ink-3 px-2 py-1 text-sm outline-none focus:border-primary/50" />
            <button onClick={onSave} disabled={busy !== null}
              className="rounded-sm bg-primary px-3 py-1 text-sm text-primary-foreground disabled:opacity-40">Stop &amp; Save</button>
            <button onClick={onCancel} disabled={busy !== null}
              className="rounded-sm border border-hairline bg-ink-3 px-3 py-1 text-sm hover:bg-ink-4 disabled:opacity-40">Cancel</button>
          </div>
          {busy && <div className="mt-1 text-xs text-muted-foreground">{busy}</div>}
        </div>
      )}

      <div className="mb-2 flex items-center gap-2 text-xs text-muted-foreground">
        <span>Speed {speed.toFixed(2)}×</span>
        <input type="range" min={0.25} max={2} step={0.05} value={speed}
          onChange={(e) => setSpeed(parseFloat(e.target.value))} className="flex-1" />
        {playing && <button onClick={onStop} disabled={busy !== null}
          className="rounded-sm border border-warning/50 bg-warning/15 px-2 py-0.5 text-xs text-warning disabled:opacity-40">Stop</button>}
      </div>

      {seqs.length === 0 && <div className="text-xs text-muted-foreground">no sequences yet</div>}
      <ul className="flex flex-col gap-1">
        {seqs.map((s) => (
          <li key={s.name} className="flex items-center gap-2 text-sm">
            <span className="flex-1 truncate">
              {s.name} <span className="text-muted-foreground">· {s.mode} · {s.duration_s.toFixed(1)}s · {s.n_waypoints}pt</span>
            </span>
            <button onClick={() => onPlay(s.name)} disabled={disabled || recording || playing || busy !== null}
              className="rounded-sm bg-primary px-2 py-0.5 text-xs text-primary-foreground disabled:opacity-40">Play</button>
            <button onClick={() => onDelete(s.name)} className="rounded-sm border border-destructive/40 bg-destructive/15 px-2 py-0.5 text-xs text-destructive hover:bg-destructive/25">Del</button>
          </li>
        ))}
      </ul>

      {error && <div className="mt-2 text-xs text-destructive">{error}</div>}

      {confirming && (
        <div className="fixed inset-0 z-50 flex items-center justify-center bg-black/60">
          <div className="glass-strong w-80 rounded-sm border border-hairline p-4">
            <h4 className="mb-2 text-sm font-semibold text-destructive">Support the arm</h4>
            <p className="mb-4 text-sm text-muted-foreground">
              Torque will be disabled and the arm will go limp. Hold it before you continue,
              then hand-guide it through the motion. (Same as the calibration flow.)
            </p>
            <div className="flex justify-end gap-2">
              <button onClick={() => setConfirming(false)} className="rounded-sm border border-hairline bg-ink-3 px-3 py-1 text-sm hover:bg-ink-4">Cancel</button>
              <button onClick={doStart} className="rounded-sm bg-destructive px-3 py-1 text-sm font-semibold text-destructive-foreground">Disable torque & record</button>
            </div>
          </div>
        </div>
      )}
    </section>
  )
}
