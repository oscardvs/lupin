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
    try { const r = await startRecording(callService, mode, includeGripper); if (!r.success) setError(r.message) }
    catch (e) { setError(e instanceof Error ? e.message : String(e)) }
  }, [callService, mode, includeGripper])

  const onRecord = useCallback(() => {
    if (mode === 'kinesthetic') setConfirming(true)  // show "support the arm" modal
    else void doStart()
  }, [mode, doStart])

  const onSave = useCallback(async () => {
    const n = name.trim()
    if (!n) { setError('name the sequence first'); return }
    try {
      const r = await saveRecording(callService, n)
      if (!r.success) setError(r.message)
      else { setName(''); await refresh() }
    } catch (e) { setError(e instanceof Error ? e.message : String(e)) }
  }, [callService, name, refresh])

  const onCancel = useCallback(async () => {
    try { await cancelRecording(callService) } catch { /* ignore */ }
  }, [callService])

  const onPlay = useCallback(async (n: string) => {
    setError(null)
    try { const r = await runSequence(callService, n, speed); if (!r.success) setError(r.message) }
    catch (e) { setError(e instanceof Error ? e.message : String(e)) }
  }, [callService, speed])

  const onStop = useCallback(async () => { try { await stopSequence(callService) } catch { /* ignore */ } }, [callService])
  const onDelete = useCallback(async (n: string) => {
    if (!confirm(`Delete sequence "${n}"?`)) return
    try { await deleteEntry(callService, 'sequence', n); await refresh() }
    catch (e) { setError(e instanceof Error ? e.message : String(e)) }
  }, [callService, refresh])

  const recording = state.recording
  const playing = state.playing

  return (
    <section className="rounded-lg border border-white/10 bg-black/20 p-3">
      <h3 className="mb-2 text-sm font-semibold tracking-wide text-white/80">Sequence Recorder</h3>

      {!recording && (
        <div className="mb-2 flex items-center gap-3 text-sm">
          <label className="flex items-center gap-1">
            <input type="radio" checked={mode === 'teleop'} onChange={() => setMode('teleop')} /> Teleop
          </label>
          <label className="flex items-center gap-1">
            <input type="radio" checked={mode === 'kinesthetic'} onChange={() => setMode('kinesthetic')} /> Kinesthetic
          </label>
          <label className="ml-auto flex items-center gap-1 text-xs text-white/60">
            <input type="checkbox" checked={includeGripper} onChange={(e) => setIncludeGripper(e.target.checked)} /> gripper
          </label>
        </div>
      )}

      {!recording ? (
        <button onClick={onRecord} disabled={disabled || playing}
          className="mb-3 w-full rounded bg-rose-600/80 px-3 py-1.5 text-sm font-semibold disabled:opacity-40">
          ● Record {mode}
        </button>
      ) : (
        <div className="mb-3 rounded bg-rose-950/40 p-2">
          <div className="mb-2 text-sm text-rose-300">
            ● Recording {state.mode} — {state.elapsed_s.toFixed(1)}s · {state.n_waypoints} pts
            {state.mode === 'kinesthetic' && ' · arm is LIMP, support it'}
          </div>
          <div className="flex gap-2">
            <input value={name} onChange={(e) => setName(e.target.value)} placeholder="name…"
              className="flex-1 rounded bg-white/5 px-2 py-1 text-sm outline-none" />
            <button onClick={onSave} className="rounded bg-emerald-600/80 px-3 py-1 text-sm">Stop & Save</button>
            <button onClick={onCancel} className="rounded bg-white/10 px-3 py-1 text-sm">Cancel</button>
          </div>
        </div>
      )}

      <div className="mb-2 flex items-center gap-2 text-xs text-white/60">
        <span>Speed {speed.toFixed(2)}×</span>
        <input type="range" min={0.25} max={2} step={0.05} value={speed}
          onChange={(e) => setSpeed(parseFloat(e.target.value))} className="flex-1" />
        {playing && <button onClick={onStop} className="rounded bg-amber-600/70 px-2 py-0.5 text-xs">Stop</button>}
      </div>

      {seqs.length === 0 && <div className="text-xs text-white/30">no sequences yet</div>}
      <ul className="flex flex-col gap-1">
        {seqs.map((s) => (
          <li key={s.name} className="flex items-center gap-2 text-sm">
            <span className="flex-1 truncate">
              {s.name} <span className="text-white/40">· {s.mode} · {s.duration_s.toFixed(1)}s · {s.n_waypoints}pt</span>
            </span>
            <button onClick={() => onPlay(s.name)} disabled={disabled || recording || playing}
              className="rounded bg-sky-600/70 px-2 py-0.5 text-xs disabled:opacity-40">Play</button>
            <button onClick={() => onDelete(s.name)} className="rounded bg-rose-600/60 px-2 py-0.5 text-xs">Del</button>
          </li>
        ))}
      </ul>

      {error && <div className="mt-2 text-xs text-rose-400">{error}</div>}

      {confirming && (
        <div className="fixed inset-0 z-50 flex items-center justify-center bg-black/60">
          <div className="w-80 rounded-lg border border-white/10 bg-zinc-900 p-4">
            <h4 className="mb-2 text-sm font-semibold text-rose-300">Support the arm</h4>
            <p className="mb-4 text-sm text-white/70">
              Torque will be disabled and the arm will go limp. Hold it before you continue,
              then hand-guide it through the motion. (Same as the calibration flow.)
            </p>
            <div className="flex justify-end gap-2">
              <button onClick={() => setConfirming(false)} className="rounded bg-white/10 px-3 py-1 text-sm">Cancel</button>
              <button onClick={doStart} className="rounded bg-rose-600 px-3 py-1 text-sm font-semibold">Disable torque & record</button>
            </div>
          </div>
        </div>
      )}
    </section>
  )
}
