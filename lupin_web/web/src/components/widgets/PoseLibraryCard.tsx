import { useCallback, useEffect, useState } from 'react'
import { useRos } from '@/lib/ros'
import {
  fetchLibrary, savePose, gotoPose, deleteEntry, renameEntry,
  BUILTIN_PRESETS,
} from '@/lib/armLibrary'
import type { ArmLibraryPoseMeta } from '@/types/ros'

/** Save / recall named arm poses. Built-in presets are shown read-only; user
 * poses (owned by arm_library_server, persisted to laptop JSON) are editable. */
export function PoseLibraryCard({ disabled }: { disabled: boolean }) {
  const { callService } = useRos()
  const [poses, setPoses] = useState<ArmLibraryPoseMeta[]>([])
  const [name, setName] = useState('')
  const [busy, setBusy] = useState(false)
  const [error, setError] = useState<string | null>(null)

  const refresh = useCallback(async () => {
    try {
      const lib = await fetchLibrary(callService)
      setPoses(lib.poses)
    } catch (e) {
      setError(e instanceof Error ? e.message : String(e))
    }
  }, [callService])

  useEffect(() => { void refresh() }, [refresh])

  const onSave = useCallback(async () => {
    const n = name.trim()
    if (!n) return
    setBusy(true); setError(null)
    try {
      const res = await savePose(callService, n)
      if (!res.success) setError(res.message)
      else { setName(''); await refresh() }
    } catch (e) { setError(e instanceof Error ? e.message : String(e)) }
    finally { setBusy(false) }
  }, [callService, name, refresh])

  const onGoto = useCallback(async (n: string) => {
    setError(null)
    try { const r = await gotoPose(callService, n); if (!r.success) setError(r.message) }
    catch (e) { setError(e instanceof Error ? e.message : String(e)) }
  }, [callService])

  const onDelete = useCallback(async (n: string) => {
    if (!confirm(`Delete pose "${n}"?`)) return
    try { await deleteEntry(callService, 'pose', n); await refresh() }
    catch (e) { setError(e instanceof Error ? e.message : String(e)) }
  }, [callService, refresh])

  const onRename = useCallback(async (n: string) => {
    const nn = prompt(`Rename "${n}" to:`, n)
    if (!nn || nn === n) return
    try { const r = await renameEntry(callService, 'pose', n, nn); if (!r.success) setError(r.message); await refresh() }
    catch (e) { setError(e instanceof Error ? e.message : String(e)) }
  }, [callService, refresh])

  return (
    <section className="rounded-lg border border-white/10 bg-black/20 p-3">
      <h3 className="mb-2 text-sm font-semibold tracking-wide text-white/80">Pose Library</h3>

      <div className="mb-3 flex gap-2">
        <input
          value={name} onChange={(e) => setName(e.target.value)}
          placeholder="name this pose…" disabled={disabled || busy}
          className="flex-1 rounded bg-white/5 px-2 py-1 text-sm text-white outline-none"
        />
        <button onClick={onSave} disabled={disabled || busy || !name.trim()}
          className="rounded bg-emerald-600/80 px-3 py-1 text-sm font-medium disabled:opacity-40">
          Save current
        </button>
      </div>

      <div className="mb-1 text-xs uppercase text-white/40">Built-in</div>
      <div className="mb-3 flex flex-wrap gap-1">
        {BUILTIN_PRESETS.map((b) => (
          <button key={b} onClick={() => onGoto(b)} disabled={disabled}
            className="rounded bg-white/5 px-2 py-1 text-xs hover:bg-white/10 disabled:opacity-40">
            {b}
          </button>
        ))}
      </div>

      <div className="mb-1 text-xs uppercase text-white/40">Saved</div>
      {poses.length === 0 && <div className="text-xs text-white/30">none yet</div>}
      <ul className="flex flex-col gap-1">
        {poses.map((p) => (
          <li key={p.name} className="flex items-center gap-2 text-sm">
            <span className="flex-1 truncate">{p.name}{p.has_gripper ? ' ·✊' : ''}</span>
            <button onClick={() => onGoto(p.name)} disabled={disabled}
              className="rounded bg-sky-600/70 px-2 py-0.5 text-xs disabled:opacity-40">Go</button>
            <button onClick={() => onRename(p.name)} className="rounded bg-white/5 px-2 py-0.5 text-xs">Rename</button>
            <button onClick={() => onDelete(p.name)} className="rounded bg-rose-600/60 px-2 py-0.5 text-xs">Del</button>
          </li>
        ))}
      </ul>

      {error && <div className="mt-2 text-xs text-rose-400">{error}</div>}
    </section>
  )
}
