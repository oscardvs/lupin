/**
 * Tiny router shim. The Shell holds the current tab in local state and
 * already exposes `?tab=` for deep-linking, so cross-cut navigation requests
 * (e.g. Take Control jumping from anywhere → Teleop) just dispatch a window
 * event the Shell listens for.
 *
 * Lifting tab state into a context would work too, but every consumer of the
 * tab id would then re-render on tab change. For one cross-cut the event is
 * cheaper and keeps the Shell as the only place that knows about routing.
 */

const EVENT = 'lupin:gotoTab'

export type LupinTab =
  | 'teleop' | 'arm' | 'voice' | 'cameras' | 'telemetry' | 'logs' | 'map'

export function gotoTab(tab: LupinTab): void {
  if (typeof window === 'undefined') return
  window.dispatchEvent(new CustomEvent(EVENT, { detail: { tab } }))
}

export function onGotoTab(handler: (tab: LupinTab) => void): () => void {
  if (typeof window === 'undefined') return () => undefined
  const wrapped = (e: Event) => {
    const detail = (e as CustomEvent<{ tab: LupinTab }>).detail
    if (detail?.tab) handler(detail.tab)
  }
  window.addEventListener(EVENT, wrapped as EventListener)
  return () => window.removeEventListener(EVENT, wrapped as EventListener)
}
