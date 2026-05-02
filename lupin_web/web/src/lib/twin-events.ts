/**
 * Tiny window-event shim for cross-component twin interactions.
 *
 * Used so that the Greenhouse State table can ask Map / Nav to highlight
 * a tag without lifting state into a context — the canvas listens for the
 * event and pulses a ring around the requested tag for ~1.5 s. Same shape
 * as `navigation.ts`'s gotoTab helper.
 */

const PULSE_TAG_EVENT = 'lupin:pulseTag'

export function pulseTag(tagId: string): void {
  if (typeof window === 'undefined') return
  window.dispatchEvent(new CustomEvent(PULSE_TAG_EVENT, { detail: { tagId } }))
}

export function onPulseTag(handler: (tagId: string) => void): () => void {
  if (typeof window === 'undefined') return () => undefined
  const wrapped = (e: Event) => {
    const detail = (e as CustomEvent<{ tagId: string }>).detail
    if (detail?.tagId) handler(detail.tagId)
  }
  window.addEventListener(PULSE_TAG_EVENT, wrapped as EventListener)
  return () => window.removeEventListener(PULSE_TAG_EVENT, wrapped as EventListener)
}
