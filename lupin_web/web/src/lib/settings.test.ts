// @vitest-environment jsdom
import { beforeEach, describe, expect, it, vi } from 'vitest'

// The settings module runs its migration once, at module-evaluation time (the
// `cached` IIFE). So each case seeds localStorage, resets the module registry,
// then dynamically imports a fresh copy and inspects the resolved settings —
// exercising the real readStored() migration path end to end.
describe('settings migration: estopAutoOnFocusLoss', () => {
  beforeEach(() => {
    localStorage.clear()
    vi.resetModules()
  })

  it('defaults estopAutoOnFocusLoss to false on a fresh install', async () => {
    const { getSettings } = await import('./settings')
    expect(getSettings().estopAutoOnFocusLoss).toBe(false)
  })

  it('drops a persisted estopAutoOnFocusLoss=true so the new default (false) applies', async () => {
    // A pre-existing install (current STORAGE_KEY) that has the old default
    // persisted. The migration must drop it so focus-loss e-stop is off.
    localStorage.setItem(
      'lupin-hmi-settings/v4',
      JSON.stringify({ estopAutoOnFocusLoss: true, speedScale: 0.7 }),
    )
    const { getSettings } = await import('./settings')
    expect(getSettings().estopAutoOnFocusLoss).toBe(false)
    // Unrelated settings still migrate forward untouched.
    expect(getSettings().speedScale).toBe(0.7)
  })

  it('preserves an explicit estopAutoOnFocusLoss=true saved under the current key', async () => {
    // Re-enabling the toggle persists under STORAGE_KEY; that opt-in must be
    // read back verbatim, not dropped (the migration only strips legacy keys).
    localStorage.setItem(
      'lupin-hmi-settings/v5',
      JSON.stringify({ estopAutoOnFocusLoss: true }),
    )
    const { getSettings } = await import('./settings')
    expect(getSettings().estopAutoOnFocusLoss).toBe(true)
  })
})
