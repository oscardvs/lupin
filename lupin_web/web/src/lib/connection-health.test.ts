import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'

import {
  createDisconnectGuard,
  isPongStale,
  shouldReconnectOnVisible,
} from './connection-health'

describe('createDisconnectGuard (debounced rosbridge-disconnect e-stop)', () => {
  beforeEach(() => vi.useFakeTimers())
  afterEach(() => vi.useRealTimers())

  it('does not latch when a brief disconnect recovers within the grace window', () => {
    const onLatch = vi.fn()
    const guard = createDisconnectGuard({ graceMs: 5000, onLatch })
    guard.onStatus('connecting')
    guard.onStatus('connected')
    // An intentional forced reconnect blips through 'closed' → 'connecting':
    guard.onStatus('closed')
    vi.advanceTimersByTime(1000)
    guard.onStatus('connecting')
    guard.onStatus('connected')
    vi.advanceTimersByTime(10_000)
    expect(onLatch).not.toHaveBeenCalled()
  })

  it('latches when a disconnect persists beyond the grace window', () => {
    const onLatch = vi.fn()
    const guard = createDisconnectGuard({ graceMs: 5000, onLatch })
    guard.onStatus('connected')
    guard.onStatus('closed')
    vi.advanceTimersByTime(4999)
    expect(onLatch).not.toHaveBeenCalled()
    vi.advanceTimersByTime(2)
    expect(onLatch).toHaveBeenCalledTimes(1)
  })

  it('does not let intermediate "connecting" attempts postpone the latch (flapping outage)', () => {
    const onLatch = vi.fn()
    const guard = createDisconnectGuard({ graceMs: 5000, onLatch })
    guard.onStatus('connected')
    guard.onStatus('closed') // t=0: outage begins
    vi.advanceTimersByTime(2000) // t=2
    guard.onStatus('connecting') // reconnect attempt — must NOT reset the timer
    vi.advanceTimersByTime(2000) // t=4
    guard.onStatus('error') // attempt failed
    vi.advanceTimersByTime(1001) // t=5.001 > grace measured from first disconnect
    expect(onLatch).toHaveBeenCalledTimes(1)
  })

  it('never latches if it was never connected (initial connect handshake)', () => {
    const onLatch = vi.fn()
    const guard = createDisconnectGuard({ graceMs: 5000, onLatch })
    guard.onStatus('connecting')
    guard.onStatus('error')
    guard.onStatus('closed')
    vi.advanceTimersByTime(60_000)
    expect(onLatch).not.toHaveBeenCalled()
  })

  it('latches only once for a single sustained outage', () => {
    const onLatch = vi.fn()
    const guard = createDisconnectGuard({ graceMs: 5000, onLatch })
    guard.onStatus('connected')
    guard.onStatus('closed')
    vi.advanceTimersByTime(6000)
    guard.onStatus('connecting')
    guard.onStatus('closed')
    vi.advanceTimersByTime(6000)
    expect(onLatch).toHaveBeenCalledTimes(1)
  })

  it('re-arms after a recovery: a later sustained outage still latches', () => {
    const onLatch = vi.fn()
    const guard = createDisconnectGuard({ graceMs: 5000, onLatch })
    guard.onStatus('connected')
    guard.onStatus('closed') // brief blip
    vi.advanceTimersByTime(1000)
    guard.onStatus('connected') // recovered — no latch
    expect(onLatch).not.toHaveBeenCalled()
    guard.onStatus('closed') // a genuinely new outage
    vi.advanceTimersByTime(5001)
    expect(onLatch).toHaveBeenCalledTimes(1)
  })

  it('dispose() cancels a pending latch (component unmount)', () => {
    const onLatch = vi.fn()
    const guard = createDisconnectGuard({ graceMs: 5000, onLatch })
    guard.onStatus('connected')
    guard.onStatus('closed')
    vi.advanceTimersByTime(2000)
    guard.dispose()
    vi.advanceTimersByTime(10_000)
    expect(onLatch).not.toHaveBeenCalled()
  })
})

describe('isPongStale', () => {
  it('is false when the last successful pong is within the threshold', () => {
    expect(isPongStale(1000, 3000, 4000)).toBe(false) // 2s old, 4s threshold
  })

  it('is true when no pong has landed within the threshold', () => {
    expect(isPongStale(1000, 6000, 4000)).toBe(true) // 5s old, 4s threshold
  })
})

describe('shouldReconnectOnVisible', () => {
  it('reconnects when the socket is not in a connected state', () => {
    expect(shouldReconnectOnVisible('closed', 0, 0, 3000)).toBe(true)
  })

  it('reconnects when nominally connected but pongs went stale (timers throttled while hidden)', () => {
    expect(shouldReconnectOnVisible('connected', 1000, 10_000, 3000)).toBe(true)
  })

  it('does not reconnect on a quick tab-switch with a fresh pong', () => {
    expect(shouldReconnectOnVisible('connected', 1000, 2000, 3000)).toBe(false)
  })
})
