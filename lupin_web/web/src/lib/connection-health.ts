// Framework-agnostic rosbridge connection-health logic, kept out of ros.tsx /
// estop.tsx so it can be unit-tested with fake timers (no React, no ROSLIB).

// Mirrors ros.tsx's RosStatus literals; defined here to avoid importing the
// heavy ROS provider module (React + roslib) into pure-logic tests.
export type ConnectionStatus = 'connecting' | 'connected' | 'closed' | 'error'

/**
 * True when the last successful latency pong is older than `thresholdMs`. Used
 * both by the ping loop (zombie-socket detection) and the visibility handler
 * (a backgrounded tab freezes the ping timer, so `lastPongAt` goes stale even
 * if the socket never emitted a close event).
 */
export function isPongStale(lastPongAt: number, now: number, thresholdMs: number): boolean {
  return now - lastPongAt > thresholdMs
}

/**
 * On returning to a backgrounded tab, force a fresh socket when we're not
 * cleanly connected, or when pongs have gone stale. A quick alt-tab with a
 * recent pong needs no reconnect.
 */
export function shouldReconnectOnVisible(
  status: ConnectionStatus,
  lastPongAt: number,
  now: number,
  thresholdMs: number,
): boolean {
  if (status !== 'connected') return true
  return isPongStale(lastPongAt, now, thresholdMs)
}

export interface DisconnectGuardOptions {
  /** Grace window (ms) from the first disconnect before the e-stop latches. */
  graceMs: number
  /** Invoked once when a disconnect has persisted past the grace window. */
  onLatch: () => void
}

export interface DisconnectGuard {
  /** Feed every rosbridge status transition here. */
  onStatus: (status: ConnectionStatus) => void
  /** Cancel any pending latch (call on unmount). */
  dispose: () => void
}

/**
 * Debounces the rosbridge-disconnect e-stop so an intentional/brief reconnect
 * (forced reconnect on tab return, a NAT blip) does NOT latch, while a genuine
 * sustained outage still does.
 *
 * Behaviour:
 *  - The grace timer starts on the FIRST disconnect ('closed'/'error') and is
 *    NOT reset by intermediate 'connecting' states — so a flapping connection
 *    that never actually reconnects still latches one grace-window after the
 *    outage began (the load-bearing safety property).
 *  - A real 'connected' cancels the pending latch and re-arms the guard, so a
 *    fast reconnect is invisible and a *later* outage latches again.
 *  - It never latches before the socket has connected at least once (the
 *    initial connect handshake cycles through 'connecting'/'error').
 *  - It latches at most once per outage (until the next recovery).
 */
export function createDisconnectGuard({ graceMs, onLatch }: DisconnectGuardOptions): DisconnectGuard {
  let everConnected = false
  let latched = false
  let timer: ReturnType<typeof setTimeout> | null = null

  const clearTimer = () => {
    if (timer != null) {
      clearTimeout(timer)
      timer = null
    }
  }

  return {
    onStatus(status: ConnectionStatus) {
      if (status === 'connected') {
        everConnected = true
        latched = false // recovered — re-arm for a future outage
        clearTimer()
        return
      }
      if (status === 'connecting') {
        // An in-progress (re)connect attempt must not postpone a pending latch.
        return
      }
      // 'closed' | 'error' — a disconnect. Schedule the latch on the FIRST one
      // only (don't reschedule on subsequent flaps, don't re-latch this outage).
      if (everConnected && !latched && timer == null) {
        timer = setTimeout(() => {
          timer = null
          latched = true
          onLatch()
        }, graceMs)
      }
    },
    dispose() {
      clearTimer()
    },
  }
}
