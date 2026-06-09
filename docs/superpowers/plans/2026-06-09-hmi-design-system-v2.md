# Lupin HMI — Design-System v2 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. This is a **visual frontend refactor** — "verification" means measuring overflow + reading screenshots in the live mock HMI (Chrome DevTools MCP), not unit tests.

**Goal:** Make the HMI feel like a native app — balanced composition with no wasted space, stronger dark-mode figure-ground, single-screen fit on the two primary surfaces, responsive everywhere, and in-app expand+zoom/pan for the vision windows (map / camera / 3D twin).

**Architecture:** Fix root causes once, globally (Phase 1: elevation tokens + opaque base tiles + a `ViewShell` height-budget primitive + an `xl` tier + container queries + density scale), then recompose each view to consume reclaimed space and fit phone (Phase 2), then build ONE shared `FocusPanel` (in-shell maximize dialog with zoom/pan) and apply it to map/camera/twin/lidar/logs (Phase 3). Every change uses existing design tokens and decorative utilities; we are *returning to* the system's own "glass = floating overlays only" rule, not adding new aesthetics.

**Tech Stack:** React 18, Vite 6, TypeScript, Tailwind 3.4, Radix UI (shadcn), framer-motion, @react-three/fiber + drei, roslib. Worktree: `/home/oskrt/worktrees/hmi-design-v2`, branch `feat/hmi-design-system-v2`, package dir `lupin_web/web`.

**Primary target surfaces (decided with the operator):**
- **Laptop** ≈ `1440×900` logical (demo + dev). Goal: kill voids, densify, fit one screen.
- **iPhone 12 Pro Max** = `428×926` logical, DPR 3 (most-shown phone). Goal: fit one screen for control+vision views.
- **Best-effort for all others** (teammates use tablet-landscape 1194×834, tablet-portrait, small laptops) — must not regress.

**Fit policy (decided):** Strict single-screen on **control + vision** views (Teleop, Map/Nav, Cameras, Telemetry); **internal scroll allowed** on inherently-long *feeds* (Arm joint list, Logs stream, Voice transcript). "Fit" = the ONE primary panel grows to fill; secondary content collapses or scrolls internally; the page itself never scrolls.

**Expand model (decided):** **In-app** `FocusPanel` (Radix Dialog, `.glass-strong`, 90vw×90vh) — E-stop + Take-Control stay visible. **Replaces** the native `requestFullscreen` on the camera.

---

## Guardrails (do not violate — these are the design system's own rules)

1. Keep the dark instrument-terminal aesthetic: Instrument Serif italic display, Geist body, JetBrains Mono `.tag`/`.ticker`, reticle L-brackets, scanline, edge-light, noise, 0.25rem radius.
2. Status lanes stay disjoint: **chartreuse=nominal, cyan=data, amber=caution, red=critical.** Any rebuilt card (Arm Pose/Sequence) maps onto these — **no emerald/sky/rose/raw-white.**
3. **Glass (`.glass`/`.glass-strong`) is for FLOATING overlays only** (drawers, dialogs, top rail, FocusPanel control rail) — never base tiles. Translucent base tiles (`bg-card/35`, `bg-card/85`, `bg-background/40`, `backdrop-blur` on Card) currently violate this and **must become opaque ink steps.**
4. Contrast fixes are **elevation / edge / interference only** — hold hue+sat constant, move only Lightness. **Do not brighten body text** (already AA ≈7:1).
5. Use ink-ramp tokens (`--ink-0..4`) + existing decorative classes; promote needed one-offs into reusable utilities.
6. Preserve honest-data discipline (staleness tags, age-desaturated pins, dashed placeholders, real empty states).
7. Preserve decoupled-render architecture: ref-driven loops (20 Hz cmd_vel, `useThrottledRender`, `missionLockedRef`) and the single `MapCanvas` projection / `worldToCanvas` funnel. Thread zoom/pan through **refs**, not per-frame React state.
8. Keep touch-safe: E-stop / Take Control / drive / STOP reachable + ≥44px on touch; camera maximize stays **in-shell**.
9. Respect a11y fallbacks (`prefers-reduced-motion`, `prefers-reduced-transparency`→solid, `prefers-contrast`, aurora→CSS) and GPU discipline (DPR clamp, `frameloop` pause on hidden, PerformanceMonitor degrade).
10. The shell height contract (App.tsx `h-full min-h-0` column, one `overflow-y-auto` TabsContent) is correct — build ON it with `ViewShell`, don't loosen it.

---

## Verification protocol (run after every task that changes layout/visuals)

Dev server: worktree `npm run dev` on `http://localhost:8090/?mock` (already running).

**A. Overflow / fit check** (Chrome MCP `evaluate_script`): for the active view at a viewport, assert the panel doesn't exceed the viewport for "fit" views.
```js
() => { const p = document.querySelector('[role="tabpanel"][data-state="active"]');
  return { vw: innerWidth, vh: innerHeight, scrollH: p.scrollHeight, clientH: p.clientHeight,
           overflow: Math.max(0, p.scrollHeight - p.clientHeight) }; }
```
**B. Two primary viewports** via `emulate`: laptop `1440x900x1`; iPhone 12 Pro Max `428x926x3,mobile,touch`. Spot-check tablet-landscape `1194x834x2,touch` for regressions.
**C. Screenshot** each touched view to `/tmp/hmi-review/shots/v2-<view>-<vp>.png` and Read it — confirm fill/balance + figure-ground vs the pre-change shot in `/tmp/hmi-review/shots/`.
**D. Contrast spot-check**: re-run the elevation delta — base vs card vs tile should now read as ≥1.4:1 luminance steps and panel edges should be visible.
**E. Build gate** (before each phase boundary commit): `npm run build` (tsc -b && vite build) must be green.

**Fit expectations after Phase 2** (laptop must fully fit; iPhone must fit for fit-class):
| View | Laptop 1440×900 | iPhone 428×926 |
|---|---|---|
| Teleop | fit, filled | fit (2-up dials) |
| Arm | fit, twin beside console | fit shell; joints scroll internally |
| Voice | fit | fit shell; transcript scrolls internally |
| Cameras | fit, stream fills | fit |
| Telemetry | fit, right-rail fills Lidar height | fit shell; sensor grid scrolls internally |
| Logs | fit, stream fills | fit shell; log scrolls internally |
| Map/Nav | fit, all rows on-screen | fit |

---

## File structure

**New files**
- `lupin_web/web/src/components/system/ViewShell.tsx` — height-budget layout primitive (`fit`/`flow`).
- `lupin_web/web/src/components/system/FocusPanel.tsx` — shared in-shell maximize dialog (Radix Dialog, glass-strong rail) + a `useFocusPanel()` hook/context.
- `lupin_web/web/src/components/ui/ExpandButton.tsx` — `.tag`-styled Maximize2 icon button used in every vision panel header.
- `lupin_web/web/src/lib/zoompan.ts` — small shared zoom/pan reducer + wheel/pinch/drag handlers (clamped), reused by Map, Camera (and the FocusPanel toolbar).

**Modified — foundations**
- `lupin_web/web/src/index.css` — ink ramp re-pitch, hairline, `--edge-tile`, density vars, aurora floor, delete redundant body grid, `.contrast-high`/`.opaque` hooks.
- `lupin_web/web/tailwind.config.ts` — `xl` already implicit; add `@tailwindcss/container-queries`; expose `ink` bg utilities + density spacing.
- `lupin_web/web/src/components/ui/card.tsx` — opaque base, no blur, stronger inset/edge.
- `lupin_web/web/src/lib/responsive.ts` — `useIsXl`, `useIsPhone` (max-width).
- `lupin_web/web/src/lib/settings.ts` + `components/SettingsDrawer.tsx` — `surfaceContrast`, `reduceTransparency`.
- `lupin_web/web/src/components/system/AuroraBackground.tsx` — raise contrast floor, lower grid opacity.

**Modified — per view (Phase 2)**: `components/views/{Teleop,Arm,Voice,Cameras,Telemetry,Logs,MapView}.tsx` + their widgets (`Joystick`, `TwistReadout`, `ArmJointsCard`, `PoseLibraryCard`, `SequenceRecorderCard`, `BatteryCard`/`ImuCard`/`OdometryCard`/`SystemCard`/`ObservationsCard`/`GreenhouseStateCard`, `CameraStream`, `MapCanvas`, `MissionControls`, `LightControl`).

**Modified — vision (Phase 3)**: `MapCanvas.tsx`, `CameraStream.tsx`, `system/RobotTwin.tsx`, `ArmView.tsx`, `TelemetryView.tsx` (Lidar), `LogsView.tsx`.

---

## PHASE 1 — Global foundations

### Task 1: Re-pitch the ink elevation ramp + hairline + edge-tile token
**Files:** Modify `lupin_web/web/src/index.css` (`:root` block ~lines 37–49, `--hairline` line 30).

- [ ] Widen the ramp (hold hue/sat; move only L). Dark `:root`:
  - `--ink-0: 120 14% 3%;` (was 3.5)
  - `--ink-1: 120 14% 4%;` (was 4.5) and set `--background: 120 14% 4%;`
  - `--ink-2: 120 10% 9.5%;` (was 7) and set `--card: 120 10% 9.5%;`
  - `--ink-3: 120 10% 12.5%;` (was 9)
  - `--ink-4: 120 12% 15%;` (was 11) ; `--glass-plate`/`--popover` follow if needed.
  - `--secondary: 120 8% 14%;` `--muted: 120 8% 13%;` (nudge up so secondary surfaces track the new card).
  - `--hairline: 120 6% 25%;` (was 18) ; `--border: 120 8% 18%;` (was 14).
  - Add `--edge-tile: 60 30% 88%;` (reuse glass-edge hue) for a 1px inset top highlight on tiles.
- [ ] **Verify (A/D)** at laptop on Telemetry: card-vs-background luminance ratio ≥1.4:1; panel edges visibly separate. Screenshot `v2-telemetry-laptop.png`, Read, compare to `tl-telemetry.png`.
- [ ] **Commit:** `style(hmi): re-pitch ink elevation ramp + firmer hairline for dark-mode figure-ground`

### Task 2: Make the base Card opaque (return glass to overlays-only)
**Files:** Modify `lupin_web/web/src/components/ui/card.tsx` (lines ~9–11).

- [ ] Change Card root surface from `bg-card/85` + `backdrop-blur-[2px]` to **solid** `bg-card` (no blur). Strengthen the inset accent from `primary/0.04` to `0.06` and add a 1px inset top highlight: `shadow-[inset_0_1px_0_0_hsl(var(--edge-tile)/0.05),inset_0_0_0_1px_hsl(var(--primary)/0.06)]` (keep existing border-hairline).
- [ ] Add an optional `edge?: boolean` prop that applies the `.edge-light` class (for hero/vision cards).
- [ ] **Verify** at laptop+iPhone on Voice + Teleop: console no longer shows aurora bleeding through; tiles read as solid objects. Screenshot both.
- [ ] **Commit:** `style(hmi): opaque base Card, no backdrop-blur (glass is overlay-only)`

### Task 3: Reduce background interference (aurora floor up, grid down, delete redundant body grid)
**Files:** Modify `lupin_web/web/src/index.css` (body `::before` grid ~147–161) and `components/system/AuroraBackground.tsx`.

- [ ] Delete the `body::before` drifting grid block (index.css ~147–161) — it double-textures behind opaque tiles now and only muddies the gaps. Keep the corner-bloom + floor-vignette `body` background.
- [ ] In `AuroraBackground.tsx`: raise the dark contrast floor (the base dim multiplier) so content sits on a darker field — find the `tone==='idle'/'active'` intensity and the base opacity; reduce overall aurora opacity ~35% and any internal grid alpha to ~0.32. (Read the file first; adjust the documented intensity constants, not the shader structure.)
- [ ] **Verify** at laptop on Map + Teleop: gaps between tiles read as a calm dark field, tiles pop. No regression to `prefers-reduced-motion` CSS fallback.
- [ ] **Commit:** `style(hmi): calm the aurora+grid behind opaque tiles for cleaner figure-ground`

### Task 4: Responsive density — xl hook, container queries, density vars
**Files:** Modify `lib/responsive.ts`, `tailwind.config.ts`, `index.css`.

- [ ] `responsive.ts`: add `export const useIsXl = () => useMediaQuery('(min-width: 1280px)')` and `export const useIsPhone = () => useMediaQuery('(max-width: 639px)')`.
- [ ] `tailwind.config.ts`: `import containerQueries from '@tailwindcss/container-queries'` and add to `plugins`. (Install: `npm i -D @tailwindcss/container-queries`.) Add `ink` background color utilities so views can use `bg-ink-2/3/4`: extend `colors` with `'ink-0'..'ink-4': 'hsl(var(--ink-N))'`.
- [ ] `index.css` `:root`: add density vars `--pad: 0.75rem; --gap: 0.75rem;` with `@media (min-width:640px){--pad:1rem;--gap:1rem}` and `@media (min-width:1280px){--pad:1.25rem;--gap:1.25rem}` (scoped on `:root`/`html`).
- [ ] **Verify:** `npm run build` green; `bg-ink-3` resolves (quick grep/use in a temp class). No visual change yet.
- [ ] **Commit:** `feat(hmi): xl breakpoint, container-queries, ink utilities + density scale`

### Task 5: `ViewShell` height-budget primitive
**Files:** Create `components/system/ViewShell.tsx`.

- [ ] Implement:
```tsx
import { cn } from '@/lib/utils'
import type { ReactNode } from 'react'
/** Every view roots in ViewShell. `fit` = page never scrolls; children distribute
 *  the remaining height (give the ONE primary child `flex-1 min-h-0`, secondary
 *  content collapses or scrolls internally). `flow` = a single internal scroll
 *  region for feed-style views. Owns the standard pad/gap scale. */
export function ViewShell({ intent = 'fit', className, children }:
  { intent?: 'fit' | 'flow'; className?: string; children: ReactNode }) {
  return (
    <div className={cn(
      'flex h-full min-h-0 w-full flex-col',
      'gap-[var(--gap)] p-[var(--pad)]',
      intent === 'fit' ? 'overflow-hidden' : 'overflow-y-auto',
      className,
    )}>{children}</div>
  )
}
```
- [ ] **Verify:** import into one view (Cameras) as a smoke test, build green. Revert the smoke import or keep if clean.
- [ ] **Commit:** `feat(hmi): ViewShell fit/flow height-budget layout primitive`

### Task 6: In-app contrast / transparency settings
**Files:** Modify `lib/settings.ts`, `components/SettingsDrawer.tsx` (and `useApplyTheme`).

- [ ] `settings.ts`: add `surfaceContrast: 'normal' | 'high'` and `reduceTransparency: boolean` to the settings type + defaults + persistence. In `useApplyTheme`, toggle `document.documentElement.classList` `contrast-high` / `opaque` from these.
- [ ] `index.css`: key the existing `prefers-contrast`/`prefers-reduced-transparency` rule bodies ALSO off `.contrast-high`/`.opaque` (turn the two media blocks into shared selectors so manual toggles reuse them).
- [ ] `SettingsDrawer.tsx`: add two Switch rows ("High contrast surfaces", "Reduce transparency") under the existing theme controls.
- [ ] **Verify** at laptop: toggling High contrast firms borders; Reduce transparency makes drawers/dialogs solid. Build green.
- [ ] **Commit:** `feat(hmi): operator contrast + reduce-transparency toggles (reuse a11y CSS)`

### Task 7: Phase-1 global verification + checkpoint
- [ ] Re-measure overflow (A) for all 7 tabs at laptop and iPhone; record before→after. Screenshot Telemetry + Teleop + Map at both viewports.
- [ ] `npm run build` green. Commit any token tidy-ups.
- [ ] **CHECK IN with operator** with before/after screenshots before starting Phase 2.

---

## PHASE 2 — Per-view recomposition

> Each task: adopt `ViewShell`, swap translucent/ inverted base tiles for opaque `bg-ink-2/3`, make the ONE primary panel `flex-1 min-h-0`, recompose to fill space + fit phone per the audit, enforce ≥44px touch targets, then Verify (A/B/C) at laptop + iPhone + tablet-landscape, then Commit.

### Task 8: Teleop
**Files:** `views/TeleopView.tsx`, `widgets/Joystick.tsx`, `widgets/TwistReadout.tsx`.
- [ ] Root → `ViewShell intent="fit"`. Console panel → `flex-1 min-h-0`, surface `bg-ink-2`; joystick + readout tiles → `bg-ink-3` (drop `bg-card/35`, `/40`, `/70`).
- [ ] Replace `sm:justify-around` with `lg:grid lg:grid-cols-[1fr_1fr_minmax(14rem,18rem)] lg:items-stretch`; group the two dials under a shared `STICKS` sub-frame, readout as `OUTPUT` column (hairline divider). Scale `stickSize` up on lg/xl (`isXl ? 240 : isLg ? 200 : isSm ? 176 : 150`); decouple phone size from `sm`.
- [ ] iPhone: 2-up dial grid at ~150px + readout below + governor/STOP on one row → fits 926px. Banners cap to one-line pills when stacked.
- [ ] Verify (target: laptop filled no-void; iPhone overflow 0). Commit `feat(hmi): recompose Teleop — grid-fill console, opaque tiles, phone 2-up dials`.

### Task 9: Arm (incl. design-system rebuild of Pose/Sequence cards)
**Files:** `views/ArmView.tsx`, `widgets/ArmJointsCard.tsx`, `widgets/PoseLibraryCard.tsx`, `widgets/SequenceRecorderCard.tsx`, `system/RobotTwin.tsx` (height only).
- [ ] Root → `ViewShell intent="fit"`. Desktop: twin beside console via `lg:grid lg:grid-cols-[1.4fr_1fr]`; joints `lg:grid-cols-2 xl:grid-cols-3` (kills orphan 5th). Twin height responsive `h-[176px] sm:h-[300px] lg:h-full`.
- [ ] **Rebuild `PoseLibraryCard` + `SequenceRecorderCard`** onto `Card/CardHeader/CardContent` (`bg-ink-2`, border-hairline, reticle); replace emerald/sky/rose/white with tokens (primary=affirmative, `text-destructive`+`destructive/15`=delete, warning=stop, cyan `.tag-accent`=meta). Replace native `prompt()`/`confirm()` with the Radix Dialog; native radio/checkbox/range → Radix Slider/Checkbox/RadioGroup; touch targets ≥44px.
- [ ] Joint cards → opaque `bg-ink-2`/step inner ghost-tick to readable alpha.
- [ ] iPhone (**flow allowed for joints**): twin ~176px + console + `Joints | Poses | Sequences` sub-tab strip (or accordion) where the joints list scrolls internally; page shell fits.
- [ ] Verify. Commit `feat(hmi): recompose Arm — twin beside console, 3-col joints, tokenized Pose/Sequence cards`.

### Task 10: Voice
**Files:** `views/VoiceView.tsx`, `widgets/VoiceOrb.tsx`.
- [ ] Root → `ViewShell intent="fit"`. Console → opaque `bg-ink-2` (drop `bg-card/35`); recessed ScrollArea wells → `bg-ink-1`/`ink-0`. Confine `.scanline` to the orb region (off the transcript body).
- [ ] Two-column layout at `md:` (not lg). Transcript = the single `flex-1 min-h-0` internal scroll region (chat pattern). Tighten left column gaps; richer empty-transcript state to fill the void.
- [ ] iPhone (**transcript scrolls internally**): orb ~160px, Tool Calls = closed accordion, transcript fills remaining height; banners cap to a pill row. Shell fits.
- [ ] Verify. Commit `feat(hmi): recompose Voice — md two-col, opaque console, transcript fills`.

### Task 11: Cameras
**Files:** `views/CamerasView.tsx`, `widgets/CameraStream.tsx`.
- [ ] Root → `ViewShell intent="fit"`. Collapse Tabs + Overlay toggle + Rescan into ONE toolbar row (tabs left, controls `ml-auto`). Stream floor responsive `flex-1 min-h-[14rem] sm:min-h-[20rem] lg:min-h-[24rem]`; give the stream an `bg-ink-2` plate + `.edge-light`.
- [ ] Overlay-AprilTags label → `text-foreground` when on. Streams chip: truncate URL `max-w-[55vw]` on phone; Rescan ≥44px on touch.
- [ ] Add a grid-mode toggle (`grid-cols-1 lg:grid-cols-2`) to show RGB+Gripper+Depth together (fills desktop). (Zoom/pan + maximize come in Phase 3.)
- [ ] Verify. Commit `feat(hmi): recompose Cameras — single toolbar, opaque plate, responsive floor, grid mode`.

### Task 12: Telemetry
**Files:** `views/TelemetryView.tsx` + scalar widget tiles (`ImuCard`,`OdometryCard`,`SystemCard`,`BatteryCard`,`ObservationsCard`).
- [ ] Root → `ViewShell intent="fit"`. Inner readout tiles: `bg-background/40` → opaque `bg-ink-3` (fix inverted ramp). Right rail stretches to the 2-row Lidar baseline (`lg:grid-rows-2` + `h-full` on tall content). Add an `md:grid-cols-3` intermediate tier. Bump 3px gauge bars to 5–6px with `bg-ink-3` tracks.
- [ ] Observations table → internal scroll + sticky `thead` (card surface). Fill the near-empty Observations band (denser chip grid or a summary tile beside it).
- [ ] iPhone (**sensor grid scrolls internally** under a fixed header): Lidar `min-h-[180px]`, scalar cards 2-up. Shell header fixed.
- [ ] Verify. Commit `feat(hmi): recompose Telemetry — opaque tiles, right-rail fills Lidar height, capped tables`.

### Task 13: Logs (fix scroll contract)
**Files:** `views/LogsView.tsx`.
- [ ] Root → `ViewShell intent="flow"` OR keep `fit` with the stream as the `flex-1 min-h-0 overflow-y-auto` child. **Drop `max-h-[60vh]`.** Remove the dead `ScrollArea` wrapper — apply card chrome + `flex-1 min-h-0 overflow-y-auto` directly to the scrolling div (keep `scrollViewportRef` on it). Panel → `bg-ink-3` + `.edge-light`; row dividers `divide-hairline` full opacity.
- [ ] Key rows by stable id (not array index). Phone: compact toolbar (selects+search one wrapping row; auto-scroll/Pause/Clear icon cluster).
- [ ] Verify (stream fills column, footer pins, no 60vh void). Commit `feat(hmi): fix Logs scroll contract — stream fills, stable keys, opaque panel`.

### Task 14: Map / Nav (layout only; zoom/pan in Phase 3)
**Files:** `views/MapView.tsx`, `widgets/MapCanvas.tsx` (container only), `widgets/LightControl.tsx`, `widgets/MissionControls.tsx`.
- [ ] Root → `ViewShell intent="fit"` with `h-full min-h-0`. Drop the `28rem/24rem` floors → canvas region `flex-1` so the bottom row (Greenhouse State + Tulip) stays on-screen. Move map header controls into a dedicated toolbar row (`flex-col sm:flex-row`, controls `overflow-x-auto` on phone).
- [ ] Status Light: replace fixed `lg:w-[360px]` with `lg:grid-cols-[1fr_minmax(320px,360px)]` + `items-stretch`. Greenhouse State table → internal scroll + sticky thead. 8px canvas safe-area for corner overlays.
- [ ] Verify (all rows visible at laptop; controls one tidy row on iPhone). Commit `feat(hmi): recompose Map/Nav — fit shell, all rows on-screen, proportional Status Light`.

### Task 15: Touch-target + native-input sweep + tab-strip/TopBar density
**Files:** `App.tsx` (tab strip), `components/TopBar.tsx`, plus any remaining sub-spec controls found.
- [ ] Tab strip: keep the ACTIVE tab's label inline on phone (`data-[state=active]:inline` on the label span), right-edge fade mask, ≥44px triggers. Responsive header height `h-14` phone / `h-16` sm+. TopBar right cluster → priority collapse (Connection+Battery+E-stop always; Clock/Docs/Settings into a kebab below md); MissionStrip `min-w-0 truncate`.
- [ ] Grep for sub-44px touch targets across views; bump on `useIsTouch`.
- [ ] Verify at iPhone (no horizontal scroll in TopBar; one worded tab anchor). Build green. **CHECK IN** before Phase 3.
- [ ] Commit `feat(hmi): touch-target floor + tab-strip/TopBar phone density`.

---

## PHASE 3 — Vision expand + zoom/pan (build once, apply to all)

### Task 16: `zoompan.ts` + `ExpandButton` + `FocusPanel`
**Files:** Create `lib/zoompan.ts`, `components/ui/ExpandButton.tsx`, `components/system/FocusPanel.tsx`.
- [ ] `zoompan.ts`: a ref-based controller `createZoomPan({min,max})` exposing `{scaleRef, offsetRef, onWheel(e,rect), onPointerDown/Move/Up (drag-pan), pinch handlers, reset()}` — clamped, cursor-anchored zoom. No per-frame React state (callers read refs in their draw/transform).
- [ ] `ExpandButton`: a `.tag`-styled icon button (`Maximize2`, h-7 hit area ≥44px on touch) with `aria-label="Maximize"`.
- [ ] `FocusPanel`: Radix `Dialog` (already a dep) with near-opaque `ink-0` backdrop + `.glass-strong` control rail + reticle/edge-light; content area 90vw×90vh hosting a render-prop child + a shared zoom/pan toolbar (zoom −/+/reset, ≥44px). Provide `useFocusPanel()` to open with `{title, render: (ctx) => ReactNode}`. Stays in-shell (E-stop visible).
- [ ] Verify: open an empty FocusPanel from a temp button; Esc closes; backdrop opaque; build green.
- [ ] Commit `feat(hmi): shared FocusPanel maximize dialog + zoom/pan controller + ExpandButton`.

### Task 17: Map zoom/pan + maximize
**Files:** `widgets/MapCanvas.tsx`, `views/MapView.tsx`.
- [ ] Add `viewScaleRef` (1) + `viewOffsetRef` ({0,0}); fold into the single `projection()`: `s = sFit*viewScale; cx=w/2+offset.x; cy=h/2+offset.y`. Both hit-tests + every draw already route through `worldToCanvas`, so goal-setting keeps working. Bind wheel (cursor-anchored) + two-finger pinch INLINE; keep single-pointer drag = goal inline.
- [ ] Add `ExpandButton` after Erase (`MapCanvas.tsx:~752`) → opens FocusPanel rendering the SAME canvas at 90vw×90vh with full drag-pan enabled (pan only in the dialog). Add a "fit/reset" control.
- [ ] Verify: inline pinch-zoom works, goal-drag intact; maximized pan+zoom+reset works; E-stop visible. Commit `feat(hmi): map zoom/pan + in-app maximize`.

### Task 18: 3D twin OrbitControls (maximized) + maximize
**Files:** `system/RobotTwin.tsx`, `views/ArmView.tsx`.
- [ ] Import `OrbitControls` from drei (already a dep); render in `<Scene>` gated by a `controllable` prop: inline twin keeps cinematic auto-rotate (no controls / pinch-zoom only), maximized variant gets `enablePan enableZoom enableDamping`, `target=[0,0.3,0]`, clamped `minDistance/maxDistance`, auto-rotate off. Respect reduced-motion.
- [ ] Add `ExpandButton` beside the live/stale badge → FocusPanel renders the twin `controllable`.
- [ ] Verify: inline unchanged; maximized orbit/zoom/pan smooth; `frameloop` still pauses on tab-hidden. Commit `feat(hmi): twin OrbitControls in maximize + expand affordance`.

### Task 19: Camera zoom/pan + replace native fullscreen with FocusPanel
**Files:** `widgets/CameraStream.tsx`, `views/CamerasView.tsx`.
- [ ] Wrap img+overlay-canvas in a transformed container; apply the SAME `transform: scale(z) translate(px,py)` to both (overlays stay registered) via `zoompan.ts`; clamp 1×–6×; wheel + pinch + drag.
- [ ] **Remove** `requestFullscreen`/`exitFullscreen` (CameraStream ~334–343,455) and the `fullscreen` state; replace the Maximize2 button with `ExpandButton` → FocusPanel rendering the stream at 90vw×90vh with zoom/pan. Derive overlay-canvas size from `img.naturalWidth/Height` (drop hard-coded 640×480). Delete the large commented-out legacy block (CameraStream top + CamerasView).
- [ ] Verify: zoom/pan registered with AprilTag boxes; maximize keeps E-stop visible. Commit `feat(hmi): camera zoom/pan + in-app maximize (drop native fullscreen)`.

### Task 20: Apply maximize to Lidar + Logs; final pass
**Files:** `views/TelemetryView.tsx` (Lidar card), `widgets/LidarCanvas.tsx`, `views/LogsView.tsx`.
- [ ] Lidar: `ExpandButton` in the card header → FocusPanel renders the scan large with wheel/pinch scaling `pxPerM` (clamp 1–12m) + drag-pan.
- [ ] Logs: `ExpandButton` in toolbar → FocusPanel renders the stream full-viewport (+ optional copy/export). (Optional/低 priority — include if time.)
- [ ] Full regression: measure all 7 tabs at laptop + iPhone (fit table holds); `prefers-reduced-motion`/`-contrast`/`-transparency` still honored; `npm run build` green.
- [ ] Commit `feat(hmi): maximize for Lidar + Logs; final regression pass`.

---

## Wrap-up
- [ ] `npm run build` green; quick smoke of every tab at laptop + iPhone + tablet-landscape in the mock HMI.
- [ ] Update `lupin/CHANGELOG.md` (if present) + a short note in the HMI docs.
- [ ] Open a GitLab MR `feat/hmi-design-system-v2` → `main` (per team MR-review policy); summary with before/after screenshots. Fast-forward to sim/hardware later if desired (generic frontend → main).

## Self-review notes
- Spec coverage: goals 1–5 map to Phases 1–3 (responsivity→T4/T15; composition→T8–T14; contrast→T1–T3; single-screen→T5/T8–T14; vision-zoom→T16–T20). ✓
- Type consistency: `ViewShell({intent})`, `useFocusPanel()`, `createZoomPan()`, `ExpandButton` used identically across tasks. ✓
- Guardrails enforced per task (opaque base tiles, glass overlay-only, token-only colors, refs-not-state for zoom/pan, in-shell maximize). ✓
