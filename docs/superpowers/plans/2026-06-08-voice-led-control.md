# `set_light` Voice Tool Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a `set_light` voice tool so the Gemini Live agent can pin the status-light strip to a named palette colour or hand it back to the mission FSM, reusing the already-deployed LED services.

**Architecture:** Extract the 12-colour palette from `LightControl.tsx` into a shared `lib/leds.ts` (single source of truth, mirrors `lib/arm.ts`). Add one `set_light` declaration to the voice tool surface and one dispatch `case` that calls the existing `/lupin/leds/set` (`mirte_msgs/srv/SetNeopixel`) and `/lupin/leds/auto` (`std_srvs/srv/Trigger`) services. Inject the authoritative palette names into the system prompt. No new ROS code.

**Tech Stack:** TypeScript, React, Vite (`lupin_web/web`); rosbridge service calls via `ros.callService`; Fumadocs MDX for the live site (`/home/oskrt/lupin-site`).

**Workspace:** Isolated worktree `/home/oskrt/worktrees/voice-led-control` on branch `feat/voice-led-control` (off `main`). All `lupin_web` paths below are relative to that worktree.

**Testing approach (read first):** The web app has **no unit-test runner** — `package.json` exposes only `dev` and `build`. The approved spec explicitly chose not to add one. So the per-task gate is `npm run build` (`tsc -b && vite build`), which statically type-checks all the wiring (wrong field names, missing imports, bad types all fail the build), followed by a **mock-mode browser exercise** (Task 7) that actually drives the dispatch against the mock services. Do not invent a `test`/`vitest` command — it does not exist.

**Spec:** `docs/superpowers/specs/2026-06-08-voice-led-control-design.md`

---

## File Structure

| File | Repo / branch | Responsibility | Change |
|---|---|---|---|
| `lupin_web/web/src/lib/leds.ts` | lupin · feat branch | Palette source of truth: `LED_PRESETS`, `LED_PRESET_NAMES`, `ledPresetByName`, rgb/hex helpers | **Create** |
| `lupin_web/web/src/components/widgets/LightControl.tsx` | lupin · feat branch | Map-tab LED widget — consume the shared palette | Modify |
| `lupin_web/web/src/lib/voice/tools.ts` | lupin · feat branch | `set_light` declaration + `ToolName` union | Modify |
| `lupin_web/web/src/lib/voice/session.tsx` | lupin · feat branch | `set_light` dispatch case + palette line in `buildSystemInstruction` | Modify |
| `lupin_web/web/src/lib/settings.ts` | lupin · feat branch | Default `voiceSystemPrompt` capability text | Modify |
| `lupin_web/web/src/lib/voice/mock.ts` | lupin · feat branch | Optional scripted `set_light` demo beat | Modify (optional) |
| `lupin_web/README.md` | lupin · feat branch | Voice-tool table row | Modify |
| `content/docs/web-hmi.mdx` | lupin-site · main | Voice tool list + count + LED section | Modify |
| `content/docs/index.mdx` | lupin-site · main | Tool-count reference | Modify |

---

## Task 0: Worktree build environment + confirm the safety claim

**Files:** none committed. Establishes the build env and verifies a claim the tool description/docs will make.

- [ ] **Step 1: Give the worktree a working `node_modules`**

The worktree was branched off `main` and has no `node_modules`. Symlink the existing one (instant; it's gitignored so it won't show in `git status`):

```bash
ln -s /home/oskrt/ros2_ws/src/lupin/lupin_web/web/node_modules \
      /home/oskrt/worktrees/voice-led-control/lupin_web/web/node_modules
```

- [ ] **Step 2: Baseline build (must be green before any change)**

Run:
```bash
cd /home/oskrt/worktrees/voice-led-control/lupin_web/web && npm run build
```
Expected: exit 0, `vite build` prints a bundle summary. If the symlink approach errors on native deps, fall back to `npm ci` in that dir.

- [ ] **Step 3: Confirm e-stop/FAULT forces red regardless of a manual hold**

The tool description and docs assert this. Verify it in the bridge:
```bash
grep -nE "estop|e_stop|FAULT|fault|255, *0, *0|force|override|red" \
  /home/oskrt/worktrees/voice-led-control/lupin_hmi/lupin_hmi/light_strip_bridge.py
```
Expected: a code path that paints red on e-stop/FAULT state irrespective of the manual-hold flag. Record the line. If the bridge does **not** enforce this, drop the "still forces red" clause from the tool description (Task 3) and the docs (Task 9), and note it for the user — do not assert behaviour that isn't there.

---

## Task 1: Create the shared palette module `lib/leds.ts`

**Files:**
- Create: `lupin_web/web/src/lib/leds.ts`

- [ ] **Step 1: Write the module**

```ts
/**
 * Status-light palette — the single source of truth shared by the Map-tab
 * LightControl widget and the voice `set_light` tool, so colour names and RGB
 * values can never drift between the UI, the agent, and the docs. Values mirror
 * lupin_hmi/light_strip_bridge's mission-state palette (see web-hmi docs).
 *
 * `name` is the canonical lowercase key the voice tool matches on; `label` is
 * the Titlecase display string the widget renders. The strip is whole-strip
 * RGB, 0–255 per channel.
 */

export interface Rgb {
  r: number
  g: number
  b: number
}

export interface LedPreset {
  /** Canonical lowercase key used by the voice tool and the system prompt. */
  name: string
  /** Titlecase display label used by the LightControl widget. */
  label: string
  rgb: Rgb
}

export const LED_PRESETS: LedPreset[] = [
  { name: 'red', label: 'Red', rgb: { r: 255, g: 0, b: 0 } },
  { name: 'orange', label: 'Orange', rgb: { r: 255, g: 128, b: 0 } },
  { name: 'amber', label: 'Amber', rgb: { r: 255, g: 191, b: 0 } },
  { name: 'yellow', label: 'Yellow', rgb: { r: 255, g: 255, b: 0 } },
  { name: 'green', label: 'Green', rgb: { r: 0, g: 255, b: 0 } },
  { name: 'spring', label: 'Spring', rgb: { r: 0, g: 255, b: 128 } },
  { name: 'teal', label: 'Teal', rgb: { r: 0, g: 180, b: 140 } },
  { name: 'cyan', label: 'Cyan', rgb: { r: 0, g: 255, b: 255 } },
  { name: 'blue', label: 'Blue', rgb: { r: 0, g: 0, b: 255 } },
  { name: 'purple', label: 'Purple', rgb: { r: 180, g: 0, b: 255 } },
  { name: 'white', label: 'White', rgb: { r: 255, g: 255, b: 255 } },
  { name: 'off', label: 'Off', rgb: { r: 0, g: 0, b: 0 } },
]

/** Canonical names, for the voice tool's error message and the system prompt. */
export const LED_PRESET_NAMES: string[] = LED_PRESETS.map((p) => p.name)

/** Resolve a (case-insensitive, trimmed) colour name to its preset, or undefined. */
export function ledPresetByName(name: string): LedPreset | undefined {
  const key = name.trim().toLowerCase()
  return LED_PRESETS.find((p) => p.name === key)
}

export function rgbToCss({ r, g, b }: Rgb): string {
  return `rgb(${r}, ${g}, ${b})`
}

export function rgbToHex({ r, g, b }: Rgb): string {
  const h = (n: number) => n.toString(16).padStart(2, '0')
  return `#${h(r)}${h(g)}${h(b)}`
}

export function hexToRgb(hex: string): Rgb {
  const m = /^#?([0-9a-f]{2})([0-9a-f]{2})([0-9a-f]{2})$/i.exec(hex.trim())
  if (!m) return { r: 0, g: 0, b: 0 }
  return {
    r: parseInt(m[1], 16),
    g: parseInt(m[2], 16),
    b: parseInt(m[3], 16),
  }
}
```

- [ ] **Step 2: Build to verify it compiles**

Run: `cd /home/oskrt/worktrees/voice-led-control/lupin_web/web && npm run build`
Expected: exit 0. (tsc type-checks the new file even though nothing imports it yet.)

- [ ] **Step 3: Commit**

```bash
cd /home/oskrt/worktrees/voice-led-control
git add lupin_web/web/src/lib/leds.ts
git commit -m "feat(web): shared status-light palette (lib/leds.ts)"
```

---

## Task 2: Refactor `LightControl.tsx` onto the shared palette

**Files:**
- Modify: `lupin_web/web/src/components/widgets/LightControl.tsx`

Goal: zero behaviour change — delete the local palette/helpers and import them. The widget already uses `p.label` and `p.rgb`, which the new `LedPreset` still provides.

- [ ] **Step 1: Replace the local `Rgb`/helpers import with `lib/leds`**

Replace these lines near the top (the `useService`/`cn` imports stay):
```ts
import { useService } from '@/lib/ros'
import { cn } from '@/lib/utils'
import {
  LED_SERVICE,
  LUPIN_SRV,
  MIRTE_SRV,
  type SetNeopixelRequest,
  type SetNeopixelResponse,
} from '@/types/ros'
```
with:
```ts
import { LED_PRESETS, hexToRgb, rgbToCss, rgbToHex, type Rgb } from '@/lib/leds'
import { useService } from '@/lib/ros'
import { cn } from '@/lib/utils'
import {
  LED_SERVICE,
  LUPIN_SRV,
  MIRTE_SRV,
  type SetNeopixelRequest,
  type SetNeopixelResponse,
} from '@/types/ros'
```

- [ ] **Step 2: Delete the local `Rgb` interface**

Remove this block (now imported from `lib/leds`):
```ts
interface Rgb {
  r: number
  g: number
  b: number
}
```

- [ ] **Step 3: Point the swatch map at `LED_PRESETS`**

Change `{PRESETS.map((p) => {` to `{LED_PRESETS.map((p) => {` (one occurrence, inside the `grid grid-cols-6` div).

- [ ] **Step 4: Delete the local `PRESETS` const and the three helper functions at the bottom**

Remove the entire `PRESETS` array (the `// Preset palette …` comment through its closing `]`) **and** the `rgbToCss`, `rgbToHex`, `hexToRgb` function definitions at the end of the file — they now live in `lib/leds.ts`.

- [ ] **Step 5: Build to verify the refactor type-checks**

Run: `cd /home/oskrt/worktrees/voice-led-control/lupin_web/web && npm run build`
Expected: exit 0, no unused-symbol or missing-import errors. (A leftover reference to the deleted `PRESETS`/helpers would fail here.)

- [ ] **Step 6: Commit**

```bash
cd /home/oskrt/worktrees/voice-led-control
git add lupin_web/web/src/components/widgets/LightControl.tsx
git commit -m "refactor(web): LightControl consumes shared lib/leds palette"
```

---

## Task 3: Declare `set_light` in the voice tool surface

**Files:**
- Modify: `lupin_web/web/src/lib/voice/tools.ts`

- [ ] **Step 1: Add the declaration**

Insert this object into the `ROBOT_TOOL_DECLARATIONS` array immediately **before** the `speak` declaration (`{ name: 'speak', …`):

```ts
  {
    name: 'set_light',
    description:
      "Control the robot's status light strip (cosmetic — never moves the robot). To pin a " +
      "colour, pass `color` with one of the named palette colours listed in the system prompt; " +
      "this takes the strip OFF mission-state control and holds that colour. To hand the strip " +
      "back to the mission state machine so it reflects robot state again, pass `mode:'auto'`. " +
      "Note `color:'off'` makes the strip dark but stays manual — that is different from " +
      "`mode:'auto'`, which resumes the automatic state colours. The robot still forces the strip " +
      "red on its own while e-stopped or faulted, regardless of a colour set here.",
    parameters: {
      type: 'object',
      properties: {
        color: {
          type: 'string',
          description:
            "Palette colour name to pin, e.g. 'blue', 'green', 'off'. Must be one of the names " +
            "listed in the system prompt. Ignored when mode='auto'.",
        },
        mode: {
          type: 'string',
          enum: ['auto'],
          description: "Pass 'auto' to return the strip to mission-state control. Omit when setting a colour.",
        },
      },
    },
  },
```
(If Task 0 Step 3 found the bridge does **not** force red, delete the final sentence of the `description`.)

- [ ] **Step 2: Add `set_light` to the `ToolName` union**

In the `export type ToolName =` union, add a line. Place it before `| 'speak'`:
```ts
  | 'set_light'
```

- [ ] **Step 3: Build to verify**

Run: `cd /home/oskrt/worktrees/voice-led-control/lupin_web/web && npm run build`
Expected: exit 0. (The `switch (name as ToolName)` in `session.tsx` does not yet handle `set_light`, but `default:` covers it, so this still compiles. Task 4 adds the case.)

- [ ] **Step 4: Commit**

```bash
cd /home/oskrt/worktrees/voice-led-control
git add lupin_web/web/src/lib/voice/tools.ts
git commit -m "feat(web): declare set_light voice tool"
```

---

## Task 4: Dispatch `set_light` + inject palette into the system prompt

**Files:**
- Modify: `lupin_web/web/src/lib/voice/session.tsx`

- [ ] **Step 1: Add imports**

Add the `lib/leds` import alongside the other `@/lib/*` imports (e.g. after the `@/lib/ros` import):
```ts
import { LED_PRESET_NAMES, ledPresetByName } from '@/lib/leds'
```
Extend the existing `@/types/ros` import block to add `LED_SERVICE`, `LUPIN_SRV`, `SetNeopixelRequest`, `SetNeopixelResponse` (keep the existing members):
```ts
import {
  LED_SERVICE,
  LUPIN_SRV,
  MIRTE_SRV,
  ROS_TYPE,
  quatToEuler,
  type BatteryState,
  type Odometry,
  type PoseStamped,
  type SetNeopixelRequest,
  type SetNeopixelResponse,
  type SetServoAngleWithSpeedRequest,
} from '@/types/ros'
```

- [ ] **Step 2: Add the dispatch case**

Insert this `case` into the `switch (name as ToolName)` block, immediately **before** `case 'query_state': {`. It is deliberately **not** gated on `estop.active`:

```ts
          case 'set_light': {
            // Cosmetic — NOT e-stop gated. The widget isn't gated either, and the
            // light_strip_bridge force-overrides the strip red while e-stopped/
            // faulted regardless of a manual hold, so a colour set here can never
            // mask a safety state.
            const mode = String(args.mode ?? '').trim().toLowerCase()
            if (mode === 'auto') {
              try {
                const res = await ros.callService<
                  Record<string, never>,
                  { success: boolean; message: string }
                >(LED_SERVICE.setAuto, LUPIN_SRV.Trigger, {})
                return finish({
                  ok: !!res?.success,
                  action: 'light_auto',
                  message: res?.message || 'following mission state',
                  ...(res?.success ? {} : { error: res?.message || 'auto rejected' }),
                })
              } catch (e) {
                return finish({
                  ok: false,
                  error: `LED auto service unavailable: ${e instanceof Error ? e.message : String(e)}`,
                })
              }
            }
            const raw = String(args.color ?? '')
            const preset = ledPresetByName(raw)
            if (!preset) {
              return finish({
                ok: false,
                error: `unknown colour "${raw}". Known: ${LED_PRESET_NAMES.join(', ')}, or pass mode:'auto'.`,
              })
            }
            try {
              const res = await ros.callService<SetNeopixelRequest, SetNeopixelResponse>(
                LED_SERVICE.setManual,
                MIRTE_SRV.SetNeopixel,
                { color: preset.rgb },
              )
              return finish({
                ok: !!res?.status,
                action: 'set_light',
                color: preset.name,
                rgb: preset.rgb,
                ...(res?.status ? {} : { error: 'LED service rejected the colour' }),
              })
            } catch (e) {
              return finish({
                ok: false,
                error: `LED service unavailable: ${e instanceof Error ? e.message : String(e)}`,
              })
            }
          }

```

- [ ] **Step 3: Append the palette names to `buildSystemInstruction`**

In `buildSystemInstruction`, after the `seqLine` declaration and before the `return`, add:
```ts
    const ledLine = `Status-light colours: ${LED_PRESET_NAMES.join(', ')}.`
```
and change the return to include it:
```ts
    return `${settings.voiceSystemPrompt}\n\n${locLine}\n${poseLine}\n${seqLine}\n${ledLine}`
```

- [ ] **Step 4: Build to verify**

Run: `cd /home/oskrt/worktrees/voice-led-control/lupin_web/web && npm run build`
Expected: exit 0. Wrong field names (`res.success` vs `res.status`) or a missing import fail here.

- [ ] **Step 5: Commit**

```bash
cd /home/oskrt/worktrees/voice-led-control
git add lupin_web/web/src/lib/voice/session.tsx
git commit -m "feat(web): dispatch set_light + palette in system prompt"
```

---

## Task 5: Mention the status light in the default system prompt

**Files:**
- Modify: `lupin_web/web/src/lib/settings.ts`

- [ ] **Step 1: Extend the capability sentence**

In the default `voiceSystemPrompt` array, change:
```ts
    'and close the gripper, and report telemetry.',
```
to:
```ts
    'and close the gripper, set the status light, and report telemetry.',
```

- [ ] **Step 2: Add a status-light instruction line**

Immediately after that line, insert a new array element:
```ts
    'For the status light, call set_light with a colour name to pin the strip, or set_light',
    "with mode 'auto' to hand it back to the mission state machine; the light is cosmetic and",
    'safe to change anytime, including while e-stopped.',
```

- [ ] **Step 3: Build to verify**

Run: `cd /home/oskrt/worktrees/voice-led-control/lupin_web/web && npm run build`
Expected: exit 0.

- [ ] **Step 4: Commit**

```bash
cd /home/oskrt/worktrees/voice-led-control
git add lupin_web/web/src/lib/settings.ts
git commit -m "feat(web): default voice prompt mentions set_light"
```

---

## Task 6 (OPTIONAL): Scripted `set_light` beat in the mock session

**Files:**
- Modify: `lupin_web/web/src/lib/voice/mock.ts`

Skip if time-constrained — the mock works without it and Task 7 verifies via typed prompts. If doing it:

- [ ] **Step 1: Insert a light exchange before the final `ready` step**

In the `steps` array, immediately **before** the final `{ delayMs: 1500, run: () => opts.onStatus('ready') }`, insert:
```ts
    { delayMs: 1400, run: () => opts.onStatus('listening') },
    { delayMs: 800, run: () => opts.onTranscript(turn('user', 'Turn the status light blue.')) },
    { delayMs: 600, run: () => opts.onStatus('thinking') },
    {
      delayMs: 400,
      run: async () => {
        await opts.onToolCall('set_light', `mock-${Date.now()}`, { color: 'blue' })
      },
    },
    { delayMs: 600, run: () => opts.onStatus('speaking') },
    { delayMs: 200, run: () => opts.onTranscript(turn('model', 'Status light is now blue.')) },
```

- [ ] **Step 2: Build, then commit**

Run: `cd /home/oskrt/worktrees/voice-led-control/lupin_web/web && npm run build` → exit 0.
```bash
cd /home/oskrt/worktrees/voice-led-control
git add lupin_web/web/src/lib/voice/mock.ts
git commit -m "chore(web): mock voice session demos set_light"
```

---

## Task 7: Mock-mode browser verification (functional gate)

**Files:** none. This is the behavioural test that substitutes for a unit-test runner.

- [ ] **Step 1: Start the dev server**

```bash
cd /home/oskrt/worktrees/voice-led-control/lupin_web/web && npm run dev
```
Note the printed local URL (Vite default `http://localhost:5173`).

- [ ] **Step 2: Open the Voice tab in mock mode**

Drive a browser (Chrome DevTools MCP or Playwright) to `http://localhost:5173/?mock=1`, then click the **Voice** tab (`03`).

- [ ] **Step 3: Exercise a colour set via the typed-prompt fallback**

Type into the text input: `set the status light to blue`. Confirm in the tool/transcript panel a `set_light` invocation with args `{ color: 'blue' }` and result `{ ok: true, action: 'set_light', color: 'blue', rgb: {r:0,g:0,b:255} }`. (Mock `SetNeopixel` returns `{status:true}`.)

- [ ] **Step 4: Exercise an unknown colour and `auto`**

Type `set the light to chartreuse` → expect `ok:false` with the "unknown colour … Known: red, orange, …" message. Type `put the lights back to auto` → expect a `set_light` call with `{ mode: 'auto' }` and `ok:true, action:'light_auto'`.

- [ ] **Step 5: Regression-check the LightControl widget**

Go to the **Map / Nav** tab (`07`). Confirm the **Status Light** card still renders all 12 swatches, the custom colour picker + Set button work, and the Auto button is present. Take a screenshot for the record.

- [ ] **Step 6: Stop the dev server**

Stop the `npm run dev` process. No commit (verification only). If any step fails, fix the relevant task before proceeding.

---

## Task 8: In-repo documentation — `lupin_web/README.md`

**Files:**
- Modify: `lupin_web/README.md`

- [ ] **Step 1: Add the `set_light` row to the voice-tool table**

After the `| `engage_estop` … |` row, add:
```markdown
| `set_light`             | Pin a palette colour via `mirte_msgs/srv/SetNeopixel` on `/lupin/leds/set`, or `mode:'auto'` → `std_srvs/srv/Trigger` on `/lupin/leds/auto`. Cosmetic; not e-stop gated. |
```
(If the README states a tool count anywhere, bump it by one. Check with `grep -n "tool" lupin_web/README.md`.)

- [ ] **Step 2: Commit**

```bash
cd /home/oskrt/worktrees/voice-led-control
git add lupin_web/README.md
git commit -m "docs(web): document set_light voice tool"
```

---

## Task 9: Live-site documentation — `lupin-site` (deployed branch `main`)

**Files (separate repo `/home/oskrt/lupin-site`, branch `main`):**
- Modify: `content/docs/web-hmi.mdx`
- Modify: `content/docs/index.mdx`

> The site repo has unrelated uncommitted WIP (`networking.mdx`, `war-stories.mdx`). **Stage only the files this task touches** — never `git add -A`.

- [ ] **Step 1: Bump the tool count 22 → 23 (three spots)**

In `content/docs/web-hmi.mdx`:
- Line ~3 (frontmatter `description`): `a 22-tool Gemini Live voice agent` → `a 23-tool Gemini Live voice agent`.
- Line ~244: `a **22-tool function-calling surface**` → `a **23-tool function-calling surface**`, and `both carry 22 entries` → `both carry 23 entries`.

In `content/docs/index.mdx`:
- Line ~62: `its 22-tool voice agent` → `its 23-tool voice agent`.

- [ ] **Step 2: Add a `StatusLight` group to the voice mermaid diagram**

In `web-hmi.mdx`, inside the `<Mermaid chart={`classDiagram … `} />` block, after the `SafetyIO` class closes, add:
```
  class StatusLight {
    set_light()
  }
```

- [ ] **Step 3: Extend the "splits into …" sentence**

Change:
```
The surface splits into drive bursts, Nav2-mediated motion, arm presets and the pose/sequence library, the gripper, and safety/IO.
```
to:
```
The surface splits into drive bursts, Nav2-mediated motion, arm presets and the pose/sequence library, the gripper, the status light, and safety/IO.
```

- [ ] **Step 4: Add a `set_light` row to the dispatch highlights table**

After the `| `speak` | … |` row (last row), add:
```markdown
| `set_light` | Pin the status strip to a named palette colour via `mirte_msgs/srv/SetNeopixel` (`/lupin/leds/set`), or `mode:'auto'` → `std_srvs/srv/Trigger` (`/lupin/leds/auto`). Cosmetic, **not** e-stop gated. |
```

- [ ] **Step 5: Note the voice path in the LED control section**

In the `### LED control` section, append to the end of the paragraph (after the BRG sentence):
```
The same two services back the voice `set_light` tool, so "make the lights blue" or "put the lights back to auto" drive the identical manual/auto path as the card.
```

- [ ] **Step 6: Build the site to validate MDX compiles**

```bash
cd /home/oskrt/lupin-site && npm run build
```
Expected: a successful Next build (MDX compiles; no syntax errors). This is slower than the web build — that's fine.

- [ ] **Step 7: Commit (surgical add)**

```bash
cd /home/oskrt/lupin-site
git add content/docs/web-hmi.mdx content/docs/index.mdx
git commit -m "docs: document set_light voice tool; tool count 22 -> 23"
```

---

## Task 10: Build gate, MR, propagation, deploy (gated on user)

**Files:** none.

- [ ] **Step 1: Final web build gate**

Run: `cd /home/oskrt/worktrees/voice-led-control/lupin_web/web && npm run build`
Expected: exit 0.

- [ ] **Step 2: Push the feature branch**

```bash
cd /home/oskrt/worktrees/voice-led-control
git push -u origin feat/voice-led-control
```

- [ ] **Step 3: Open a GitLab MR** `feat/voice-led-control` → `main` (use the gitlab-tudelft MCP `create_merge_request`). Title: `feat(web): set_light voice tool`. Body: summary + the spec link + that it's verified green-build + mock-mode-exercised, not yet hardware-tested.

- [ ] **Step 4: Deploy the live site (CONFIRM WITH USER FIRST)**

The site deploys from `lupin-site` `main`. Confirm the deploy method (push to the Vercel-connected remote, or `vercel --prod`) and **get explicit go-ahead before the production deploy** — it is outward-facing. After deploy, verify https://lupin-robot.vercel.app/docs/web-hmi shows `23-tool` and the `set_light` row.

- [ ] **Step 5: Three-branch propagation (after MR merge)**

Once the MR merges to `main`, fast-forward/merge into `hardware` and `sim` for parity (the tool is generic). Coordinate with the user — there is unrelated uncommitted WIP on `main`'s working tree that must not be swept in.

---

## Self-Review

**Spec coverage:**
- §4.1 shared `lib/leds.ts` → Task 1. §4.2 LightControl refactor → Task 2. §4.3 declaration → Task 3. §4.4 dispatch → Task 4. §4.5 system prompt (both `settings.ts` default and `buildSystemInstruction`) → Task 5 + Task 4 Step 3. §4.6 optional mock → Task 6. §5 verification → Task 0 (build env) + per-task builds + Task 7 (mock browser). §6 docs → Task 8 (README) + Task 9 (site). §7 branch/MR/propagation → Task 10. Safety claim verification → Task 0 Step 3. All covered.

**Placeholder scan:** No TBD/TODO. Every code step shows complete code; every command shows expected output. The two conditional branches (bridge red-override absent; README tool-count) give concrete instructions, not vague hedges.

**Type consistency:** `SetNeopixel` response field is `status` (used in Task 4 manual branch + mock `{status:true}`); `Trigger` response is `success`/`message` (used in Task 4 auto branch). `ledPresetByName`/`LED_PRESET_NAMES`/`LED_PRESETS`/`Rgb` defined in Task 1 are used with matching names/signatures in Tasks 2 and 4. `LED_SERVICE.setManual`/`.setAuto`, `MIRTE_SRV.SetNeopixel`, `LUPIN_SRV.Trigger`, `SetNeopixelRequest`/`SetNeopixelResponse` all exist in `types/ros.ts` (verified during design). Tool name `set_light` consistent across tools.ts declaration, `ToolName` union, dispatch case, mock beat, and both docs.
