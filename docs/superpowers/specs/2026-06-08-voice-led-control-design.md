# Voice tool: `set_light` — status-light control from the voice agent

- **Date:** 2026-06-08
- **Branch:** `feat/voice-led-control` (off `main`)
- **Status:** Approved design — ready for implementation plan

## 1. Motivation

The Lupin HMI voice agent (`lupin_web`) exposes a tool surface the Gemini Live
model can call to drive, navigate, move the arm, and read telemetry. The robot
has a NeoPixel status-light strip whose colour is normally driven by the mission
state machine (`lupin_hmi/light_strip_bridge`), with a manual override already
reachable from the `LightControl` widget on the Map view.

There is currently no way to control the status light by voice. This spec adds
**one tool, `set_light`**, exposing the *existing* LED backend — pin a colour or
hand control back to the FSM — with **no new ROS code**.

## 2. Existing backend (reused as-is)

Two services, already deployed and already typed in `web/src/types/ros.ts`:

| Intent | Service (`LED_SERVICE`) | Type | Request | Response |
|---|---|---|---|---|
| Pin colour | `/lupin/leds/set` | `mirte_msgs/srv/SetNeopixel` | `{ color: {r,g,b} }` (0–255) | `{ status: bool }` |
| Hand back to FSM | `/lupin/leds/auto` | `std_srvs/srv/Trigger` | `{}` | `{ success, message }` |

The mock RosCore (`web/src/lib/ros.tsx`, `mockServiceResponse`) already returns
`{status:true}` for `SetNeopixel` and `{success:true, message:''}` for `Trigger`,
so the tool works in the offline `?mock=1` harness with **no mock changes**.

Safety: `light_strip_bridge` force-overrides the strip to red while
e-stopped/faulted, regardless of a manual hold. *(Verify this in
`light_strip_bridge.py` before asserting it in the tool description / docs.)*

## 3. Design decisions (locked with the user)

- **Named palette, not RGB.** The tool takes a colour *name* from the fixed
  12-colour status palette, so a voice-set colour is byte-identical to the
  mission-FSM / widget colour it represents. (Chosen over raw RGB and over
  name-or-RGB hybrid.)
- **One tool, not two.** `set_light` with a `mode` discriminator, mirroring the
  existing `gripper({action})` pattern, rather than a separate `light_auto`.
- **Not e-stop gated.** Cosmetic; the widget isn't gated; the bridge enforces
  safety red anyway.

## 4. Components

### 4.1 `web/src/lib/leds.ts` — NEW, single source of truth

The palette currently lives inline in `LightControl.tsx`. Extract it so the
widget, the voice tool, and the docs share one definition (mirrors `lib/arm.ts`
for gripper limits).

Exports:
- `interface Rgb { r: number; g: number; b: number }`
- `interface LedPreset { name: string; label: string; rgb: Rgb }`
- `LED_PRESETS: LedPreset[]` — the 12 entries: canonical lowercase `name`,
  Titlecase `label`, and `rgb`.
- `LED_PRESET_NAMES: string[]` = `LED_PRESETS.map(p => p.name)`.
- `ledPresetByName(name: string): LedPreset | undefined` — trims + lowercases,
  exact match against `name`.
- helpers `rgbToCss`, `rgbToHex`, `hexToRgb` (moved verbatim from
  `LightControl.tsx`).

Palette (names lowercase; values **identical** to the current `LightControl`
`PRESETS` so nothing changes on screen):

| name | label | r | g | b |
|---|---|---|---|---|
| red | Red | 255 | 0 | 0 |
| orange | Orange | 255 | 128 | 0 |
| amber | Amber | 255 | 191 | 0 |
| yellow | Yellow | 255 | 255 | 0 |
| green | Green | 0 | 255 | 0 |
| spring | Spring | 0 | 255 | 128 |
| teal | Teal | 0 | 180 | 140 |
| cyan | Cyan | 0 | 255 | 255 |
| blue | Blue | 0 | 0 | 255 |
| purple | Purple | 180 | 0 | 255 |
| white | White | 255 | 255 | 255 |
| off | Off | 0 | 0 | 0 |

### 4.2 `web/src/components/widgets/LightControl.tsx` — REFACTOR

Replace the inline `PRESETS`, the local `Rgb` interface, and the three colour
helpers with imports from `lib/leds.ts`. The widget renders `LED_PRESETS` using
`.label` for the title/aria text and `.rgb` for the swatch — **no behaviour
change**. Net effect: deletions + imports; the card looks and behaves identically.

### 4.3 `web/src/lib/voice/tools.ts` — ADD declaration

Append a `set_light` `FunctionDeclaration` and add `'set_light'` to the
`ToolName` union. Declaration:

```ts
{
  name: 'set_light',
  description:
    "Control the robot's status light strip (cosmetic — never moves the robot). " +
    "To pin a colour, pass `color` with one of the named palette colours listed in " +
    "the system prompt; this takes the strip OFF mission-state control and holds that " +
    "colour. To hand the strip back to the mission state machine (so it reflects robot " +
    "state again), pass `mode:'auto'`. Note `color:'off'` makes the strip dark but stays " +
    "manual — that is different from `mode:'auto'`, which resumes automatic state colours. " +
    "The robot still forces the strip red on its own while e-stopped or faulted, regardless " +
    "of a colour set here.",
  parameters: {
    type: 'object',
    properties: {
      color: {
        type: 'string',
        description:
          "Palette colour name to pin, e.g. 'blue', 'green', 'off'. Must be one of the " +
          "names listed in the system prompt. Ignored when mode='auto'.",
      },
      mode: {
        type: 'string',
        enum: ['auto'],
        description: "Pass 'auto' to return the strip to mission-state control. Omit when setting a colour.",
      },
    },
  },
}
```

No `required` array: either `color` or `mode` is valid. The dispatcher validates
and, if neither is usable, returns a helpful error listing the valid names.

### 4.4 `web/src/lib/voice/session.tsx` — ADD dispatch case

Add `case 'set_light':` to `dispatchTool`. Logic:

1. If `args.mode === 'auto'` → call `setAuto` (`LED_SERVICE.setAuto`,
   `LUPIN_SRV.Trigger`, `{}`) → `finish({ ok: !!res.success, action: 'light_auto',
   message: res.message || 'following mission state' })`.
2. Else resolve `ledPresetByName(String(args.color ?? ''))`:
   - not found → `finish({ ok: false, error: 'unknown colour "<x>". Known:
     <LED_PRESET_NAMES joined>, or pass mode:\'auto\'.' })`.
   - found → call `setManual` (`LED_SERVICE.setManual`, `MIRTE_SRV.SetNeopixel`,
     `{ color: preset.rgb }`) → `finish({ ok: !!res.status, action: 'set_light',
     color: preset.name, rgb: preset.rgb, ...(res.status ? {} : { error: 'LED
     service rejected the colour' }) })`.
3. Service throw → `finish({ ok: false, error: 'LED service unavailable: <msg>' })`.

Notes:
- `mode:'auto'` takes precedence if the model passes both `mode` and `color`.
- **NOT** gated by `estop.active` (add a one-line comment explaining why).
- Imports to add: `LUPIN_SRV`, `SetNeopixelRequest`, `SetNeopixelResponse` from
  `@/types/ros` (`MIRTE_SRV` is already imported); `LED_SERVICE` from
  `@/types/ros`; `ledPresetByName`, `LED_PRESET_NAMES` from `@/lib/leds`.

### 4.5 System prompt — `web/src/lib/settings.ts` + `buildSystemInstruction`

- Default `voiceSystemPrompt` (`settings.ts`): extend the capability sentence to
  "…open and close the gripper, set the status light, and report telemetry." and
  add one sentence: "For the status light, call set_light with a colour name to
  pin the strip, or set_light with mode 'auto' to hand it back to the mission
  state machine; the light is cosmetic and safe to change anytime."
- `buildSystemInstruction()` (`session.tsx`): append an authoritative palette
  line built from `LED_PRESET_NAMES`, alongside the existing location/pose/
  sequence lines — e.g. `Status-light colours: <names>.` This guarantees the
  names are always present even if the operator customised their prompt, and
  sources them once from `lib/leds.ts`.

### 4.6 `web/src/lib/voice/mock.ts` — OPTIONAL demo beat

Add one `set_light` call to the scripted offline demo so the tab shows the tool
live. Low priority; skip if it complicates the script.

## 5. Verification

- `cd web && npm run build` (`tsc -b && vite build`) green — this is the
  typecheck + build gate. (Worktree needs `node_modules`: symlink the existing
  `lupin_web/web/node_modules` into the worktree, or `npm ci`.)
- Mock-mode browser check (`?mock=1`) via the typed-prompt fallback:
  - "make the lights blue" → `/lupin/leds/set`, status ok, transcript confirms.
  - "put the lights back to auto" → `/lupin/leds/auto`.
  - Regression: `LightControl` widget still renders all 12 swatches + custom
    picker after the palette extraction.
- No web unit-test runner is configured; not adding one for this change.

## 6. Documentation

- `lupin_web/README.md`: add a `set_light` row to the voice-tool table.
- `lupin-site/content/docs/web-hmi.mdx`: add `set_light` to the documented voice
  tool list/section.
- Redeploy the live site (Vercel project `lupin-robot`,
  https://lupin-robot.vercel.app). **Confirm with the user before the production
  deploy.**

## 7. Branch / commit strategy

- Implement on `feat/voice-led-control` (this worktree, off `main`).
- Open a GitLab MR into `main`.
- After merge, propagate to `hardware` and `sim` for three-branch parity (the
  tool is generic — sim-mock and hardware both work).
- **No** Co-Authored-By / Claude attribution trailer on commits.

## 8. Non-goals (YAGNI)

- No brightness / per-pixel / animation params — whole-strip RGB only; "dim"/
  "warm" is handled by the model picking a different palette entry (or simply
  unsupported beyond the 12).
- No RGB or hex input path — names only (decided).
- No new ROS nodes, messages, or services.
- No separate `light_auto` tool — folded into `set_light`'s `mode`.

## 9. Acceptance criteria

1. `set_light({color:'<name>'})` for each of the 12 names publishes the correct
   RGB to `/lupin/leds/set`; an unknown name returns `ok:false` with the valid list.
2. `set_light({mode:'auto'})` calls `/lupin/leds/auto`.
3. `set_light` works while e-stopped (not gated).
4. `LightControl` widget behaviour unchanged after the palette extraction.
5. `npm run build` green; mock-mode check passes.
6. README + `web-hmi.mdx` updated; live site redeployed.
