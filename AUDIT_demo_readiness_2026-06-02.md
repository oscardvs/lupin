# Lupin demo-readiness audit — 2026-06-02

Read-only multi-agent audit (5 dimensions, 20 findings, adversarially verified).
Anchored to `lupin_bringup/DEMO_DAY.md`. This is the **verify pass before the fix pass**.
Branch `main` @ `b1bd4e4`. Untracked doc — delete or commit as you like.

> Audit-completeness caveat: the workflow's final synthesis + part of the verify
> stage hit a usage limit (resets 1pm). 6 of the ~14 actionable findings got an
> independent adversarial verdict (marked ✅ CONFIRMED); the rest are single-source
> (high-confidence, but re-check the cited lines before acting). Findings that need
> a live on-robot check are tagged ⚙️.

---

## TL;DR

- **Voice "forward 1 m" did nothing because the voice tab was in the MOCK harness, not Gemini Live.** No API key in that browser profile ⇒ `isLive=false` ⇒ `start()` runs the scripted mock, which never calls a motion tool. ✅ **Fix: paste a Gemini key into HMI Settings + internet on the tether.** (`settings.ts:130`, `session.tsx:116-119,613-625`)
- **One runbook gap explains most of the "perception/mission doesn't show" worry.** The DEMO_DAY 8-terminal layout (T1–T8) never launches the mission orchestrator, `perception_aggregator`, `yolo_detector`, `greenhouse_bridge`, or `seed_amcl_pose`. The code exists and is well-built — it's just **not started** by the documented bring-up. ✅ (`DEMO_DAY.md:328-339`, `perception.launch.py:81`)
- **The glass redesign DID stick** — aurora/boot/3D-twin/tickers are all in the 09:48 `dist/` that `preview` serves (verified via shader + three.js literals in the bundle). My earlier "stale build = glass didn't stick" guess was **wrong**. The *only* stale piece is the arm-joint tab (one commit, `96627d4`, after the build). One `npm run build` closes it.
- **"Bug/pest detection" is not a real detector** — `bug` is a 4th YOLO class surfaced as a boolean `anomaly` flag riding the flower path; the dedicated `KIND_ANOMALY`/`AnomalyReport` contract is an unpopulated stub. ✅
- **There is no QR decoding** — only numeric AprilTag IDs, shown as a camera overlay (ID + distance), not a readable info panel and not as a map marker.

---

## Your 4 questions, answered

### a. Why did voice "drive forward one meter" fail?

**Ranked root cause:**

1. **(✅ CONFIRMED, high) Mock harness, not Gemini Live.** `geminiApiKey` defaults to `''` (`settings.ts:130`), is only ever set via the Settings drawer and persisted per-browser in `localStorage` (no env injection). With an empty key, `isLive` is false (`session.tsx:116-119`) and `start()` runs `runMockSession()` (`session.tsx:613-625`), which only fires a scripted `query_state` and **never** calls `nav_forward`/`drive`. The robot literally cannot receive a motion tool call. → **Set the key in Settings, confirm internet on the separate interface (DEMO_DAY §0), and watch the voice status go to a live state, not mock.**
2. **(⚙️ UNCERTAIN→low) If a key is present, the agent might pick `drive`.** `drive` is capped at `0.3 m/s × 2 s = 0.6 m` (`session.tsx:39,184-205`) — it *cannot* reach 1 m and isn't a Nav2 goal. The prompt only weakly steers "go N metres" → `nav_forward`. Tighten the system prompt / tool descriptions so distance requests always route to `nav_forward`.
3. **(✅ CONFIRMED, medium) `nav_forward` is a SILENT no-op when `mapPoseRef` is null** (`session.tsx:230-236`; `rotate` same at `275-280`) — returns `{ok:false}` with no operator-visible error. Rare with SLAM+Nav2 up, but invisible when it bites.
4. **(✅ CONFIRMED, medium) A key/connection failure only shows as a small status tag** (`gemini-live.ts:90-98` → `session.tsx:672-679` → `VoiceView.tsx`), easily read as "nothing happened."

The goal pipeline itself is fine: voice `publishGoal` and the working map-click publish an **identical** `PoseStamped` to `/goal_pose` — Nav2 consumption is proven good, so it is **not** the differentiator.

### b. Mission control — does it work per DEMO_DAY?

**The orchestrator is genuinely complete and tested** (✅ M4): HSM `BOOT→READY→PREPARE→{EXPLORING|INSPECTING}→MONITORING→RETURNING→DONE`, with pause/e-stop/battery as flags and a FAULT sink. EXPLORING/MONITORING are fully wired across `MissionState.msg` → `types/ros.ts` → `MissionStrip`/`MissionControls`/`formatMissionPhase`; the old "dark states" bug is closed and covered by `test_exploration_discovers_then_monitors`. One-click Take-Control works (M7).

**But (high):**
- **M1 — DEMO_DAY never launches the mission pipeline.** T1–T8 bring up *zero* mission capability and §5 has no mission/explore/monitor acceptance row. `hardware.launch.py` defaults `mission:=false`.
- **M2 (medium) — Low-battery resume bug.** Resuming an Exploration/Monitoring mission after a dock routes `RETURNING→INSPECTING`, dropping into the wrong sub-machine.
- **M3 (⚙️ medium) — `BatteryMonitor` compares `percentage < 0.20` against unverified `power_watcher` units.**

### c. Perception → HMI — per path

| Path | Backend publishes? | HMI renders? | End-to-end | Verdict |
|---|---|---|---|---|
| **Flower → map marker** | ✅ real YOLO (`tulip_red/white/pink`) → `KIND_FLOWER` on `/floranova/observations` → twin → `/twin/state` | ✅ `MapCanvas` species-coloured ring | **Code complete, but NOT launched in the demo runbook** | partial |
| **Bug/pest → map marker** | ⚠️ no real detector — `bug` YOLO class → boolean `anomaly` flag on the *flower* path (`perception_aggregator.py:133,359-376`); `KIND_ANOMALY` is a stub | ✅ red dashed ring + "pest detected" tooltip on the flower marker | **Piggybacked + not launched** | partial/missing |
| **QR / AprilTag info → visible** | ✅ AprilTag IDs + corners + camera-frame distance + `tag_<id>` TF (hardware-live). **No QR, no payload decode** | camera-overlay only (ID+distance on MJPEG, behind "Overlay AprilTags"); **no decoded-info panel, not a map marker** | **Works (overlay only)** | partial |

Two extra caveats:
- **Flower/bug map pose** comes from the `tag_<id>` TF → map lookup, and the orchestrator pins at the **robot standoff pose**, not the plant — so a marker can land ~standoff-distance off the physical plant (P3/P4, medium).
- **YOLO is laptop-only and silently no-ops** if `ultralytics`/`numpy<2` aren't installed in that env (P6, ⚙️ medium).

### d. Glass-mode redesign — did it stick?

**Yes.** ✅ Aurora background (shader `fbm`/`gl_FragColor` literals ×43), boot sequence ("Establishing rosbridge link", "All systems nominal"), reactive tickers, and the URDF 3D twin (`RobotTwin` + meshes in `dist/urdf`) are all wired into `App.tsx` and **present in the 09:48 `dist/` that `preview` serves**. The missing word "aurora" in the bundle is a minification artifact (it lives in comments/identifiers), **not** missing code. All showpiece components are in the main working tree at HEAD (not stranded in the `feat/hmi-showpiece` worktree).

**The only gap (B1, medium):** the arm-joint slider fix (`96627d4`, 10:58) is the *single* HMI commit after the 09:48 build, so `preview` shows the full glass UI but the **old arm tab**. → `npm run build` then relaunch.

If the glass UI *still* looks off after rebuild, it's almost certainly **browser cache** — hard-reload (Ctrl/Cmd+Shift+R) so the page picks up the new hashed bundle.

---

## Prioritized findings (confirmed + high-confidence)

| Sev | Area | Finding | Fix | Conf | Evidence |
|---|---|---|---|---|---|
| **HIGH** ✅ | Voice | No Gemini key ⇒ voice runs MOCK; motion tools never called | Set key in Settings; verify live status + tether internet | 0.78 | `settings.ts:130`, `session.tsx:116-119,613-625` |
| **HIGH** ✅ | Perception/HMI | DEMO_DAY runbook never starts aggregator/yolo/mission ⇒ no flowers/bugs/discovered-tags/mission live | Add the missing launches (or use `hardware.launch.py mission:=true perception:=true yolo:=true`) | 0.9 | `DEMO_DAY.md:328-339`, `perception.launch.py:81` |
| **HIGH** | Mission | DEMO_DAY brings up zero mission capability; §5 has no mission row | Same launch fix + add a §5 acceptance row | 0.85 | `DEMO_DAY.md §4–5`, `hardware.launch.py mission default` |
| **MED** ✅ | Voice | `nav_forward`/`rotate` silent no-op when map→base TF null | Surface an operator-visible error/toast | 0.5 | `session.tsx:230-236,275-280` |
| **MED** ✅ | Voice | Gemini key/connection failure only a tiny status tag | Make mock/offline state loud in VoiceView | 0.55 | `gemini-live.ts:90-98`, `session.tsx:672-679` |
| **MED** ✅ | Perception | No real bug/pest detector — boolean `anomaly` on flower path; `KIND_ANOMALY` stub | Decide: keep piggyback (document it) or build the anomaly contract | 0.93 | `perception_aggregator.py:133,135,359-376,391` |
| **MED** ✅ | Perception | Flower type real YOLO but reaches map only via temporal co-location w/ a tag (no own 3D pose) | Document limitation; or add depth-based flower localization | 0.92 | `yolo_detector_node.py:28,54,91-145`, `perception_aggregator.py` |
| **MED** | HMI | AprilTag info is camera-overlay-only; no decoded panel, not on map; no QR | Add a tag-info panel if needed for the demo story | 0.92 | `CameraStream.tsx` overlay; map pins from `/twin/state` |
| **MED** | HMI | Flower/bug marker can land at robot standoff pose, not the plant | Pin at plant pose (offset by standoff) | 0.78 | orchestrator TAG_READING pose |
| **MED** | HMI build | `preview` shows old arm tab (one commit after build) | `npm run build` + relaunch | 0.97 | `96627d4` vs dist 09:48 |
| **MED** | Mission | Low-battery resume of Explore/Monitor → wrong (Inspection) sub-machine | Route resume back to the originating sub-machine | 0.75 | `node.py` RETURNING→ logic |
| **MED** ⚙️ | Perception | YOLO laptop-only; silent no-op without `ultralytics`/`numpy<2` | Verify install in the demo env | 0.85 | `yolo_detector_node.py` import |
| **LOW** ⚙️ | Voice | `drive` (if picked) maxes at 0.6 m — can't reach 1 m | Steer prompt to `nav_forward` for distances | — | `session.tsx:39,184-205` |

(Info/"works" findings omitted: V2/V6/V7, P1/P5, M4/M6/M7, G1/G2/G3/C1/BB1.)

---

## Corrected / withdrawn

- **"Glass redesign didn't stick because of the stale build"** — *withdrawn.* The audit verified the glass UI **is** in the served `dist/`; the staleness is limited to the arm tab. (G1/G2/G3/C1, conf 0.93–0.98.)
- **"drive can't reach 1 m" as the voice root cause** — *downgraded to low.* The math is right, but the agent more likely picks `nav_forward`, and the real blocker is the mock harness (V1). (V3 → uncertain/low.)

---

## Recommended fix order for the full pass

1. **`npm run build`** the HMI (closes the arm-tab gap; cheap, unblocks accurate testing). Hard-reload the browser.
2. **Set the Gemini API key** in HMI Settings + confirm tether internet → re-test voice "forward 1 m" (should now call `nav_forward`).
3. **Fix the demo bring-up** so perception/mission actually run: either extend DEMO_DAY's terminals with `perception_aggregator` + `yolo_detector` + the mission orchestrator + `greenhouse_bridge` + `seed_amcl_pose`, or document using `hardware.launch.py mission:=true perception:=true yolo:=true`. Add a §5 acceptance row for explore/monitor/flower. **(Highest leverage — lights up flowers, bugs, mission, discovered-tags in one move.)**
4. Make voice failure **loud**: surface mock/offline state and `nav_forward` null-TF errors in VoiceView.
5. Decide the **bug/pest story**: either own that it's a boolean anomaly flag on the flower marker (and say so in the pitch) or build the `KIND_ANOMALY` path.
6. Fix the **low-battery resume** sub-machine routing (M2).
7. Verify **YOLO install** + **battery units** in the actual demo env (P6, M3).
8. Consider **plant-pose** marker placement and a **tag-info panel** if the demo narrative needs decoded info on-screen.
