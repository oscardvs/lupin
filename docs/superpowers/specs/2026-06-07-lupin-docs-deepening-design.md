# Lupin docs deepening, design (2026-06-07)

Target: the public Fumadocs site at `/home/oskrt/lupin-site` only. Goal: turn a
good overview into a deep, lived-in, top-tier open-source documentation set.
Non-AI prose, no em-dashes, native Mermaid schematics, loud image placeholders.

## Inputs
- Read-only gap analysis workflow (`lupin-docs-gap-analysis`, 16 agents) produced
  a per-page gap report, a 30-entry war-stories catalogue, and a house-style sheet.
  Split into `/tmp/lupin-docs-plan/gap-<page>.json`, `war-stories.json`,
  `house-style.json`.

## Accuracy fixes found (not just additions)
- `architecture`: "10 messages + 6 services" is wrong; real count is 11 + 11.
- frontier planner is not a BFS; it is frontier-cell detect, clearance filter,
  8-connected flood-fill clustering, size/distance scoring.
- sim drive topic section is self-contradictory (`/..._unstamped` has no sub in
  that launch; the working sim path is `/cmd_vel` into `planar_move`).
- e-stop + battery subsystem is invisible on the architecture page.
- voice agent is 22-tool now, docs say 16.
- stale `cmd_vel_mux`-era node/topic dumps must not be shown verbatim.

## Plan
- Deepen + correct all 14 existing pages (+ light `index` refresh).
- Add 3 new pages: `war-stories` (Project), `design-decisions` (after
  architecture), `simulation` (The stack). Nav updated in `meta.json`.
- ~+19k words, ~75 Mermaid diagrams, ~40 `<Figure>` image placeholders.

## Shared infra (done before writing)
- New `<Figure src caption />` MDX component: renders the real image when present,
  falls back to the loud fuchsia `IMAGE_PLACEHOLDER` block naming `public<src>`.
  Registered in `src/components/mdx.tsx`.
- `meta.json` nav updated with the three new pages.

## Execution
1. Writing workflow: one agent per page (17), each handed its gap report,
   the house-style sheet, its war stories, the diagram list, the existing doc,
   and the code. Different files, so no worktree contention.
2. Central `next build` to catch MDX/Mermaid errors, fix any that surface.
3. No-em-dash lint, internal-link check, per-page tone/accuracy review.

## Guardrails
- No push/deploy to Vercel without explicit go-ahead.
- No changes to the LaTeX report or in-repo README.
- No restructure beyond the three added pages.
