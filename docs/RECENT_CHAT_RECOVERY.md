# Recent-chat integration — 2026-09-14

Current master is preserved, including its shared renderer and CYBR YARD updates. This recovery does not replace active shared source with older archived copies.

## Active recovered projects

FUSE C220 printer: `examples/fuse_c220/`, with geometry, toolpaths, mechanism/edge validation, and the current shared V9 renderer adapter. REACH/REACH-2 forearm and elbow: `examples/reach*`, `tools/reach*`, the v5 CAD exporter, and associated tests. Both `reach2_orbit_integrated_v5.py` and its v5b variant are retained.

## Original source and integration fixes

588 archived source files under `archive/published_branches/2026-09-14/` and 35 active recovered forearm files were checked against their publication receipt before editing. Archived originals remain unchanged. Two active historical recipes are repaired: the 8 mm motor adapter uses a nondegenerate 3 mm fillet, and REACH-1 v2 constructs the same direct trapezoidal gussets as its retained v2_fixed recipe. The differential model now creates its validation output directory on a clean checkout. Exact before/after checksums and removed transport checksums are in `docs/validation/recent_source/master-integration-sources-2026-09-14.json`.

## Regression and source-only reproduction

Run `python tools/rebuild_differential_inputs.py` before the full test suite when differential arrays are absent. Existing complete arrays are retained; regenerated arrays carry their own receipt and are not represented as recovered historical bytes. The read-only `Recovery regression tests` workflow generates missing inputs once, checks identical input hashes in all eight test partitions, and verifies complete nonoverlapping coverage of every active and legacy Python test file. Initial failures, fixes, final results, and the merge head are recorded in PR #11. Skipped historical media/catalogue tests are not counted as passes.

## Publication still incomplete

CELL-01/AERIS, Hot Springs v4, clouds v4, BACKLOT-01, spectral quartz, and Prismatic II have not all been restored as ordinary GitHub source. The previous transfer stopped at 8 of 22 encoded parts. Its incomplete `.chat_recovery/source.part*` files and obsolete one-off recovery workflows are excluded from the active tree; history at `a7c4a7f1ab4da2c2ea532d849e8c9a55d1df2e33` retains the originals. The separately delivered `cybr-recent-chat-source-payload.zip` preserves the recovered source and printer preview/video. Large historical CAD and render deliveries are not claimed fully published.

These checks establish source preservation, CAD/software regression behavior, and deterministic numerical checks—not physical printer operation, complete torque/thermal safety certification, or a fresh full render.
