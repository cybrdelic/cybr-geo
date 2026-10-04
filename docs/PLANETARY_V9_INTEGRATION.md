# Planetary / V9 integration decision

The September 14, 2026 integration retains the renderer from master commit
`5a1790506983343ec07cebcb554e5a903c5d0c0a`, rather than replacing it with the older
renderer in planetary PR #4 (`498e56b8ebe71f8cab39ea91d2c080c7ca99fb20`).

All renderer source is preserved byte-for-byte from that master baseline,
including the V9/Mitsuba dispatch, native implementation, shared finishing,
fixed studio, capped sections, surface/pose temporal reprojection, exact SPP
allocation, immediate surface-buffer cleanup, and strict encode/decode/frame-count
checks. This is a preservation decision, not a new claim of measured image quality.

The planetary gear library, both actuator model modules, geometry/interface/
kinematics tests, documentation, film tool and existing workflows are retained.
The registry adds the planetary recipe without losing master's dependency-aware
cache fingerprinting or bound-motion support.

The old PR's private-helper film tests have been replaced with behavioral tests
against the selected film loop. They check exact 48/64/193-SPP allocation,
per-sample cleanup including `.surfaces`, retained reprojection inputs, and the
numbered frame sequence. Their tracer and encoder are explicit unit-test doubles;
these tests are not rendered-image evidence. Existing dedicated render workflows
remain separate from the combined current/legacy regression and assembly gate.

The superseded branch's subframe camera-orbit and explosion sampling changes are
NOT included: selecting the newer renderer does not justify silently claiming
those additional behaviors. They remain recoverable in the original PR history.
Mechanical design limitations remain those in PLANETARY_ACTUATOR.md; this merge
does not establish load ratings, manufacturing tolerances or physical qualification.
