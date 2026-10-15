# Validation

Run `python -m pytest -q` from the project root. Current simulation tests exercise nonflat lake-at-rest equilibrium, positivity at dry fronts, constant-concentration transport, every grain inventory, repeated complete depletion, dry settling, repose transfer, soil-bucket capacity, saved-state round trips, invalid settings, and seeded terrain history. The final simulation report independently checks water and each grain inventory to a relative tolerance of `1e-9`.

Publication tests check that a failed state save preserves the earlier complete archive and that a matched but unreadable cache regenerates. Recipe tests render different views with different geometry budgets through a controlled renderer, then verify that earlier scene paths, asset bytes, modification times, and receipt hashes remain unchanged. Actual geometry/atlas/GLB regeneration is also checked for deterministic recipe reuse.

For an end-to-end native check, run:

```bash
terrain doctor
terrain demo all --quality smoke --grid 33 --duration 2 --threads 2
```

The command verifies state and geometry before rendering. CYBR LIGHT validates native film dimensions and finite pixels. A final receipt is written only after successful output publication and records the simulation/source/settings/file hashes. Preview success is evidence of that exact configuration, not an assertion that every possible input has been tested.

The original transient project configuration and tests were not retained in the recovery archive. The recovered Python package source was preserved; the current project configuration, documentation, and simulation tests were rebuilt and rerun. Historical logs under the separate recovery archive are not substituted for current test results.
