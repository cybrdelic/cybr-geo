# Pinned P5 task policy: final numeric evidence

These results apply to ARX belt P5 at source commit `aabdf8c57a4ee64f77b8a246ad9aca7b51d9cba2`, with GEO's explicit 17.4-second phase clock and `lateral_tilt_scale: 0.0`. They do not describe the latest ARX revision. The optional source-task coefficient defaults to 1.0 for the retained source oracle; the authored default project selects 0.0. TCP targets, forward tilt and roll remain unchanged.

The accepted runs used one freshly constructed, source-bound native build. The public command and independent artifact readback passed: 118 part definitions, 375 placements, 52 nominal interface checks, 385 aggregate components and 392 valid reopened STEP solids. Actual nominal replay media is in [the parent media packet](../MEDIA.md).

| Evidence | Outcome |
| --- | --- |
| Full source scenario inventory | 43/43 ordinary rows passed; all 6 adverse rows produced the expected rejection |
| Targeted half timestep | Nominal plus normal and weak-spring/double-delay stops at matched phases 3.2 and 7.2: 5/5 passed |
| Friction-control prefixes | Exact nominal state equality before both declared interventions: 240 and 282 recorded frames |
| Jaw travel | Zero observed residual beyond either authored slide range across all 49 full-suite and 5 refinement traces; every integration step included |

The minimum strict estimated transmitted-torque margin is **0.0767166924 Nm**, becoming **0.0766868513 Nm** at half timestep, at J2 in the late weak-spring case. Nominal J2 margin is 0.1136683515 Nm; commanded motor torque reaches its configured clamp and is reported separately. Matched 7.2 normal power loss gives J4 post-deadline sampled speed 0.0000316918 rad/s (half timestep 0.0000316952), against the unchanged 0.002 limit.

These are bounded numerical results. The weak-spring/double-delay 3.2 case has a contact-sensitive J5 peak change of -0.916136 Nm under refinement; both resolutions pass, but this is not a claim of whole-system convergence. Emergency payload retention is not an ordinary source acceptance condition: the matched 7.2 case still drops the payload about 0.201 m. No hardware qualification or complete continuous collision envelope is claimed. Original general numeric verdicts remain distinct from the separately applied adverse placement test.

Run the portable inspector from the repository root (Python standard library only):

```sh
python docs/pr-assets/arx-faults/final/inspect_final_faults.py
python docs/pr-assets/arx-faults/final/inspect_final_faults.py --extract-to outputs/restored-p5-final-receipts
```

The extraction destination must be new. The inspector verifies manifest hashes, exact gzip restoration of the six numeric receipts, compact-summary reconstruction, native/source bindings, scenario counts, prefix checks and execution metadata. It verifies recorded evidence; it does not rerun the simulations or reconstruct hardware.

`full-suite-summary.json` and `refined-suite-summary.json` preserve every original field except repeated reference objects, which are replaced by exact pointers to the lossless input files. The corresponding `.json.gz` files retain the complete original serialized bytes. `comparative-metrics.json` contains all case outcomes, strict margins, timestep comparisons and a hash-linked reference to the [failed earlier policy](../diagnostic-17.4s/fault-evidence-manifest.json). Full traces are kept in the recorded local run directories and are represented here by their verified hashes.

The execution receipts retain recorded arguments and resource settings with the path substitutions described below. The direct refinement used three workers with OPENBLAS=1, OMP=2 and LP=2. Its 996.8-second wall time is not an isolated performance benchmark. The prior Python-wrapped attempt failed with Windows WinError 6 before any scenario result; its execution receipt is explicitly excluded from numerical acceptance. No model or threshold changed for the direct retry.

## Public path substitutions and reproduction

The four `*-execution*.json` receipts are **derived public copies**, not the original serialized execution records. [publication-provenance.json](publication-provenance.json) records the original packet manifest SHA, original byte lengths and SHA-256 digests, exact JSON pointers changed, replacement strings and hashes of the replaced values. Only those string fields and the added `public_derivation` metadata differ. Timings, success/failure facts, commands' non-path arguments, source hashes and numerical evidence are unchanged. Serialization and the canonical unchanged-field projection are specified in that file; the inspector verifies the public projections. The original execution bytes remain in the local archived packet. Their recorded hashes are provenance anchors; public copies cannot establish their original byte equality. With that private archive available, `--original-packet PATH` additionally verifies its original manifest and bytes, and proves that precisely the declared substitutions were made.

`${REPO_ROOT}`, `${PYTHON_EXECUTABLE}`, `${PATHSEP}` and `${EXTRA_DEPENDENCIES}` describe the historical checkout, interpreter, platform path-list separator and extra dependency directory. These tokens are explanatory metadata, not commands to paste. The original machine-specific PowerShell wrapper and private curation helper are omitted, with their original hashes retained in provenance. The successful receipt's `wrapper_sha256` continues to identify the **original** PowerShell wrapper.

The two lossless `*-inputs.json.gz` archives still contain their original `/output` and `/native_directory` JSON values. Those four local absolute paths are historical run-location metadata retained for exact input-hash verification; the inspector never executes or requires them. No claim that the entire archived packet is path-redacted is made. Summaries, metrics, travel receipts and all six gzip files are unchanged.

For a new run, first install the dependencies following the [repository README](../../../../README.md), then construct a fresh native build from this checkout. The new [Python launcher](run_final_refinement.py) uses the active interpreter, discovers the checkout relative to itself, joins paths with `pathlib` and uses `os.pathsep`. It accepts `--repo`, `--native`, `--output` and `--workers`; the output directory must be new. It preserves any inherited `PYTHONPATH` after prepending this checkout's `src` directory. No historical private dependency directory is needed.

```sh
lab mechanism arx_belt --step --out outputs/arx_belt
python docs/pr-assets/arx-faults/final/run_final_refinement.py --dry-run
python docs/pr-assets/arx-faults/final/run_final_refinement.py
```

The launcher selects the five matched-phase half-timestep scenarios and recorded thread settings. It is a new reproduction aid, not the wrapper that generated the archived timings. A dry run only prints the invocation. An actual run executes the current checkout and rejects stale native source bindings; its results must be assessed independently. This packet change does not claim a fresh simulation run or additional operating-system validation.
