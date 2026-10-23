# Compact PETG cinematic: installed consumers, source-owned scene

The installed `cybrgeo.examples.arx_petg` package owns the existing compact
cache writer and cinema contract. Both use `cybrgeo.native_mesh` for canonical
ARXP serialization. This is a move of the existing consumers, not a new renderer,
mechanical framework, causal simulation or physical qualification path.

The companion source migration deletes the superseded maintained consumers from a working
copy of the checkpoint, changes its renderer/test imports to the installed
package, and preserves the frozen source archive unchanged. No renderer, kernel,
third-party runtime, geometry, camera edit or material implementation is copied
into GEO. The existing 12-test cinema suite is retained with package imports
and assertions adjusted for source-owned read-only poses. ServiceEvaluator is
the single service/reassembly pose owner; the timeline no longer duplicates
its interpolation. External-input validation and packet precision checks remain.

## Run the installed consumer

Install this GEO checkout into the chosen Python environment. The narrow mesh
and contract paths need its existing NumPy and trimesh dependencies; rendering
also needs the original checkpoint runtime dependencies (including Numba/Pillow,
compiled renderer, and OIDN when selected). The isolated tested installation used
`pip install --no-deps --no-build-isolation -e "$GEO"` with dependencies present.
Both consumers were exercised with `PYTHONPATH` unset.

```sh
# CHECKPOINT is a verified, writable working copy, not the frozen archive.
git -C "$CHECKPOINT" apply "$MIGRATION_PATCH"
ASSETS="$CHECKPOINT/production/work/cinema-production"
python -m cybrgeo.examples.arx_petg.rebuild_native \
  --geometry "$ASSETS/geometry.npz" --provenance "$ASSETS/geometry.json" \
  --output "$ASSETS/geometry-regenerated.bin"
cmp "$ASSETS/geometry-native.bin" "$ASSETS/geometry-regenerated.bin"
python -m cybrgeo.examples.arx_petg.cinema_contract --help
python "$ASSETS/render_pathtraced.py" --help
```

The package writer requires explicit geometry/provenance paths: it never looks
for assets next to installed code. The renderer retains its source-owned asset
root and uses the canonical package import. Keep full-film approval false; the
single OIDN proof is not full-film acceptance or manufacturing evidence.

## Immutable inputs and adoption blockers

The recovered source ZIP is 256,429,784 bytes with SHA-256
`ea7fc0e0e3006ec805e023941e3b8eb05b0a0dfa59c23b18c4275d849947cfca`.
Its `checkpoint-manifest.json` has SHA-256
`2c3c036c8070be8d839d0cde6b8999918910b82d6be42e45fcba73f7eb8e063b`;
that manifest pins all 298 archived files, including the original renderer,
kernel, geometry, capture and tracks. A maintained source revision applies the
migration patch; it must not rewrite the frozen archive or pretend its original
manifest covers modified files.

Full geometry NPZ SHA-256:
`7e57547cc2365a3a3411c4e64505191d0ac9b8747776fd5d9cefd6bd7de834b3`.
The independent native oracle is 125,433,432 bytes, 400 parts, SHA-256
`bbe31126100ee112dfe57b07093028cfc0ef466446d4fa9a79e008f7662ea7b8`.
The recovered 170 reconstructed components retain their disclosed provenance.

No repository-accessible immutable asset URL has been approved or established.
Until the owner chooses the source repository/release or other durable hosting
and access policy, a fresh clone alone cannot obtain the scene. The intended
asset dependency is the exact hash-pinned archive, stored once outside the GEO
wheel, not copied implementations or fabricated substitute geometry.

The two moved Python files are traceable to that archive:
- `production/work/cinema-production/rebuild_native.py` SHA-256
  `3e8f6f984d9a3f872dc22aa33854774b9477579eae5f0c1d9eaaf5106ca38225`.
- `production/work/kernel-cinema/cinema_contract.py` SHA-256
  `53e8463bac63d60e088eb60d91ba7169b46eabe76fb59a1c99405dfe6132d9fb`.

Neither moved file contains third-party notices or authorship restrictions.
Absent license metadata alone is not an ownership blocker: the user identifies
this as their work and authorized GEO integration. No license is changed.
The separately retained renderer carries PBRT Apache-2.0 and SciPy BSD notices;
it is not imported into GPL-2.0-only GEO by this migration. Its distribution
terms and third-party notices must be preserved separately.
