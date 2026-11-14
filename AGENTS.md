# Working on CYBR GEO

Read [the modeling workflow](docs/AGENT_MODELING.md) before building or revising an
asset. Read [the construction API](docs/CORE_WORKFLOW.md) and
[physical assembly contract](docs/PHYSICAL_ASSEMBLIES.md) before claiming protection.

- Preserve the small general-purpose core. Object-specific design belongs in
  examples; simulations and image-fitting workflows belong outside the core.
  Do not add boot, car, tree or other asset factories to fix an example.
- Inspect the actual source and installed package before choosing operations.
  Do not silently replace a stronger earlier construction with simpler panels.
  Distinguish an asset regression from a core regression using the same authoring
  program on both core versions before drawing a conclusion.
- Build primary form first, actual construction second, detail third. Use shared
  curves, surfaces and measured features, not independently guessed coordinates.
  More polygons, smoothing, textures and lighting cannot repair wrong structure.
- Use the protected construction path for applicable physical assemblies. Do not
  bypass a rejection, blanket-allow overlaps, or treat `assume_valid_solid=True`
  as a test. Report unsupported contacts and unprotected output honestly.
- Preserve source and previous good results. Inspect multiple clay views and
  sections of the actual result. Passing tests is not proof of visual quality,
  physical function, structural strength or an “AAA” standard.
- Preserve normals, UVs and correspondence when exporting; verify what the
  receiving renderer actually loads. Keep all deliverables tied to one source
  and core revision. Never present stale media as a new build.
- A construction film must show operations in the 3D view, including intermediate
  and removed geometry, not merely reveal completed parts. Explain setup lines
  honestly rather than inventing a geometric effect for an import or assignment.
- Fix a core defect with a minimal reproducer and a general fix. Use focused local
  tests and relevant examples; do not introduce slow CI as a substitute for
  inspecting geometry or as an unnecessary prerequisite to showing progress.

## Surface-network additions

Read [SURFACE_API.md](SURFACE_API.md) for the additional APIs. Earlier releases
do not necessarily include these exports; record the actual imported revision. SurfaceNetwork seam constraints
and joined tessellation are not automatic closed-solid construction or design
quality guarantees. Preserve returned corner normals and UVs in render/export.

Final analytic physical assemblies must pass `Assembly.build()` and use its
full-part export, not a hand-picked pair check. Read
[FINAL_ASSEMBLY.md](docs/FINAL_ASSEMBLY.md). Declare every final Solid and intended
contact; a contact declaration never waives positive-volume intersection. Raw
mesh/Graph outputs are not equivalent to this analytic finalization contract.
