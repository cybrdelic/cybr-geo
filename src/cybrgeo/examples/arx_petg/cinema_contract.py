#!/usr/bin/env python3
"""Revision-bound consumer of the ARX Design Kernel for cinema previews.

This module constructs a fresh concept-preview contract, never a physical
teardown release. The source kernel's false mechanical gate is retained. Scene
transforms, renderer code and settings are inputs, not evidence of load capacity.
Uses the installed GEO package, retained ARX kernel and standard library.
"""
from __future__ import annotations

import argparse
import ast
from dataclasses import dataclass, field
import hashlib
import importlib
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import types
from typing import Any, Iterable, Mapping, Sequence

import numpy as np

from cybrgeo import native_mesh


STATUS = "selected_geometric_service_preview"
SCHEMA = "arx.cinema.contract/1"
_IMPORTED_KERNEL_BINDING: tuple[str, str] | None = None


class ContractError(ValueError):
    """A current, truthful cinema contract could not be constructed."""


class StaleRenderError(ContractError):
    """An input or rendered artifact differs from the recorded revision."""


class FrameViolation(ContractError):
    """Scene matrices or component coverage violate the selected motion scope."""


def canonical(value: Any) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":"),
                      ensure_ascii=False, allow_nan=False).encode("utf-8")


def digest(value: Any) -> str:
    return hashlib.sha256(canonical(value)).hexdigest()


def sha256_file(path: str | Path) -> str:
    h = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def read_json(path: str | Path) -> Any:
    def unique(pairs):
        out = {}
        for key, value in pairs:
            if key in out:
                raise ContractError(f"Duplicate JSON key: {key}")
            out[key] = value
        return out
    def nonfinite(value):
        raise ContractError(f"Nonfinite JSON constant: {value}")
    return json.loads(Path(path).read_text(encoding="utf-8"),
                      object_pairs_hook=unique, parse_constant=nonfinite)


def write_json(path: str | Path, value: Any) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    data = (json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + "\n").encode()
    fd, temp = tempfile.mkstemp(prefix="." + path.name + ".", dir=path.parent)
    try:
        with os.fdopen(fd, "wb") as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temp, path)
    finally:
        if os.path.exists(temp):
            os.unlink(temp)


def file_record(path: str | Path) -> dict:
    path = Path(path).resolve()
    if not path.is_file():
        raise ContractError(f"Required input is unavailable: {path}")
    return {"path": str(path), "sha256": sha256_file(path), "bytes": path.stat().st_size}


def native_geometry_digest(path: str | Path) -> dict:
    """Preserve the contract API while sharing the writer's canonical stream."""
    try:
        return native_mesh.native_geometry_digest(path)
    except ValueError as error:
        raise ContractError(str(error)) from error


def hash_bindings(records: Mapping[str, dict]) -> dict:
    """Paths aid resolution; content, labels and byte counts establish identity."""
    return {k: {"sha256": v["sha256"], "bytes": v["bytes"]}
            for k, v in sorted(records.items())}


def assert_files_current(records: Mapping[str, dict], root: str | Path | None = None) -> None:
    root = Path(root).resolve() if root else None
    changed = []
    for name, expected in records.items():
        path = Path(expected["path"])
        if not path.is_absolute():
            if root is None:
                raise StaleRenderError(f"Relative input {name} requires a bundle root")
            path = root / path
        if (not path.is_file() or path.stat().st_size != expected["bytes"]
                or sha256_file(path) != expected["sha256"]):
            changed.append(name)
    if changed:
        raise StaleRenderError("Changed or missing revision inputs: " + ", ".join(changed))


@dataclass(frozen=True)
class ContractInputs:
    kernel_root: Path
    assets_dir: Path
    timeline: Path
    backend_sources: tuple[Path, ...]
    kernel_project: Path | None = None
    geometry: Path | None = None
    display_geometry: Path | None = None
    backend_binary: Path | None = None
    driver: Path | None = None
    extra_sources: tuple[Path, ...] = ()
    fps: int = 24
    duration_s: float = 96.0
    service_window_s: tuple[float, float] = (34.0, 58.0)
    reassembly_window_s: tuple[float, float] = (76.0, 89.0)
    catalogue_window_s: tuple[float, float] = (64.0, 76.0)
    settings: dict = field(default_factory=dict)

    def normalized(self) -> "ContractInputs":
        kernel = Path(self.kernel_root).resolve()
        assets = Path(self.assets_dir).resolve()
        return ContractInputs(
            kernel_root=kernel, assets_dir=assets, timeline=Path(self.timeline).resolve(),
            backend_sources=tuple(Path(x).resolve() for x in self.backend_sources),
            kernel_project=Path(self.kernel_project).resolve() if self.kernel_project else kernel / "examples/p6b",
            geometry=Path(self.geometry).resolve() if self.geometry else assets / "geometry.npz",
            display_geometry=Path(self.display_geometry).resolve() if self.display_geometry else None,
            backend_binary=Path(self.backend_binary).resolve() if self.backend_binary else None,
            driver=Path(self.driver).resolve() if self.driver else None,
            extra_sources=tuple(Path(x).resolve() for x in self.extra_sources),
            fps=self.fps, duration_s=self.duration_s,
            service_window_s=self.service_window_s, reassembly_window_s=self.reassembly_window_s,
            catalogue_window_s=self.catalogue_window_s, settings=dict(self.settings))


def compile_kernel_snapshot(kernel_root: Path, project: Path) -> dict:
    """Use the real kernel entry point, with no CAD, solver or cache mutation."""
    global _IMPORTED_KERNEL_BINDING
    kernel_root, project = Path(kernel_root).resolve(), Path(project).resolve()
    if not (kernel_root / "arxdesign/compiler.py").is_file():
        raise ContractError("kernel_root must contain arxdesign/compiler.py")
    existing = sys.modules.get("arxdesign")
    if existing and Path(existing.__file__).resolve().parent.parent != kernel_root:
        raise ContractError("A different ARX kernel is already imported; use a fresh Python process")
    implementation_files = {p.name: sha256_file(p) for p in sorted((kernel_root/"arxdesign").glob("*.py"))}
    implementation_binding = (str(kernel_root), digest(implementation_files))
    if existing and _IMPORTED_KERNEL_BINDING != implementation_binding:
        raise ContractError("Kernel code was imported outside this contract or changed in-process; use a fresh Python process to compile current code")
    old_no_bytecode = sys.dont_write_bytecode
    sys.dont_write_bytecode = True
    sys.path.insert(0, str(kernel_root))
    try:
        compiler = importlib.import_module("arxdesign.compiler")
        _IMPORTED_KERNEL_BINDING = implementation_binding
        build = compiler.construct(project, cache=False, run_solvers=False, run_cad=False)
        build.require("concept_preview")
        snapshot = {
            "schema": "arx.cinema.fresh-kernel-snapshot/1",
            "revision": build.revision,
            "implementation": build.graph.implementation,
            "design_sha256": digest(build.design),
            "source_inputs": build.sources,
            "assembly": build.value("assembly"),
            "service_construct": build.value("service_construct"),
            "interfaces_construct": build.value("interfaces_construct"),
            "render_plan": build.value("render_plan"),
            "sizing": build.value("sizing"),
            "node_keys": {name: build.graph.evaluate(name).key for name in
                          ("assembly", "service_construct", "interfaces_construct", "render_plan", "sizing")},
            "execution": {"fresh_construction": True, "cache": False,
                          "cad_requested": False, "external_solvers_requested": False},
        }
        build.verify_inputs_unchanged()
        if {p.name: sha256_file(p) for p in sorted((kernel_root/"arxdesign").glob("*.py"))} != implementation_files:
            raise StaleRenderError("Kernel implementation changed during construction")
        return snapshot
    finally:
        sys.path.remove(str(kernel_root))
        sys.dont_write_bytecode = old_no_bytecode


def smoother(x: float) -> float:
    x = float(np.clip(x, 0.0, 1.0))
    return x * x * x * (x * (6.0 * x - 15.0) + 10.0)


class ServiceEvaluator:
    """Authoritative frozen-pose transforms for the selected geometric service.

    This evaluator does not run collision detection. It consumes only the
    existing exact polyline tracks and keeps their scope and exceptions intact.
    """
    def __init__(self, geometry: str | Path, capture: str | Path, tracks: str | Path):
        self.geometry_path = Path(geometry).resolve()
        self.capture_path = Path(capture).resolve()
        self.tracks_path = Path(tracks).resolve()
        with np.load(self.geometry_path, allow_pickle=False) as geo:
            self.offsets = np.asarray(geo["offsets"], dtype=np.int64)
            self.body_indices = np.asarray(geo["body_indices"], dtype=np.int64)
            self.part_count = len(self.offsets)
            if self.offsets.shape != (self.part_count, 2) or self.body_indices.shape != (self.part_count,):
                raise ContractError("Invalid geometry part or body arrays")
            if np.any(self.offsets[:, 1] <= 0) or np.any(self.offsets[:, 1] % 3):
                raise ContractError("Every modeled part must have nonempty triangle geometry")
        self.capture = read_json(self.capture_path)
        self.states = np.asarray(self.capture["bodies"], dtype=np.float64)
        self.track_document = read_json(self.tracks_path)
        self.tracks = self.track_document["tracks"]
        self.pose_frame = int(self.track_document["pose_capture_frame"])
        self.pose_time_s = float(self.track_document["pose_time_s"])
        if self.states.ndim != 3 or self.states.shape[2] != 12 or not np.all(np.isfinite(self.states)):
            raise ContractError("Capture must contain finite body translation/rotation rows")
        if not 0 <= self.pose_frame < len(self.states):
            raise ContractError("Service capture frame is out of range")
        if np.any(self.body_indices < 0) or np.any(self.body_indices >= self.states.shape[1]):
            raise ContractError("Part references a missing captured body")
        self.base = self.pose_rows(self.states[self.pose_frame])
        self._validate_tracks()
        self.visible = np.arange(self.part_count)
        self.visible.flags.writeable = False

    def pose_rows(self, rows: np.ndarray) -> np.ndarray:
        bodies = np.repeat(np.eye(4)[None], len(rows), axis=0)
        for i, row in enumerate(rows):
            u, _, vt = np.linalg.svd(row[3:].reshape(3, 3))
            r = u @ vt
            if np.linalg.det(r) < 0.999999:
                raise ContractError("Captured body rotation reverses orientation")
            bodies[i, :3, :3] = r
            bodies[i, :3, 3] = row[:3]
        return bodies[self.body_indices].copy()

    def _validate_tracks(self) -> None:
        seen: set[int] = set()
        prior_end = 0.0
        for index, track in enumerate(self.tracks):
            ids = track["part_ids"]
            if not ids or len(set(ids)) != len(ids) or any(type(i) is not int for i in ids):
                raise ContractError(f"Invalid track membership: {track.get('id', index)}")
            if set(ids) & seen or any(i < 0 or i >= self.part_count for i in ids):
                raise ContractError("Service tracks repeat a part or reference a missing component")
            seen.update(ids)
            start, end = float(track["start"]), float(track["end"])
            if abs(start - prior_end) > 1e-10 or end <= start or end > 1 + 1e-10:
                raise ContractError("Service tracks must be sequential and contiguous in [0,1]")
            prior_end = end
            points = np.asarray(track["waypoints_world_m"], dtype=np.float64)
            fractions = np.asarray(track["waypoint_fractions"], dtype=np.float64)
            if points.ndim != 2 or points.shape[1] != 3 or len(points) < 2 or not np.all(np.isfinite(points)):
                raise ContractError("Service waypoints must be finite 3D points")
            if np.max(np.abs(points[0])) > 1e-12:
                raise ContractError("Each unique part track must start at its captured position")
            lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
            if np.any(lengths <= 0) or fractions.shape != (len(points),):
                raise ContractError("Invalid or empty service segment")
            expected = np.r_[0, np.cumsum(lengths)] / sum(lengths)
            if np.max(np.abs(fractions - expected)) > 1e-9:
                raise ContractError("Waypoint fractions must match the exact segment lengths")
        if not self.tracks or abs(prior_end - 1) > 1e-10:
            raise ContractError("Service progress must cover [0,1]")
        recorded = self.track_document.get("pose_body_transforms")
        if recorded is not None and not np.allclose(recorded, self.states[self.pose_frame], atol=1e-12, rtol=0):
            raise ContractError("Service pose no longer matches captured body transforms")
        self.moving_part_ids = sorted(seen)

    def matrices(self, progress: float) -> np.ndarray:
        if not math.isfinite(progress) or not 0 <= progress <= 1:
            raise FrameViolation("Service progress must be finite and in [0,1]")
        out = self.base.copy()
        for track in self.tracks:
            f = float(np.clip((progress - track["start"]) / (track["end"] - track["start"]), 0, 1))
            if f <= 0:
                continue
            points = np.asarray(track["waypoints_world_m"], dtype=np.float64)
            fractions = np.asarray(track["waypoint_fractions"], dtype=np.float64)
            j = min(len(points) - 2, max(0, int(np.searchsorted(fractions, f, side="right")) - 1))
            u = smoother((f - fractions[j]) / (fractions[j + 1] - fractions[j]))
            offset = points[j] * (1-u) + points[j+1] * u
            out[track["part_ids"], :3, 3] += offset
        out.flags.writeable = False
        return out

    def validate_frame(self, matrices: np.ndarray, visible: Iterable[int], progress: float,
                       *, atol: float = 2e-7) -> dict:
        matrices = np.asarray(matrices, dtype=np.float64)
        visible = list(visible)
        if matrices.shape != self.base.shape or not np.all(np.isfinite(matrices)):
            raise FrameViolation("Frame has invalid or nonfinite component matrices")
        if len(visible) != self.part_count or set(visible) != set(range(self.part_count)):
            raise FrameViolation("Every source part must remain in the service/reassembly draw list")
        error = float(np.max(np.abs(matrices - self.matrices(float(progress)))))
        if error > atol:
            raise FrameViolation(f"Frame leaves exact service tracks or changes rotations: error={error:.9g}")
        return {"passed": True, "maximum_matrix_error": error, "parts": self.part_count}


def _timeline_module(path: Path, assets_dir: Path) -> types.ModuleType:
    """Load the actual Geometry/Edit code without instantiating its old renderer."""
    source = path.read_text(encoding="utf-8")
    tree = ast.parse(source, filename=str(path))
    # The unchanged historical timeline file also contains a rasterizer. Its
    # unused graphics import is omitted for matrix-only validation; no function
    # or class body, scene parameter, geometry path or interpolation is patched.
    tree.body = [node for node in tree.body if not (
        isinstance(node, ast.Import) and len(node.names) == 1 and node.names[0].name == "moderngl")]
    module = types.ModuleType("arx_cinema_checked_timeline")
    module.__file__ = str(path)
    old_bytecode = sys.dont_write_bytecode
    sys.dont_write_bytecode = True
    try:
        exec(compile(tree, str(path), "exec"), module.__dict__)
    finally:
        sys.dont_write_bytecode = old_bytecode
    module.ROOT = assets_dir
    module.WORK = assets_dir.parent
    module.SOURCE = assets_dir.parent / "source/ARX6-P5-Refined/ARX6-P5-Refined"
    return module


def audit_timeline(spec: ContractInputs, evaluator: ServiceEvaluator | None = None) -> dict:
    spec = spec.normalized()
    evaluator = evaluator or ServiceEvaluator(spec.geometry, spec.assets_dir / "capture.json", spec.assets_dir / "service_tracks.json")
    module = _timeline_module(spec.timeline, spec.assets_dir)
    old_geometry = os.environ.get("ARX_GEOMETRY")
    os.environ["ARX_GEOMETRY"] = str(spec.display_geometry or spec.geometry)
    try:
        geometry = module.Geometry()
    finally:
        if old_geometry is None:
            os.environ.pop("ARX_GEOMETRY", None)
        else:
            os.environ["ARX_GEOMETRY"] = old_geometry
    if geometry.count != evaluator.part_count or not np.array_equal(geometry.bodies, evaluator.body_indices):
        raise FrameViolation("Timeline and full model part/body identities differ")
    edit = module.Edit(geometry, evaluator)
    frame_count = int(round(spec.fps * spec.duration_s))
    if spec.fps <= 0 or frame_count <= 0 or abs(frame_count / spec.fps - spec.duration_s) > 1e-10:
        raise ContractError("Timeline duration must contain an integral number of frames")
    matrix_hasher = hashlib.sha256()
    coverage: set[int] = set()
    counts = {"service": 0, "reassembly": 0, "catalogue": 0}
    for frame in range(frame_count):
        t = frame / spec.fps
        role = None
        if spec.service_window_s[0] <= t < spec.service_window_s[1]:
            role = "service"
        elif spec.reassembly_window_s[0] <= t < spec.reassembly_window_s[1]:
            role = "reassembly"
        elif spec.catalogue_window_s[0] <= t < spec.catalogue_window_s[1]:
            role = "catalogue"
        if role is None:
            continue
        state = edit.frame(t)
        counts[role] += 1
        if role == "catalogue":
            ids = list(state["visible"])
            if len(ids) != len(set(ids)) or any(i < 0 or i >= evaluator.part_count for i in ids):
                raise FrameViolation("Catalogue includes duplicate or invalid part IDs")
            coverage.update(ids)
        else:
            matrix_hasher.update(np.asarray(state["mats"], dtype="<f8").tobytes())
    missing = sorted(set(range(evaluator.part_count)) - coverage)
    if missing:
        raise FrameViolation(f"Catalogue omitted source components: {missing}")
    if any(counts[k] == 0 for k in counts):
        raise FrameViolation("Timeline does not cover every required frame role")
    if not np.allclose(edit.frame(spec.reassembly_window_s[0])["mats"], evaluator.matrices(1), atol=2e-7, rtol=0):
        raise FrameViolation("Reassembly does not start at the extracted endpoint")
    if not np.allclose(edit.frame(spec.reassembly_window_s[1])["mats"], evaluator.base, atol=2e-7, rtol=0):
        raise FrameViolation("Reassembly does not finish at the assembled pose")
    return {"schema": "arx.cinema.timeline-audit/1", "passed": True,
            "frame_count": frame_count, "fps": spec.fps, "duration_s": spec.duration_s,
            "counts": counts, "part_count": evaluator.part_count,
            "catalogue_unique_part_count": len(coverage), "catalogue_missing_part_ids": missing,
            "matrix_source": "ServiceEvaluator.matrices",
            "service_and_reassembly_matrices_sha256": matrix_hasher.hexdigest(),
            "all_service_parts_visible": True, "reassembly_endpoints_match": True,
            "collision_queries_executed": False, "continuous_collision_certified": False,
            "loader": "Actual Geometry/Edit definitions; unused rasterizer import omitted; assets root supplied explicitly",
            "scope": "Constructs service poses once; checks catalogue coverage and endpoints, not encoded pixels, support loads, or real service feasibility"}


def teardown_obligations(snapshot: dict, source_audit: dict, tracks: dict) -> dict:
    service = snapshot["service_construct"]
    unresolved = source_audit.get("unresolved_interfaces", [])
    obligations = []
    for op in service["operations"]:
        obligations.append({
            "id": op["id"], "after": op.get("after", []), "part_ids": op.get("parts", []),
            "state": "unresolved", "source_energy_state": op.get("energy_state"),
            "path_defined": op.get("path_defined", False), "support_defined": op.get("support_defined", False),
            "notes": op.get("notes", ""),
            "required_evidence": (["power_isolation_and_payload_removal_procedure"] if op["id"] == "isolate_power_and_payload" else
                                  ["support_fixture_geometry", "fixture_load_and_retention_analysis", "tool_access", "continuous_swept_clearance"])
                                  + (["spring_restraint_and_unloading_tool", "residual_energy_check"] if "preload" in op["id"] else [])})
    obligations.extend([
        {"id": "real_fastener_release", "after": ["isolate_power_and_payload"], "state": "unresolved",
         "required_evidence": ["individual_fastener_axes_and_pitches", "driver_access_sweeps", "helical_withdrawal_or_documented_captive_release"],
         "current_scope": "Grouped nominal screw envelopes translate without thread-contact simulation"},
        {"id": "disconnect_services", "after": ["isolate_power_and_payload", "support_distal_links"], "state": "unresolved",
         "required_evidence": ["electrical_connector_map", "cable_routes", "connector_access_and_separation_paths"]},
        {"id": "continuous_solid_clearance", "after": ["support_distal_links"], "state": "unresolved",
         "required_evidence": ["baseline_contact_classification", "continuous_swept_solid_queries", "solid_containment_check", "declared_minimum_clearance"],
         "current_scope": "Discrete full-mesh triangle intersection sampling with nominal mate exceptions"},
    ])
    return {"schema": "arx.cinema.full-teardown-obligations/1",
            "kernel_revision": snapshot["revision"], "mechanical_teardown_animation_allowed": service["mechanical_teardown_animation_allowed"],
            "source_gate_modified": False, "service_blockers": service["blockers"],
            "assembly_blockers": snapshot["assembly"]["blockers"], "operations": obligations,
            "unresolved_interfaces": unresolved,
            "sealed_reducer_pairs": source_audit.get("sealed_reducer_pairs", []),
            "counterbalance_keep_sealed_ids": source_audit.get("counterbalance_keep_sealed_ids", []),
            "preview_track_preconditions": [{"id": t["id"], "preconditions": t.get("preconditions", [])} for t in tracks["tracks"]],
            "do_not_infer": ["physical_payload_rating", "printed_reducer_function", "full_real_life_teardown", "support_or_tool_access_validation"]}


def _contract_identity(contract: Mapping[str, Any]) -> str:
    return digest({"schema": SCHEMA, "status": contract["status"],
                   "bindings": hash_bindings(contract["inputs"]),
                   "kernel_snapshot_sha256": contract["kernel_snapshot_sha256"],
                   "timeline_audit_sha256": contract["timeline_audit_sha256"],
                   "full_teardown_obligations_sha256": contract["full_teardown_obligations_sha256"],
                   "settings": contract["settings"], "scope": contract["scope"]})


def build_contract(inputs: ContractInputs, *, run_frame_audit: bool = True) -> dict:
    """Build a preview gate bound to current sources; no physical gate is opened."""
    spec = inputs.normalized()
    if not spec.backend_sources:
        raise ContractError("At least one actual backend source must be bound")
    records = {"kernel.design": file_record(spec.kernel_project / "design.json"),
               "model.full_npz": file_record(spec.geometry),
               "model.provenance": file_record(spec.assets_dir / "geometry.json"),
               "motion.capture": file_record(spec.assets_dir / "capture.json"),
               "motion.tracks": file_record(spec.assets_dir / "service_tracks.json"),
               "motion.saved_evidence": file_record(spec.assets_dir / "service_tracks_validation.json"),
               "motion.source_audit": file_record(spec.assets_dir / "assembly_source_audit.json"),
               "timeline.source": file_record(spec.timeline),
               "adapter.source": file_record(Path(__file__)),
               "mesh.serializer": file_record(Path(native_mesh.__file__))}
    for path in sorted((spec.kernel_root / "arxdesign").glob("*.py")):
        records["kernel.code/" + path.name] = file_record(path)
    design = read_json(spec.kernel_project / "design.json")
    for name, relative in sorted(design["sources"].items()):
        path = (spec.kernel_project / relative).resolve()
        if not path.is_relative_to(spec.kernel_project.resolve()):
            raise ContractError("Kernel source escapes its project")
        records["kernel.input/" + name] = file_record(path)
    for index, path in enumerate(spec.backend_sources):
        records[f"backend.source/{index:03d}/{path.name}"] = file_record(path)
    for index, path in enumerate(spec.extra_sources):
        records[f"extra.source/{index:03d}/{path.name}"] = file_record(path)
    for label, path in (("model.display_npz", spec.display_geometry), ("backend.binary", spec.backend_binary), ("render.driver", spec.driver)):
        if path is not None:
            records[label] = file_record(path)
    snapshot = compile_kernel_snapshot(spec.kernel_root, spec.kernel_project)
    service_gate = snapshot["service_construct"]
    if service_gate.get("mechanical_teardown_animation_allowed") is not False:
        raise ContractError("This adapter is restricted to the unresolved preview branch; review any changed mechanical gate explicitly")
    if not snapshot["assembly"]["capabilities"].get("concept_preview"):
        raise ContractError("The kernel does not permit the concept preview capability")
    evaluator = ServiceEvaluator(spec.geometry, spec.assets_dir / "capture.json", spec.assets_dir / "service_tracks.json")
    if evaluator.part_count != len(design["inventory"]):
        raise ContractError("Cinematic model and kernel component inventories differ")
    metadata = read_json(spec.assets_dir / "geometry.json")
    native = native_geometry_digest(spec.geometry)
    if native["sha256"] != metadata.get("output_native_sha256"):
        raise ContractError("Full NPZ no longer reproduces the geometry used by saved service evidence")
    if metadata.get("kernel_render_record_sha256") != records["kernel.input/render_inventory"]["sha256"]:
        raise ContractError("Retained model's source inventory differs from the current kernel inventory")
    tracks = evaluator.track_document
    evidence = read_json(spec.assets_dir / "service_tracks_validation.json")
    if tracks.get("geometry_sha256") != metadata.get("output_native_sha256") or evidence.get("geometry_sha256") != metadata.get("output_native_sha256"):
        raise ContractError("Saved service evidence references a different source geometry")
    if len(evidence.get("validations", [])) != sum(len(t["waypoints_world_m"])-1 for t in tracks["tracks"]):
        raise ContractError("Service evidence and selected segment count differ")
    for track in tracks["tracks"]:
        indices = track.get("validation_segment_indices", [])
        if len(indices) != len(track["waypoints_world_m"])-1:
            raise ContractError("Track lacks an explicit evidence mapping")
        for j, index in enumerate(indices):
            item = evidence["validations"][index]
            delta = np.asarray(track["waypoints_world_m"][j+1])-track["waypoints_world_m"][j]
            expected = np.asarray(item["direction_world"]) * item["travel_m"]
            if sorted(item["part_ids"]) != sorted(track["part_ids"]) or not np.allclose(delta, expected, atol=1e-10, rtol=0):
                raise ContractError("Track segment no longer matches its saved sampled evidence")
            if item["sampled_collisions"] or not item["sampled_path_clear"]:
                raise ContractError("Selected service track contains a recorded nonexempt collision")
    audit = audit_timeline(spec, evaluator) if run_frame_audit else {"passed": False, "not_executed": True}
    source_audit = read_json(spec.assets_dir / "assembly_source_audit.json")
    obligations = teardown_obligations(snapshot, source_audit, tracks)
    scope = {
        "design_preview_only": True,
        "kernel_concept_preview_allowed": True,
        "kernel_mechanical_teardown_animation_allowed": False,
        "physical_teardown_gate_modified": False,
        "validated_operating_film": False,
        "geometry_part_count": evaluator.part_count,
        "full_geometry_native_sha256": native["sha256"],
        "full_geometry_triangle_count": native["triangles"],
        "service_moving_part_count": len(evaluator.moving_part_ids),
        "service_track_count": len(evaluator.tracks),
        "service_segment_count": len(evidence["validations"]),
        "continuous_collision_certified": False,
        "solid_containment_checked": False,
        "all_parts_have_extraction_paths": False,
        "geometry_reconstruction": {
            k: metadata[k] for k in ("original_P5_mesh_parts", "original_P6B_BREP_parts", "P6A_reconstructed_parts")},
        "required_status_text": "ENGINEERING UNRESOLVED — DESIGN PREVIEW",
        "status_text_source": "fresh kernel render_plan",
        "geometry_use": "Retained P6B prototype mesh; no new passing geometry or payload capacity inferred",
        "frame_audit_executed": bool(audit["passed"]),
    }
    contract = {"schema": SCHEMA, "status": STATUS,
                "preview_render_allowed": bool(audit["passed"]),
                "inputs": records, "kernel_snapshot": snapshot,
                "kernel_snapshot_sha256": digest(snapshot),
                "current_compiled_service_gate_sha256": digest(service_gate),
                "timeline_audit": audit, "timeline_audit_sha256": digest(audit),
                "settings": {"fps": spec.fps, "duration_s": spec.duration_s,
                             "service_window_s": list(spec.service_window_s),
                             "reassembly_window_s": list(spec.reassembly_window_s),
                             "catalogue_window_s": list(spec.catalogue_window_s),
                             "render": spec.settings},
                "scope": scope, "full_teardown_obligations": obligations,
                "full_teardown_obligations_sha256": digest(obligations),
                "evidence_scope": "Existing sampled geometric service evidence is reused under hashes; this adapter does not rerun it"}
    contract["contract_id"] = _contract_identity(contract)
    assert_files_current(records)
    return contract


def require_current(contract: Mapping[str, Any], *, root: str | Path | None = None) -> None:
    if contract.get("schema") != SCHEMA or contract.get("status") != STATUS:
        raise StaleRenderError("Unsupported cinema contract")
    if _contract_identity(contract) != contract.get("contract_id"):
        raise StaleRenderError("Contract identity does not match its recorded contents")
    if digest(contract["kernel_snapshot"]) != contract["kernel_snapshot_sha256"]:
        raise StaleRenderError("Compiled kernel snapshot was altered")
    if digest(contract["timeline_audit"]) != contract["timeline_audit_sha256"]:
        raise StaleRenderError("Timeline audit was altered")
    if digest(contract["full_teardown_obligations"]) != contract["full_teardown_obligations_sha256"]:
        raise StaleRenderError("Full-teardown obligations were altered")
    if digest(contract["kernel_snapshot"]["service_construct"]) != contract["current_compiled_service_gate_sha256"]:
        raise StaleRenderError("Current compiled service gate digest was altered")
    if not contract.get("preview_render_allowed") or not contract["timeline_audit"].get("passed"):
        raise ContractError("Preview frame contract was not executed successfully")
    if contract["scope"]["kernel_mechanical_teardown_animation_allowed"] or contract["kernel_snapshot"]["service_construct"]["mechanical_teardown_animation_allowed"]:
        raise ContractError("This contract must not turn a preview into a physical teardown release")
    assert_files_current(contract["inputs"], root)


def new_render_manifest(contract: Mapping[str, Any], *, outputs: Sequence[str | Path] = ()) -> dict:
    require_current(contract)
    return {"schema": "arx.cinema.render-manifest/1", "status": "ready" if not outputs else "rendered",
            "contract_id": contract["contract_id"], "cinema_scope": STATUS,
            "settings": contract["settings"], "kernel_revision": contract["kernel_snapshot"]["revision"],
            "mechanical_teardown_animation_allowed": False,
            "outputs": [file_record(path) for path in outputs]}


def verify_render_manifest(manifest: Mapping[str, Any], contract: Mapping[str, Any],
                           *, root: str | Path | None = None, verify_outputs: bool = True) -> dict:
    require_current(contract, root=root)
    if manifest.get("status") == "invalidated" or manifest.get("contract_id") != contract["contract_id"]:
        raise StaleRenderError("Render manifest belongs to an invalidated or different cinema revision")
    if manifest.get("settings") != contract["settings"] or manifest.get("cinema_scope") != STATUS:
        raise StaleRenderError("Render settings or claim scope changed")
    if manifest.get("mechanical_teardown_animation_allowed") is not False:
        raise ContractError("Render manifest attempts to claim physical teardown permission")
    if verify_outputs:
        assert_files_current({f"output/{i}": item for i, item in enumerate(manifest.get("outputs", []))}, root)
    return {"current": True, "contract_id": contract["contract_id"], "outputs_checked": len(manifest.get("outputs", [])) if verify_outputs else 0}


def invalidate_if_stale(manifest_path: str | Path, contract: Mapping[str, Any],
                        *, root: str | Path | None = None) -> dict:
    """Mark stale work unusable without deleting its output files or evidence."""
    path = Path(manifest_path)
    manifest = read_json(path)
    try:
        result = verify_render_manifest(manifest, contract, root=root)
    except (ContractError, OSError) as exc:
        manifest["status"] = "invalidated"
        manifest["invalidation"] = {"reason": str(exc), "expected_contract_id": contract.get("contract_id"),
                                    "outputs_deleted": False}
        write_json(path, manifest)
        return {"current": False, "invalidated": True, "reason": str(exc)}
    return result


def _cli() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    build = commands.add_parser("build")
    build.add_argument("--kernel-root", type=Path, required=True)
    build.add_argument("--kernel-project", type=Path)
    build.add_argument("--assets-dir", type=Path, required=True)
    build.add_argument("--timeline", type=Path, required=True)
    build.add_argument("--backend-source", type=Path, action="append", required=True)
    build.add_argument("--backend-binary", type=Path)
    build.add_argument("--driver", type=Path)
    build.add_argument("--geometry", type=Path)
    build.add_argument("--display-geometry", type=Path)
    build.add_argument("--extra-source", type=Path, action="append", default=[])
    build.add_argument("--settings", type=Path)
    build.add_argument("--output", type=Path, required=True)
    check = commands.add_parser("check")
    check.add_argument("--contract", type=Path, required=True)
    check.add_argument("--manifest", type=Path)
    check.add_argument("--root", type=Path)
    check.add_argument("--invalidate", action="store_true")
    args = parser.parse_args()
    try:
        if args.command == "build":
            spec = ContractInputs(kernel_root=args.kernel_root, kernel_project=args.kernel_project,
                                  assets_dir=args.assets_dir, timeline=args.timeline,
                                  backend_sources=tuple(args.backend_source), backend_binary=args.backend_binary,
                                  driver=args.driver, geometry=args.geometry, display_geometry=args.display_geometry,
                                  extra_sources=tuple(args.extra_source),
                                  settings=read_json(args.settings) if args.settings else {})
            contract = build_contract(spec)
            write_json(args.output, contract)
            write_json(args.output.with_name("full-teardown-obligations.json"), contract["full_teardown_obligations"])
            print(json.dumps({"contract": str(args.output.resolve()), "contract_id": contract["contract_id"],
                              "status": contract["status"], "preview_render_allowed": contract["preview_render_allowed"],
                              "physical_teardown_allowed": False, "timeline_audit": contract["timeline_audit"]}))
        else:
            contract = read_json(args.contract)
            if args.manifest and args.invalidate:
                result = invalidate_if_stale(args.manifest, contract, root=args.root)
                print(json.dumps(result))
                return 0 if result["current"] else 2
            require_current(contract, root=args.root)
            result = verify_render_manifest(read_json(args.manifest), contract, root=args.root) if args.manifest else {"current": True, "contract_id": contract["contract_id"]}
            print(json.dumps(result))
    except (ContractError, OSError, ValueError) as exc:
        print(json.dumps({"error": type(exc).__name__, "message": str(exc)}), file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(_cli())
