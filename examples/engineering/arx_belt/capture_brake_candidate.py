"""Render paired ARX brake diagnostics through GEO's shared native replay.

The power-loss diagnostic is not an accepted nominal transfer run. This entry
point reads its two recorded traces and exact candidate CAD directly, then
renders them with the same renderer, quality and camera. No pose is invented.
"""
from __future__ import annotations

import argparse
from copy import deepcopy
import gzip
import hashlib
import json
from pathlib import Path
import subprocess

import numpy as np

from mechanism_lab.engineering_media import frame_schedule, verify_replay
from mechanism_lab.engineering_source import assert_project_source_identity
from mechanism_lab.media import probe
from mechanism_lab.physical_model import digest
from mechanism_lab.render import Studio, labelled
from mechanism_lab.truth import assert_renderable

from .gripper_retention import CANDIDATE_REVISION
from .project import create


SIZE = (1920, 1080)
FPS = 24
SUPERSAMPLE = 2
TRACE_NAMES = ("brake_engaged", "pinion_brake_disabled")


def _sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _trace(path, expected):
    if _sha(path) != expected:
        raise ValueError(f"Candidate trace changed: {path.name}")
    with gzip.open(path, "rt", encoding="utf-8") as stream:
        return json.load(stream)


def _video(assembly, name, camera, output):
    times = assembly.metadata["recorded_times_s"]
    schedule = frame_schedule(times, FPS)
    view = deepcopy(assembly.views["hero"])
    view.az, view.el, view.scale, view.target = camera
    view.floor = False
    final = output / f"{name}.mp4"
    partial = output / f"{name}.partial.mp4"
    command = ["ffmpeg", "-y", "-v", "error", "-threads", "2", "-f", "rawvideo",
               "-pix_fmt", "rgb24", "-s", f"{SIZE[0]}x{SIZE[1]}",
               "-r", str(FPS), "-i", "-", "-an", "-c:v", "libx264",
               "-threads", "2", "-preset", "medium", "-crf", "17",
               "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(partial)]
    studio = None
    with (output / f"{name}-encoding.log").open("wb") as errors:
        encoder = subprocess.Popen(command, stdin=subprocess.PIPE, stderr=errors)
        try:
            studio = Studio(assembly, size=SIZE, floor=False,
                            supersample=SUPERSAMPLE, exposure=view.exposure)
            studio.set_view(view)
            for row in schedule:
                studio.pose(row["recorded_time_s"])
                raw = studio.render()
                captioned = labelled(raw, "ARX BELT / BRAKE DIAGNOSTIC",
                    f'{name.replace("_", " ")} / recorded t={row["recorded_time_s"]:.4f} s',
                    "MuJoCo/CAD model only; brake hardware and manufacturing unqualified",
                    tag="CYBR GEO / RECORDED-STATE COMPARISON")
                encoder.stdin.write(np.ascontiguousarray(captioned, dtype=np.uint8).tobytes())
                row["pixel_sha256_before_caption"] = hashlib.sha256(raw.tobytes()).hexdigest()
                if row["video_frame"] % FPS == 0 or row is schedule[-1]:
                    print(json.dumps(dict(trace=name, rendered_frames=row["video_frame"] + 1,
                                          total_frames=len(schedule))), flush=True)
            encoder.stdin.close()
            if encoder.wait() != 0:
                raise RuntimeError(f"FFmpeg failed; inspect {name}-encoding.log")
            partial.replace(final)
        except BaseException:
            if encoder.stdin and not encoder.stdin.closed:
                try:
                    encoder.stdin.close()
                except (BrokenPipeError, OSError):
                    pass
            if encoder.poll() is None:
                encoder.kill()
            encoder.wait()
            partial.unlink(missing_ok=True)
            raise
        finally:
            if studio is not None:
                studio.close()
    streams = probe(final).get("streams", [])
    if (len(streams) != 1 or int(streams[0]["width"]) != SIZE[0]
            or int(streams[0]["height"]) != SIZE[1]
            or int(streams[0]["nb_read_frames"]) != len(schedule)):
        raise ValueError(f"{name} encoded dimensions/frame count differ from replay")
    numerator, denominator = map(int, streams[0]["r_frame_rate"].split("/"))
    if numerator / denominator != FPS:
        raise ValueError(f"{name} encoded frame rate differs from replay")
    frame_path = output / f"{name}-frames.json"
    frame_path.write_text(json.dumps(schedule, indent=2) + "\n", encoding="utf-8")
    return dict(path=final.name, sha256=_sha(final),
                frame_map_sha256=_sha(frame_path), frame_count=len(schedule),
                recorded_duration_s=times[-1]-times[0],
                resolution=list(SIZE), fps=FPS, supersample=SUPERSAMPLE,
                codec="libx264", crf=17, preset="medium",
                sampling="preceding recorded state; no interpolation",
                probe=probe(final))


def capture(run, output):
    run, output = Path(run).resolve(), Path(output).resolve()
    report_path = run / "candidate-verification.json"
    report = json.loads(report_path.read_text(encoding="utf-8"))
    if (report.get("schema") != "cybrgeo-arx-brake-candidate-diagnostic/1"
            or report.get("candidate_revision") != CANDIDATE_REVISION
            or report.get("passed_software_diagnostic") is not True
            or report.get("manufacturing_qualified") is not False
            or report.get("posed_clearance", {}).get("passed") is not True
            or report.get("motor_off_loaded_puck", {}).get("passed") is not True):
        raise ValueError("Media requires the current passing native and loaded-fault report")
    if output.exists() and any(output.iterdir()):
        raise ValueError("Use an empty diagnostic media directory")
    project = create()
    specification = deepcopy(project.specification)
    specification["gripper_retention_revision"] = CANDIDATE_REVISION
    if digest(specification) != report["candidate_specification_sha256"]:
        raise ValueError("Candidate recipe differs from its native and fault report")
    native = run / "native"
    receipt = json.loads((native / "native-build.json").read_text(encoding="utf-8"))
    if digest(receipt) != report["native_build_sha256"]:
        raise ValueError("Native CAD receipt differs from the loaded-fault report")
    output.mkdir(parents=True, exist_ok=True)
    videos, checks, trace_hashes = {}, {}, {}
    traces, assemblies, bounds = {}, {}, []
    source_files = None
    recorded_times = None
    first_view = None
    for name in TRACE_NAMES:
        path = run / f"{name}-trace.json.gz"
        trace = _trace(path, report["traces_sha256"][path.name])
        if trace.get("model_sha256") != report["model_sha256"]:
            raise ValueError(f"{name} model differs from the candidate report")
        if source_files is None:
            source_files = trace["reference_source"]["source_files"]
            assert_project_source_identity(Path(__file__).parent, source_files)
        elif trace["reference_source"]["source_files"] != source_files:
            raise ValueError("Paired traces use different authored project sources")
        if name == TRACE_NAMES[0] and trace.get("control_sha256") != report["control_sha256"]:
            raise ValueError("Brake-engaged trace control differs from the candidate report")
        assembly = project.replay(native, trace)
        checks[name] = verify_replay(assembly, trace)
        assert_renderable(assembly, intent="inspection")
        times = assembly.metadata["recorded_times_s"]
        if recorded_times is None:
            recorded_times = times
            first_view = assembly.views["hero"]
        elif times != recorded_times:
            raise ValueError("Paired candidate traces have different recorded time axes")
        bounds.append(np.asarray(assembly.metadata["render_bounds_mm"], dtype=float))
        traces[name] = trace
        assemblies[name] = assembly
        trace_hashes[path.name] = _sha(path)
    del assembly
    union = np.array([np.min([row[0] for row in bounds], axis=0),
                      np.max([row[1] for row in bounds], axis=0)])
    if not np.isfinite(union).all() or np.any(union[1] <= union[0]):
        raise ValueError("Candidate motion has invalid recorded render bounds")
    camera = (first_view.az, first_view.el, float(max(union[1] - union[0]) * .62),
              tuple(union.mean(axis=0)))
    for name in TRACE_NAMES:
        videos[name] = _video(assemblies.pop(name), name, camera, output)
    if videos[TRACE_NAMES[0]]["frame_count"] != videos[TRACE_NAMES[1]]["frame_count"]:
        raise ValueError("Paired candidate traces have unequal video frame counts")
    comparison = output / "brake-comparison.mp4"
    comparison_partial = output / "brake-comparison.partial.mp4"
    subprocess.run(["ffmpeg", "-y", "-v", "error",
        "-i", str(output / "brake_engaged.mp4"),
        "-i", str(output / "pinion_brake_disabled.mp4"),
        "-filter_complex", "hstack=inputs=2", "-an", "-c:v", "libx264",
        "-threads", "2", "-preset", "medium", "-crf", "17",
        "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(comparison_partial)], check=True)
    comparison_partial.replace(comparison)
    compared = probe(comparison)["streams"][0]
    if (int(compared["width"]) != 2*SIZE[0]
            or int(compared["height"]) != SIZE[1]
            or int(compared["nb_read_frames"]) != videos[TRACE_NAMES[0]]["frame_count"]):
        raise ValueError("Side-by-side video differs from its two recorded clips")
    commit = subprocess.run(["git", "rev-parse", "HEAD"], capture_output=True,
                            text=True, check=True).stdout.strip()
    assert_project_source_identity(Path(__file__).parent, source_files)
    result = dict(schema="cybrgeo-arx-brake-diagnostic-media/1",
        source_checkout_commit=commit, source_commit=report["source_commit"],
        candidate_revision=CANDIDATE_REVISION,
        report_sha256=_sha(report_path), native_build_sha256=digest(receipt),
        native_source_binding_sha256=report["native_source_binding_sha256"],
        candidate_specification_sha256=digest(specification),
        trace_sha256=trace_hashes,
        trace_control_sha256={name: traces[name]["control_sha256"] for name in TRACE_NAMES},
        project_source_files_sha256=source_files, replay_checks=checks,
        shared_builders=["native_replay.build_assembly", "physical_replay.build_assembly"],
        same_camera=dict(azimuth_deg=camera[0], elevation_deg=camera[1],
                         half_height_mm=camera[2], target_mm=list(camera[3])),
        videos=videos, comparison=dict(path=comparison.name, sha256=_sha(comparison),
            frame_count=int(compared["nb_read_frames"]),
            resolution=[int(compared["width"]), int(compared["height"])],
            derivation="FFmpeg horizontal stack of the two recorded-state clips"),
        manufacturing_qualified=False,
        scope="Diagnostic model/CAD replay; no physical brake test, continuous clearance or manufactured fit")
    receipt_path = output / "candidate-motion.json"
    receipt_path.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n",
                            encoding="utf-8")
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    result = capture(args.run, args.out)
    print(json.dumps(dict(receipt=str(args.out / "candidate-motion.json"),
                          videos={name: row["sha256"] for name, row in result["videos"].items()},
                          comparison_sha256=result["comparison"]["sha256"]), indent=2))


if __name__ == "__main__":
    main()
