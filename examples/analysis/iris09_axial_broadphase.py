"""Screen every IRIS native cross-body pair for invariant axial separation.

This is the fast, conservative first stage of an ideal-path clearance study.
Pairs whose axial intervals overlap are unresolved, not certified clear.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

if __package__:
    from . import iris09_selected_envelope as priority_client
else:
    import iris09_selected_envelope as priority_client
from mechanism_lab.engineering_source import (load_project, verify_native_build,
    verify_native_source_binding)
from mechanism_lab.physical_model import digest


def screen(native_directory, *, required_clearance_mm=0.):
    native = Path(native_directory).resolve()
    project, source_files = load_project('iris09')
    receipt = verify_native_build(native, project.specification,
                                  source_files=source_files)
    binding = verify_native_source_binding(native, project.specification,
                                           source_files, receipt=receipt)
    path = project.planar_path(project.specification, source_files)
    result = path.axial_clearance_broadphase(native, receipt,
                                             required_clearance_mm=required_clearance_mm)
    separated = {tuple(row['parts']) for row in result['axially_separated_pairs']}
    unresolved = {tuple(row['parts']) for row in result['unresolved_pairs']}
    priorities = []
    for pair in priority_client.PAIRS:
        names = tuple(sorted(pair))
        if names not in separated | unresolved:
            raise ValueError(f'Priority native interface pair is missing from full inventory: {names}')
        priorities.append(dict(parts=list(names), status=(
            'axially_separated' if names in separated else 'requires_exact_sweep_or_mate_evidence')))
    result.update(native_build_sha256=digest(receipt),
                  native_source_binding_sha256=digest(binding),
                  project_source_files_sha256=digest(source_files),
                  client_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  priority_pair_source_sha256=hashlib.sha256(
                      Path(priority_client.__file__).read_bytes()).hexdigest(),
                  priority_pairs=priorities)
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--native', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--required-clearance-mm', type=float, default=0.)
    args = parser.parse_args()
    report = screen(args.native, required_clearance_mm=args.required_clearance_mm)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    print(json.dumps(dict(screened=report['screened'],
        cross_body_pairs=report['cross_body_pairs'],
        axially_separated_pair_count=report['axially_separated_pair_count'],
        unresolved_pair_count=report['unresolved_pair_count'],
        full_native_noncontact_pair_coverage=report['full_native_noncontact_pair_coverage'])))
