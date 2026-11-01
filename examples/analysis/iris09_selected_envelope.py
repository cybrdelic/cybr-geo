"""Scoped ideal-path native STEP clearance for the nine-leaf IRIS client.

The one planar mechanism service supplies all body poses and rigorous point
speeds. This source-specific client selects six high-value noncontact pairs;
all other moving interfaces remain unexamined, including intentional mates.
"""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path

from mechanism_lab.engineering_source import (load_project, verify_native_build,
    verify_native_source_binding)
from mechanism_lab.native_pose_envelope import (certify_parametric_path,
    load_placed_native_parts)
from mechanism_lab.physical_model import digest


PAIRS = (
    ('rod_00__bored_connecting_rod', 'fixed__housing_material_2'),
    ('leaf_00__leaf_shaft_crank', 'leaf_01__leaf_shaft_crank'),
    ('leaf_00__leaf_shaft_crank', 'leaf_08__leaf_shaft_crank'),
    ('rod_00__bored_connecting_rod', 'rod_01__bored_connecting_rod'),
    ('rod_00__bored_connecting_rod', 'leaf_01__leaf_shaft_crank'),
    ('leaf_00__output_pin_keeper', 'fixed__housing_material_2'),
)


def certify(native_directory):
    native = Path(native_directory).resolve()
    project, source_files = load_project('iris09')
    receipt = verify_native_build(native, project.specification,
                                  source_files=source_files)
    binding = verify_native_source_binding(native, project.specification,
                                           source_files, receipt=receipt)
    path = project.planar_path(project.specification, source_files)
    selected = {name for pair in PAIRS for name in pair}
    parts, imported = load_placed_native_parts(native,
        expected_file_hashes=receipt['files_sha256'], selected_names=selected)
    selected_bodies = {row['body'] for row in parts.values()}
    radii = imported['body_local_point_radius_mm']

    def pose_at(u):
        complete = path.pose_at(u)
        return {name: complete[name] for name in selected_bodies}

    def point_speed_bound(part_name, lo, hi):
        return path.point_speed_bound(parts[part_name]['body'], radii[part_name], lo, hi)

    client_sha = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    report = certify_parametric_path(parts, pose_at, point_speed_bound,
        source_identity=f'IRIS planar path {path.source_sha256}; client {client_sha}; native {digest(receipt)}',
        clearance_pairs=PAIRS, required_clearance_mm=0., distance_margin_mm=1e-4,
        max_depth=12, max_pair_intervals=4095)
    records = json.loads((native / 'placed-native-parts.json').read_text())['records']
    counts = Counter(row['body'] for row in records)
    cross_body_total = sum(a*b for index, a in enumerate(counts.values())
                           for b in list(counts.values())[index+1:])
    report.update(native_step_import=imported, client_source_sha256=client_sha,
        native_build_sha256=digest(receipt),
        native_source_binding_sha256=digest(binding),
        project_source_files_sha256=digest(source_files),
        full_native_cross_body_pair_count=cross_body_total,
        selected_full_assembly_pair_count=len(PAIRS),
        unexamined_full_assembly_pair_count=cross_body_total-len(PAIRS),
        omitted_native_part_count=len(records)-len(parts),
        full_native_inventory_selected=imported['complete_native_inventory_selected'],
        full_native_noncontact_pair_coverage=False,
        scope=('Six named noncontact pairs over the source-bound 0 to -0.9 rad ideal '
               'annular input path. Other native pairs, actual MuJoCo departures, '
               'part variation, intentional bearing/pin mates and hardware remain unqualified.'))
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--native', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    result = certify(args.native)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    print(json.dumps(dict(passed=result['passed'],
        failures=result['failures'], selected_pairs=len(result['clearance_pairs']),
        unexamined_full_assembly_pair_count=result['unexamined_full_assembly_pair_count'])))
    if not result['passed']:
        raise SystemExit(1)
