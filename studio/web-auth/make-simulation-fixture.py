"""Compare browser replay with native simulated_assembly on actual CAD vertices.

Requires the repository CAD environment. An optional exact source-bound native
cache avoids rebuilding CAD; it is still checked by simulated_assembly.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src'))
import numpy as np
from mechanism_lab.constructed import build_from_plan
from mechanism_lab.core import load_cache
from mechanism_lab.dynamics import simulated_assembly


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recipe', type=Path)
    parser.add_argument('simulation', type=Path)
    parser.add_argument('browser_assembly', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--cache', type=Path)
    args = parser.parse_args()
    plan = json.loads(args.recipe.read_text(encoding='utf-8'))
    result = json.loads(args.simulation.read_text(encoding='utf-8'))
    browser = json.loads(args.browser_assembly.read_text(encoding='utf-8'))
    assembly = load_cache(args.cache) if args.cache else build_from_plan(plan)
    identities = {key: plan['construction'][key] for key in ('recipe_sha256', 'mechanism_sha256', 'model_sha256')}
    for key, value in identities.items():
        if result.get(key) != value or browser.get('construction', {}).get(key) != value or assembly.metadata['construction_recipe']['construction'].get(key) != value:
            raise ValueError('Mismatched simulation/CAD/source identity: ' + key)
    solved = simulated_assembly(assembly, result)
    by_name = {part.name: part for part in solved.parts}
    samples = []
    indices = sorted(set(np.linspace(0, len(result['samples']) - 1, 5).round().astype(int)))
    for index in indices:
        time = result['samples'][index]['time_s']
        parts = {}
        for node in browser['nodes']:
            # Match the Float32BufferAttribute used by the actual inspector.
            vertices = np.asarray(browser['geometries'][node['shape']]['positions'], dtype=np.float32).reshape(-1, 3).astype(float)
            vertex_indices = [0, len(vertices) // 2, len(vertices) - 1]
            matrix = solved.pose(by_name[node['name']], time)
            points = np.column_stack((vertices[vertex_indices], np.ones(3)))
            parts[node['name']] = {'matrix_mm': matrix.tolist(), 'vertex_indices': vertex_indices,
                                   'world_vertices_mm': (matrix @ points.T).T[:, :3].tolist()}
        samples.append({'time_s': time, 'simulation_sample_index': int(index), 'parts': parts})
    fixture = {'schema': 'cybrgeo-native-simulated-vertices/1', **identities,
               'simulation_sha256': hashlib.sha256(args.simulation.read_bytes()).hexdigest(),
               'browser_assembly_sha256': hashlib.sha256(args.browser_assembly.read_bytes()).hexdigest(),
               'scope': 'Native simulated_assembly applied to vertices from the actual WASM CAD artifact; mm/Z-up.',
               'samples': samples}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(fixture, indent=2), encoding='utf-8')
    print(f'Wrote {len(samples)} native simulated-state samples across {len(by_name)} parts')


if __name__ == '__main__':
    main()
