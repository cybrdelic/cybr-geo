"""P6B's six actual test articles as clients of shared native construction.

Usage: python examples/engineering/arx_compact/print_first.py --output OUTPUT
The source default spec is editable. Fixed BREP joint ends remain tied to their
verified source dimensions; resizing an entire arm is deliberately unsupported.
No output or CAD work is performed on import.
"""
from __future__ import annotations

import argparse
import hashlib
from io import BytesIO
import json
import math
from pathlib import Path

from mechanism_lab.manufacturing_gears import InvoluteRackPair
from mechanism_lab.manufacturing_joints import CaptiveNutPattern, RoundedBoxTube, SleevedCrossBore, SplitLapSplice, SOURCE, native_bounds
from mechanism_lab.native_construction import NativeConstruction, read_specification
from mechanism_lab import manufacturing_gears, manufacturing_joints, native_construction


DATA = Path(__file__).resolve().parent/'data'
DEFAULT_SPEC = DATA/'print-first-spec.json'
INPUT_SHA256 = {
    'print-first-upper.brep': 'd1bb7d6de4bc26f44bd18e6872facb034005f44877426089950dc44b552e59d6',
    'print-first-fore.brep': '8fcb58eef0407a46ab233fed71248d238e98a50c13c15091bcfd56e1e5e8c738',
}
STATUS = 'Source-derived geometry test articles; no printing, whole-arm, reducer or load qualification'
LIMITATIONS = [
    'Four structural articles and separate pinion/rack trials; not an assembled printable robot.',
    'Four M4 bolt and OD6/ID4.5 x 60 mm metal sleeve stacks are proposed, not modeled or qualified.',
    'Sixteen M2.5 captive nuts and cover screws require fit, pullout and stack testing; no hardware torque released.',
    'Source elbow relief changes radius 36.5 to 36.65 mm over z18.8..22.2 mm; the interface remains unqualified.',
    'PETG mass/inertia use the declared solid-continuum density (default 1250 kg/m3); infill, anisotropy, creep and thermal effects are absent.',
    'No whole-arm collision/dynamics, FEA, stiffness, fatigue, transmission strength or payload rating follows from these solids.',
    'Print bounds are a planning envelope only; supports, slicing, shrinkage, actual tolerances and printing are unverified.',
    'The coarse gear pair requires redesigned surrounding interfaces and controller scaling; it is not a drop-in gripper.',
]

CONSTRUCTION_CODE_PATHS = {
    'examples/engineering/arx_compact/print_first.py': Path(__file__).resolve(),
    'src/mechanism_lab/manufacturing_joints.py': Path(manufacturing_joints.__file__).resolve(),
    'src/mechanism_lab/manufacturing_gears.py': Path(manufacturing_gears.__file__).resolve(),
    'src/mechanism_lab/native_construction.py': Path(native_construction.__file__).resolve(),
}


def construction_code_hashes():
    return {name:hashlib.sha256(path.read_bytes()).hexdigest() for name, path in CONSTRUCTION_CODE_PATHS.items()}


_LOADED_CODE_HASHES = construction_code_hashes()


def require_current_construction(expected):
    if construction_code_hashes() != expected:
        raise ValueError('Construction code changed; reload the client and rebuild before accepting output')


def specification(value=DEFAULT_SPEC):
    document, identity = read_specification(value)
    if document.get('schema') != 'cybrgeo-print-first-test-articles/1':
        raise ValueError('Unsupported print-first specification schema')
    expected = {'schema', 'material', 'upper_span', 'fore_span', 'splice', 'upper_pockets',
                'fore_pockets', 'cross_bore', 'gear_pair', 'planning_cube_mm', 'scope'}
    if set(document) != expected:
        raise ValueError('Print-first specification has missing or unknown fields')
    upper, fore = (RoundedBoxTube(**document[key]) for key in ('upper_span', 'fore_span'))
    # These fixed BREP boundaries cannot be resized by changing a span parameter.
    for span, dimensions in ((upper, (60, 44, 168, 148)), (fore, (60, 60, 62, 95))):
        if (span.width_mm, span.height_mm, span.length_mm, span.center_y_mm) != dimensions:
            raise ValueError('Verified source joint ends require their original span dimensions and frame')
        if span.wall_mm < 2.5 or span.outer_radius_mm != 0:
            raise ValueError('Source reinforcement must retain the original wall and outer interface')
    cross_bore = SleevedCrossBore(**document['cross_bore'])
    cross_bore.validate_for(upper)
    splice = SplitLapSplice(beam=upper, hole_radius_mm=cross_bore.hole_radius_mm, **document['splice'])
    upper_pockets, fore_pockets = (CaptiveNutPattern(**document[key]) for key in ('upper_pockets', 'fore_pockets'))
    upper_pockets.validate_for(upper)
    fore_pockets.validate_for(fore)
    gears = InvoluteRackPair(**document['gear_pair'])
    cube = document['planning_cube_mm']
    density = document['material'].get('density_kg_m3')
    for value, label in ((cube, 'planning cube'), (density, 'material density')):
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f'Positive finite {label} required')
    if document['material'].get('name') != 'PETG':
        raise ValueError('This source-derived client requires an explicit PETG material assumption')
    if not isinstance(document['material'].get('assumption'), str) or not document['material']['assumption'].strip():
        raise ValueError('Material assumption must be a nonempty description')
    return document, identity, upper, fore, splice, upper_pockets, fore_pockets, gears


def verified_input_snapshot(data=DATA):
    data = Path(data)
    manifest = json.loads((data/'print-first-inputs.json').read_text(encoding='utf-8-sig'))
    if manifest.get('commit') != SOURCE['commit']:
        raise ValueError('Input source commit differs from the pinned constructive source')
    if {row['file']:row['sha256'] for row in manifest['inputs']} != INPUT_SHA256:
        raise ValueError('Input manifest differs from the two verified Refined BREP sources')
    payloads = {name:(data/name).read_bytes() for name in INPUT_SHA256}
    for filename, expected in INPUT_SHA256.items():
        if hashlib.sha256(payloads[filename]).hexdigest() != expected:
            raise ValueError('Source BREP hash mismatch: '+filename)
    return manifest, payloads


def verified_inputs(data=DATA):
    """Return source identities after verifying one immutable bytes snapshot."""
    return verified_input_snapshot(data)[0]


def structural_shapes(value=DEFAULT_SPEC, *, data=DATA):
    """Retain verified joint ends and construct all coupled reinforcement mates."""
    _, payloads = verified_input_snapshot(data)
    return _structural_shapes_from_snapshot(value, payloads)


def _structural_shapes_from_snapshot(value, payloads):
    import cadquery as cq
    _, _, upper, fore, splice, upper_pockets, fore_pockets, _ = specification(value)
    source_upper = cq.Shape.importBrep(BytesIO(payloads['print-first-upper.brep']))
    source_fore = cq.Shape.importBrep(BytesIO(payloads['print-first-fore.brep']))
    reinforced = source_upper.fuse(upper.native_shape()).clean()
    reinforced = upper_pockets.cut(reinforced, upper)
    # Exact upstream topology repair, declared as an unqualified fit change.
    relief = cq.Solid.makeCylinder(36.65, 3.4, cq.Vector(0, 280, 18.8), cq.Vector(0, 0, 1))
    pieces = splice.native_shapes(reinforced.cut(relief).clean())
    fore_shape = fore_pockets.cut(source_fore.fuse(fore.native_shape()).clean(), fore)
    return {
        'upper-frame-proximal': pieces['proximal'],
        'upper-frame-distal': pieces['distal'],
        'upper-frame-lap-coupler': pieces['coupler'],
        'fore-frame-reinforced': fore_shape,
    }, splice.interface_requirements(pieces)


def build(output, value=DEFAULT_SPEC, *, data=DATA):
    """Export shared native CAD, density integrals and a bounded print plan."""
    import cadquery as cq
    code_hashes = dict(_LOADED_CODE_HASHES)
    require_current_construction(code_hashes)
    document, identity, _, _, splice, _, _, gears = specification(value)
    source_inputs, payloads = verified_input_snapshot(data)
    shapes, requirements = _structural_shapes_from_snapshot(document, payloads)
    gear_shapes = gears.native_shapes()
    # Stable identifiers describe roles; changed spec values are in the receipt.
    shapes.update({'gripper-pinion-tooth-trial':gear_shapes['pinion'],
                   'gripper-rack-tooth-trial':gear_shapes['rack']})
    for name, shape in shapes.items():
        bounds = native_bounds(shape)
        # Each source print frame starts X/Y at +5 mm and Z at zero. Check
        # occupied plate coordinates, not only the untranslated part size.
        if max(bounds.xlen+5, bounds.ylen+5, bounds.zlen) > document['planning_cube_mm']+1e-6:
            raise ValueError('Planning print envelope exceeded including plate placement margin: '+name)
    provenance = dict(source=SOURCE, source_inputs=source_inputs, **identity,
                      construction_code_sha256=code_hashes,
                      reusable_construction='manufacturing_joints + manufacturing_gears + NativeConstruction',
                      material_assumption=document['material'], status=STATUS)
    builder = NativeConstruction(output, {'PETG':document['material']['density_kg_m3']}, provenance=provenance)
    print_dir = builder.output/'print-local'
    print_dir.mkdir(exist_ok=True)
    records, print_hashes = [], {}
    for name, shape in shapes.items():
        builder.addpart(name, shape, 'PETG', STATUS)
        # The three mating source components share the same parent coordinates.
        body = 'upper-test-assembly' if name.startswith('upper-frame-') else name+'-article'
        builder.place(body, name, shape, 'PETG', note='Native volume integral; '+document['material']['assumption'])
        step = cq.importers.importStep(str(builder.cad_directory/(name+'.step'))).val()
        if not step.isValid() or len(step.Solids()) != 1 or abs(step.Volume()/shape.Volume()-1) > 1e-8:
            raise ValueError('STEP readback failed: '+name)
        bb = native_bounds(shape)
        translation = [-bb.xmin+5, -bb.ymin+5, -bb.zmin]
        print_shape = shape.translate(tuple(translation))
        for suffix in ('step', 'stl'):
            path = print_dir/(name+'.'+suffix)
            cq.exporters.export(print_shape, str(path), **({'tolerance':.025, 'angularTolerance':.08} if suffix == 'stl' else {}))
            print_hashes[path.relative_to(builder.output).as_posix()] = hashlib.sha256(path.read_bytes()).hexdigest()
        records.append(dict(name=name, valid_single_solid=True, step_roundtrip_valid=True,
                            step_volume_relative_error=abs(step.Volume()/shape.Volume()-1),
                            source_bounds_mm=[[bb.xmin, bb.ymin, bb.zmin], [bb.xmax, bb.ymax, bb.zmax]],
                            print_size_mm=[bb.xlen, bb.ylen, bb.zlen], print_translation_mm=translation,
                            print_occupied_bounds_mm=[[5, 5, 0], [bb.xlen+5, bb.ylen+5, bb.zlen]]))
    report = dict(schema='cybrgeo-print-first-plan/1', status=STATUS, parts=records,
                  material_assumption=document['material'], splice=splice.to_source(),
                  cross_bore=SleevedCrossBore(**document['cross_bore']).to_source(),
                  gear_pair=gears.to_source(), source_module_scaling=gears.scaling_from(module_mm=.3, teeth=20),
                  planning_cube_mm=document['planning_cube_mm'],
                  planning_bounds_kernel_allowance_mm=1e-6,
                  print_orientation='Parent Z normal to plate; long beam direction in XY; not sliced or qualified',
                  print_files_sha256=print_hashes, limitations=LIMITATIONS)
    require_current_construction(code_hashes)
    result = builder.finish(STATUS, {'print-first-plan.json':report, 'input-spec.json':document},
                            native_requirements=requirements)
    try:
        require_current_construction(code_hashes)
    except ValueError:
        # A source edit during export cannot leave a success receipt behind.
        (builder.output/'native-build.json').unlink(missing_ok=True)
        raise
    return result


def render_preview(output, *, size=(1280, 960)):
    """Render verified native test articles; layout translations are display only."""
    import cadquery as cq
    from PIL import Image
    from PIL.PngImagePlugin import PngInfo
    from mechanism_lab.core import Assembly, Material, View, cad_part
    from mechanism_lab.render import render_still

    output = Path(output).resolve()
    receipt_path = output/'native-build.json'
    receipt = json.loads(receipt_path.read_text(encoding='utf-8'))
    require_current_construction(receipt['provenance']['construction_code_sha256'])
    for relative, expected in receipt['files_sha256'].items():
        path = (output/relative).resolve()
        path.relative_to(output)
        if hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise ValueError('Native preview receipt mismatch: '+relative)
    plan = json.loads((output/'print-first-plan.json').read_text(encoding='utf-8'))
    offsets = {
        'upper-frame-proximal': (0, 40, 0),
        'upper-frame-distal': (95, 40, 0),
        'fore-frame-reinforced': (210, 40, 0),
        'upper-frame-lap-coupler': (310, 90, 0),
        'gripper-pinion-tooth-trial': (130, 0, 0),
        'gripper-rack-tooth-trial': (200, 5, 0),
    }
    if {row['name'] for row in plan['parts']} != set(offsets):
        raise ValueError('Preview requires the six declared source test-article roles')
    parts = []
    for name, offset in offsets.items():
        relative = 'print-local/'+name+'.step'
        path = output/relative
        if hashlib.sha256(path.read_bytes()).hexdigest() != plan['print_files_sha256'][relative]:
            raise ValueError('Print-local geometry hash mismatch: '+name)
        shape = cq.importers.importStep(str(path)).val().translate(offset)
        parts.append(cad_part(name, shape, material=0 if 'frame' in name else 1,
            role='Source-derived test article', provenance='designed-concept', tolerance=.05, angular=.10))
    metadata = dict(source_commit=receipt['provenance']['source']['commit'],
        source_input_sha256={row['file']:row['sha256'] for row in receipt['provenance']['source_inputs']['inputs']},
        native_build_sha256=hashlib.sha256(receipt_path.read_bytes()).hexdigest(),
        input_spec_sha256=receipt['provenance']['input_spec_sha256'],
        print_plan_sha256=hashlib.sha256((output/'print-first-plan.json').read_bytes()).hexdigest(),
        preview_recipe_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        construction_code_sha256=receipt['provenance']['construction_code_sha256'],
        print_step_sha256={name:plan['print_files_sha256']['print-local/'+name+'.step'] for name in offsets},
        display_translation_mm=offsets, geometry_scale=1,
        scope='Six actual CAD test articles; display placement only; not an assembled robot, successful print, simulation or hardware qualification',
        material_display='Opaque inspection colors; no optical or printed-finish inference')
    assembly = Assembly('arx_p6b_shared_native_test_articles', parts,
        [Material('PETG geometry inspection', (.64, .73, .79), rough=.48),
         Material('PETG tooth inspection', (.91, .49, .18), rough=.48)], metadata=metadata)
    low, high = assembly.bounds
    assembly.views['hero'] = View(az=-70, el=60, scale=float(max(high-low)*.52),
        target=tuple((low+high)/2), projection='orthographic',
        title='ARX P6B / six source-derived test articles',
        note='Native CAD at one scale. Hardware is proposed, not modeled. No print or load qualification.')
    image_path = render_still(assembly, output/'print-first-preview.png', size=size, intent='concept')
    # Store provenance in the PNG without altering the rendered pixels.
    with Image.open(image_path) as opened:
        pixels = opened.copy()
    png_metadata = PngInfo()
    png_metadata.add_text('cybrgeo_evidence', json.dumps(metadata, sort_keys=True))
    pixels.save(image_path, pnginfo=png_metadata)
    evidence = dict(metadata, image_sha256=hashlib.sha256(image_path.read_bytes()).hexdigest(),
                    image_file=image_path.name, renderer_metadata=image_path.with_suffix('.render.json').name)
    (output/'print-first-preview-evidence.json').write_text(json.dumps(evidence, indent=2)+'\n', encoding='utf-8')
    return evidence


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--spec', type=Path, default=DEFAULT_SPEC)
    parser.add_argument('--render', action='store_true', help='Render the actual six native test articles')
    parser.add_argument('--render-only', action='store_true', help='Verify and render saved native CAD without rebuilding')
    args = parser.parse_args(argv)
    if not args.render_only:
        build(args.output, args.spec)
    if args.render or args.render_only:
        render_preview(args.output)
    print(json.dumps(dict(output=str(args.output.resolve()), parts=6, status=STATUS), indent=2))


if __name__ == '__main__':
    main()
