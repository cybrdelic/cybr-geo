"""Engineering revision of AERIS with a real nominal bearing interface.

This wraps the original showcase recipe instead of editing it in-place so the
visual/reference revision stays reproducible. The engineering assembly keeps
all original part names/counts but replaces the two ad-hoc bearing envelopes and
their seats with nominal 6000-series bearing geometry (10 x 26 x 8 mm).

Nominal CAD is not a tolerance assignment. The production shaft/seat tolerance
classes, bearing manufacturer, internal clearance and preload remain release
inputs and are deliberately not invented here.
"""
from __future__ import annotations
from dataclasses import replace
import numpy as np
from recipe import build as build_reference, cutaway as reference_cutaway, ring, drill, bolt_circle
from mechanism_lab.core import cad_part, Assembly

BEARING_SERIES = "6000"
BEARING_BORE_MM = 10.0
BEARING_OD_MM = 26.0
BEARING_WIDTH_MM = 8.0
BEARING_PITCH_RADIUS_MM = 9.35
BEARING_BALL_RADIUS_MM = 1.75
BEARING_X = (49.5, 100.0)


def _replace_like(old, shape, *, role=None, tags=None):
    return cad_part(old.name, shape, old.material, tolerance=.055, angular=.105,
        group=old.group, explode=np.asarray(old.explode, float), role=role or old.role,
        provenance=old.provenance, motion=old.motion,
        tags=old.tags if tags is None else tuple(tags), finish_axis=old.finish_axis,
        finish_origin=old.finish_origin)


def build_engineered() -> Assembly:
    a = build_reference(); original = {p.name: p for p in a.parts}; replacements = {}
    front = drill(ring(35.0, 13.05, 49.0, 58.0), bolt_circle(29.0, 4), 2.2, 48.0, 59.0)
    replacements['Motor_front_mount_plate'] = _replace_like(
        original['Motor_front_mount_plate'], front,
        role='Front motor/bearing carrier; nominal 6000 bearing seat, 26 mm OD',
        tags=original['Motor_front_mount_plate'].tags + ('bearing-6000-seat',))
    rear = ring(35.0, 0.0, 102.0, 110.0).cut(ring(13.05, 0.0, 101.0, 111.0))
    from recipe import cyl
    for y, z in bolt_circle(29.0, 4): rear = rear.cut(cyl(2.2, 10.0, (101.0, y, z)))
    replacements['Motor_rear_bearing_endbell'] = _replace_like(
        original['Motor_rear_bearing_endbell'], rear,
        role='Rear motor/bearing carrier; nominal 6000 bearing seat, 26 mm OD',
        tags=original['Motor_rear_bearing_endbell'].tags + ('bearing-6000-seat',))
    import cadquery as cq
    for x in BEARING_X:
        width = BEARING_WIDTH_MM
        outer = ring(BEARING_OD_MM/2, 10.9, x, x+width)
        inner = ring(7.7, BEARING_BORE_MM/2, x, x+width)
        replacements[f'Shaft_bearing_outer_{x:g}'] = _replace_like(original[f'Shaft_bearing_outer_{x:g}'], outer,
            role=f'{BEARING_SERIES}-series outer ring nominal envelope', tags=original[f'Shaft_bearing_outer_{x:g}'].tags + ('bearing-6000',))
        replacements[f'Shaft_bearing_inner_{x:g}'] = _replace_like(original[f'Shaft_bearing_inner_{x:g}'], inner,
            role=f'{BEARING_SERIES}-series inner ring with nominal 10 mm bore', tags=original[f'Shaft_bearing_inner_{x:g}'].tags + ('bearing-6000',))
        for j, (y, z) in enumerate(bolt_circle(BEARING_PITCH_RADIUS_MM, 10)):
            name=f'Bearing_ball_{x:g}_{j}'; ball=cq.Solid.makeSphere(BEARING_BALL_RADIUS_MM, cq.Vector(x+width/2, y, z))
            replacements[name] = _replace_like(original[name], ball, role=f'{BEARING_SERIES}-series nominal rolling element envelope', tags=original[name].tags + ('bearing-6000',))
        for old_x in (x+.15, x+5.85):
            name=f'Bearing_shield_{x:g}_{old_x:g}'; new_x = x+.20 if abs(old_x-(x+.15)) < .1 else x+width-.65
            shield=ring(10.9, 7.7, new_x, new_x+.45)
            replacements[name] = _replace_like(original[name], shield, role=f'{BEARING_SERIES}-series shield envelope', tags=original[name].tags + ('bearing-6000',))
    parts=[replacements.get(p.name,p) for p in a.parts]; meta=dict(a.metadata)
    meta['engineering_revision']='AERIS-E1'
    meta['bearing_interface']={'series':BEARING_SERIES,'nominal_bore_mm':BEARING_BORE_MM,'nominal_od_mm':BEARING_OD_MM,
        'nominal_width_mm':BEARING_WIDTH_MM,'shaft_nominal_mm':10.0,'production_tolerance_class':'UNASSIGNED',
        'housing_tolerance_class':'UNASSIGNED','internal_clearance':'UNASSIGNED','preload':'UNASSIGNED','manufacturer_part_number':'UNASSIGNED'}
    meta['limitations']=list(meta.get('limitations',[]))+['AERIS-E1 selects nominal 6000-series bearing geometry only; manufacturer, fit classes, internal clearance and preload remain release inputs.']
    return replace(a, name='aeris_e1', parts=parts, metadata=meta)


def cutaway_engineered(assembly: Assembly):
    return replace(reference_cutaway(assembly), name='aeris_e1_cutaway')
