"""Public entry point for the verified planetary actuator concept.

The underlying mechanism remains the verified model. This wrapper only adds
inspection views that expose the actual planetary gearset instead of allowing the
front carrier/output stack to occlude it in hero and kinematics renders.
"""
from dataclasses import replace

from .planetary_actuator_verified import SPEC, MATERIALS, pose, build as _verified_build


_INTERNAL_HIDE = (
    'front_cover',
    'carrier',
    'carrier_fasteners',
    'output',
    'bearings',
    'seals',
    'fasteners',
)

_GEARSET_ONLY_HIDE = _INTERNAL_HIDE + (
    'housing',
    'motor_interface',
    'connector',
    'encoder',
)


def build():
    assembly = _verified_build()
    views = dict(assembly.views)

    # Keep the rear housing and interfaces for context, but remove only the stack
    # that physically blocks the 18T/27T/72T gearset from the front camera.
    views['gearset_hero'] = replace(
        views['hero'],
        az=216,
        el=15,
        scale=39,
        target=(29, 0, 0),
        focal_length_mm=86,
        camera_distance_mm=238,
        f_stop=8.0,
        focus_distance_mm=238,
        floor=False,
        hide=_INTERNAL_HIDE,
        title='PLANETARY ACTUATOR / EXPOSED 5:1 GEARSET',
        note='Actual 18T sun / 3 x 27T planets / 72T fixed ring; carrier/output removed for inspection only',
    )

    # A tighter view used to judge the involute mesh itself. No housing, carrier,
    # output shaft or electronics can hide the tooth engagement.
    views['gear_macro'] = replace(
        views['gear_macro'],
        az=208,
        el=8,
        scale=22,
        target=(29, 14, 0),
        focal_length_mm=92,
        camera_distance_mm=190,
        f_stop=8.0,
        focus_distance_mm=190,
        hide=_GEARSET_ONLY_HIDE,
        title='INVOLUTE SUN / PLANET / RING MESH',
        note='Actual generated gear geometry; 0.10 mm pitch-circle backlash; carrier removed for optical access',
    )

    # The motion proof must show the gears that are actually moving, not a carrier
    # plate in front of them. Kinematics and transforms are unchanged.
    views['kinematics'] = replace(
        views['kinematics'],
        az=214,
        el=10,
        scale=34,
        target=(29, 0, 0),
        focal_length_mm=82,
        camera_distance_mm=225,
        f_stop=8.0,
        focus_distance_mm=225,
        hide=_GEARSET_ONLY_HIDE,
        title='WORKING 5:1 PLANETARY GEARSET',
        note='Sun input / fixed ring / carrier-frame motion retained mathematically; carrier plates hidden only for inspection',
    )

    return replace(assembly, views=views)


__all__ = ['SPEC', 'MATERIALS', 'pose', 'build']
