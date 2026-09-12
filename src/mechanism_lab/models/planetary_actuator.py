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
    # The gear steel is nearly fully metallic, so the inspection studio intentionally
    # provides a brighter indirect field and broad softboxes. This changes lighting,
    # not geometry or material identity.
    views['gearset_hero'] = replace(
        views['hero'],
        az=188,
        el=7,
        scale=38,
        target=(29, 0, 0),
        focal_length_mm=86,
        camera_distance_mm=250,
        f_stop=8.0,
        focus_distance_mm=250,
        environment_strength=.46,
        background_strength=.62,
        light_size=1.90,
        light_intensity=1.82,
        exposure=1.28,
        floor=False,
        hide=_INTERNAL_HIDE,
        title='PLANETARY ACTUATOR / EXPOSED 5:1 GEARSET',
        note='Actual 18T sun / 3 x 27T planets / 72T fixed ring; carrier/output removed for inspection only',
    )

    # A tighter, nearly axial view used to judge the generated involute mesh itself.
    # No housing, carrier, output shaft or electronics can hide tooth engagement.
    views['gear_macro'] = replace(
        views['gear_macro'],
        az=184,
        el=4,
        scale=21,
        target=(29, 14, 0),
        focal_length_mm=92,
        camera_distance_mm=198,
        f_stop=8.0,
        focus_distance_mm=198,
        environment_strength=.42,
        background_strength=.58,
        light_size=1.95,
        light_intensity=1.78,
        exposure=1.26,
        hide=_GEARSET_ONLY_HIDE,
        title='INVOLUTE SUN / PLANET / RING MESH',
        note='Actual generated gear geometry; 0.10 mm pitch-circle backlash; carrier removed for optical access',
    )

    # The motion proof must show the gears that are actually moving, not a carrier
    # plate in front of them. Kinematics and transforms are unchanged.
    views['kinematics'] = replace(
        views['kinematics'],
        az=188,
        el=5,
        scale=33,
        target=(29, 0, 0),
        focal_length_mm=82,
        camera_distance_mm=232,
        f_stop=8.0,
        focus_distance_mm=232,
        environment_strength=.42,
        background_strength=.60,
        light_size=1.85,
        light_intensity=1.72,
        exposure=1.24,
        hide=_GEARSET_ONLY_HIDE,
        title='WORKING 5:1 PLANETARY GEARSET',
        note='Sun input / fixed ring / carrier-frame motion retained mathematically; carrier plates hidden only for inspection',
    )

    return replace(assembly, views=views)


__all__ = ['SPEC', 'MATERIALS', 'pose', 'build']
