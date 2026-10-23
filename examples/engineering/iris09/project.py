"""IRIS-derived linkage client of the shared engineering architecture."""
import math

from mechanism_lab.core import Material
from mechanism_lab.engineering_source import EngineeringProject
from mechanism_lab.physical_model import digest
from mechanism_lab.planar_native_envelope import PlanarBody, PlanarNativePath

from .mechanism import build, compile_model, linkage, reference, requirements, specification


def planar_path(spec, source_files):
    """Nine radial copies of one constrained leaf/rod/rotor construction."""
    closed = linkage(spec)
    bodies = dict(fixed=PlanarBody('ground'), rotor=PlanarBody('rocker', closed))
    count = spec['source_geometry']['leaf']['count']
    for index in range(count):
        azimuth = index * math.tau / count
        bodies[f'leaf_{index:02}'] = PlanarBody('crank', closed,
            world_rotation_rad=-azimuth,
            yaw_offset_rad=-spec['leaf_crank_angle_rad'])
        bodies[f'rod_{index:02}'] = PlanarBody('coupler', closed,
            world_rotation_rad=-azimuth, yaw_offset_rad=math.pi,
            anchor='rocker_pin_mm', z_mm=spec['rod_height_mm'])
    return PlanarNativePath(bodies, -spec['hinge_range_rad'][0],
        -spec['hinge_range_rad'][1],
        source_identity=digest(dict(project_source_files=source_files, specification=spec)))


def replay(directory, result):
    from mechanism_lab.native_replay import build_assembly
    spec = result['task_parameters']['specification']
    bodies = result['task_parameters']['body_ids']
    materials = dict(aluminum=Material('assumed aluminum', (.30,.34,.39), .7, .3),
        steel=Material('assumed steel', (.20,.24,.29), .8, .27),
        brass=Material('assumed brass', (.64,.42,.15), .8, .27),
        glass=Material('optical reference', (.25,.45,.55), 0., .1, opacity=.25))
    return build_assembly(directory, result, specification=spec,
        body_mapping={name:name for name in bodies}, mass_mapping={name:name for name in bodies if name!='fixed'},
        materials=materials, name='iris09', metadata=dict(
            qualification='New derived single-actuator linkage; ideal pin dynamics and unqualified foil/motor assumptions'))


def create():
    spec = specification()
    return EngineeringProject('iris09', spec, build, compile_model, reference, replay, requirements,
                              duration_s=spec['period_s'], planar_path=planar_path)
