"""A gravity-loaded four-bar client of the common GEO engineering protocol."""
from mechanism_lab.core import Material
from mechanism_lab.engineering_source import EngineeringProject
from mechanism_lab.physical_replay import Y_UP_TO_Z_UP
from mechanism_lab.physical_model import digest
from mechanism_lab.planar_native_envelope import PlanarBody, PlanarNativePath

from .mechanism import build, compile_model, linkage, reference, requirements, specification
import math


def planar_path(spec, source_files):
    """Bind the same crank-rocker construction to native motion screening."""
    closed = linkage(spec)
    bodies = dict(fixed=PlanarBody('ground'),
        crank=PlanarBody('crank', closed),
        rocker=PlanarBody('rocker', closed),
        coupler=PlanarBody('coupler', closed, z_mm=spec['coupler_height_mm']))
    return PlanarNativePath(bodies, spec['initial_crank_angle_rad'],
        spec['initial_crank_angle_rad'] + math.tau,
        source_identity=digest(dict(project_source_files=source_files, specification=spec)))


def replay(directory, result):
    from mechanism_lab.native_replay import build_assembly
    bodies = result['task_parameters']['body_ids']
    materials = dict(aluminum=Material('machined aluminum frame', (.25, .29, .34), .72, .28),
        bronze=Material('nominal bronze bushings', (.57, .36, .14), .8, .24),
        steel=Material('ground steel coupler and retainers', (.56, .60, .64), .85, .22),
        steel_blue=Material('steel crank with blue finish', (.055, .23, .43), .72, .25),
        steel_orange=Material('steel loaded rocker with orange finish', (.72, .25, .065), .72, .25))
    return build_assembly(directory, result,
        specification=result['task_parameters']['specification'],
        body_mapping={name: name for name in bodies},
        mass_mapping={name: name for name in bodies if name != 'fixed'},
        materials=materials, world_rotation=Y_UP_TO_Z_UP, name='fourbar_lift',
        metadata=dict(qualification='New gravity-loaded ideal-pin mechanism; CAD-derived mass, assumed motor and nominal manufactured interfaces'))


def create():
    spec = specification()
    return EngineeringProject('fourbar_lift', spec, build, compile_model, reference, replay,
                              requirements, duration_s=spec['period_s'], planar_path=planar_path)
