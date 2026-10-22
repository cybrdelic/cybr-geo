"""IRIS-derived linkage client of the shared engineering architecture."""
from mechanism_lab.core import Material
from mechanism_lab.engineering_source import EngineeringProject
from .mechanism import build, compile_model, reference, requirements, specification


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
                              duration_s=spec['period_s'])
