"""ARX scene declarations consumed by GEO's shared native replay service.

All arm and handed-jaw CAD, placement and mass records come from construction.
Only the run's free workpiece and nest primitives are additional scene solids.
Placement follows aabdf8c's coordinate conventions; the derived pinion has
its own CAD body and recorded solver angle. Recorded solver states own motion.
No archived film, commanded FK or cosmetic drivetrain phase is used.
"""
import cadquery as cq
import numpy as np

from mechanism_lab.core import Material, pose_cad
from mechanism_lab.native_replay import build_assembly as replay_native
from mechanism_lab.physical_model import PhysicalModel
from mechanism_lab.physical_replay import Y_UP_TO_Z_UP


def _quaternion_rotation(values):
    q = np.asarray(values, float)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-12:
        raise ValueError('Scene geom requires a finite quaternion')
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def build_assembly(native_directory, simulation_result, *, analytic=True):
    """Bind one construction to one integrated run using the common loader.

    Body-local CAD uses mm and the physical model uses SI Y-up. The shared
    loader verifies source/build/reference identities, exact native mass
    mapping, every STEP placement and all recorded pose bindings.
    """
    parameters = simulation_result['task_parameters']
    physical = PhysicalModel.from_source(simulation_result['physical_source'])
    physical.require_free_body('puck')
    scenes, shapes = [], {}

    def scene(shape, body, name, material):
        scenes.append(dict(body=body, name=name, material=material))
        shapes[body + '__' + name] = shape

    puck = parameters['puck']
    shape = (cq.Workplane('XZ').circle(puck['outer_radius_m']*1000)
             .circle(puck['bore_radius_m']*1000).extrude(puck['height_m']*500, both=True).val())
    scene(shape, 'puck', 'free_workpiece', 'workpiece')
    for name in ('nest_A_base', 'nest_A_seat', 'nest_B_base', 'nest_B_seat'):
        geom = physical.root.find(f"worldbody/geom[@name='{name}']")
        if geom is None or geom.get('type') != 'cylinder' or any(key in geom.attrib for key in ('euler', 'axisangle', 'xyaxes', 'zaxis', 'fromto')):
            raise ValueError(f'{name} needs an explicit worldbody cylinder with quaternion orientation')
        radius, half = [float(x)*1000 for x in geom.get('size').split()]
        shape = cq.Workplane('XY').circle(radius).extrude(half*2).val().translate((0, 0, -half))
        transform = np.eye(4)
        transform[:3, :3] = _quaternion_rotation([float(x) for x in geom.get('quat', '1 0 0 0').split()])
        transform[:3, 3] = [float(x)*1000 for x in geom.get('pos', '0 0 0').split()]
        scene(pose_cad(shape, transform), 'world', name, 'rubber' if name.endswith('base') else 'aluminum')

    materials = {
        'aluminum': Material('Aluminum source appearance', (.66, .70, .75), .65, .32),
        'steel': Material('Steel source appearance', (.26, .29, .33), .75, .30),
        'rubber': Material('Rubber source appearance', (.10, .11, .12), 0, .72),
        'polymer': Material('Polymer source appearance', (.23, .27, .31), 0, .60),
        'workpiece': Material('Free aluminum workpiece', (.72, .63, .38), .5, .36),
    }
    moving = ('turret', 'upper', 'fore', 'roll', 'bend', 'tool')
    mapping = {name: name for name in moving}
    mapping.update(fixed='fixed_hardware', jaw='jaw_left', jaw_opposed='jaw_right', puck='puck', world='world')
    mass_mapping = {name: name for name in moving}
    mass_mapping.update(jaw_left='jaw', jaw_right='jaw_opposed')
    rotating_pinion = 'gripper_pinion' in physical.bodies
    if rotating_pinion:
        mapping['pinion'] = 'gripper_pinion'
        mass_mapping['gripper_pinion'] = 'pinion'
    assembly = replay_native(native_directory, simulation_result,
        specification=parameters['integration'], body_mapping=mapping, mass_mapping=mass_mapping,
        materials=materials, world_rotation=Y_UP_TO_Z_UP, analytic=analytic,
        scene_records=scenes, scene_shapes=shapes, name='ARX belt / integrated native geometry',
        metadata={'visual_limitations': [
            'Four deforming spring coil visuals are omitted; spring forces remain in the physical model.',
            ('Internal input shafts, drive pulleys and belt teeth retain native local phase; '
             'the source-derived gripper pinion hinge follows recorded solver angle.'
             if rotating_pinion else
             'Internal input shafts, drive pulleys, pinion and belt teeth retain native local phase; '
             'separate rotor/belt material states are not recorded.'),
            'Flexible cable harness is omitted; contacts use the separate declared conservative proxy model.',
            'Workbench surface is omitted from the display; its authored physical contacts remain active.',
            'Workpiece bore is native visual geometry; its physical contact primitive remains the declared cylinder.',
            'Display finishes are source appearance, not a measured material or manufacturing qualification.',
        ]})
    assembly.views['hero'].floor = False
    return assembly
