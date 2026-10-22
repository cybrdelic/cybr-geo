"""A material pin layout must close under one motor and fail when disconnected."""
import math
from dataclasses import replace
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from mechanism_lab.linkages import PinnedLink
from mechanism_lab.physical_model import PhysicalModel
from mechanism_lab.causal_runtime import CausalSimulation


def frame(x=0., angle=0.):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0, x], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])


def link(**kwargs):
    options = dict(width_mm=6., thickness_mm=2., pin_diameter_mm=2.,
                   diametral_clearance_mm=.1, minimum_wall_mm=1.)
    options.update(kwargs)
    return PinnedLink.from_frames('coupler', 'input', 'output', frame(), frame(50., math.pi/3),
                                  [30, 0, 0], [30, 0, 0], **options)


def test_native_strap_bores_and_closure_axes_have_one_geometry():
    rod = link()
    shape = rod.native_shape()
    expected = (rod.width_mm * rod.length_mm + math.pi * (rod.width_mm/2)**2
                - 2 * math.pi * ((rod.pin_diameter_mm + rod.diametral_clearance_mm)/2)**2) * rod.thickness_mm
    assert shape.isValid() and len(shape.Solids()) == 1
    assert shape.Volume() == pytest.approx(expected, rel=1e-10)
    import cadquery as cq
    # Actual clearance bores admit declared pins without material intersection.
    for x in (0, rod.length_mm):
        pin = cq.Workplane('XY').center(x, 0).circle(rod.pin_diameter_mm/2).extrude(4).translate((0, 0, -2)).val()
        assert shape.intersect(pin).Volume() < 1e-9
        assert shape.distance(pin) == pytest.approx(rod.diametral_clearance_mm/2, abs=1e-7)


@pytest.mark.parametrize('kwargs', [dict(width_mm=3.), dict(diametral_clearance_mm=0.),
    dict(pin_diameter_mm=float('nan')), dict(thickness_mm=-1)])
def test_unmanufacturable_declared_pin_interfaces_rejected(kwargs):
    with pytest.raises(ValueError):
        link(**kwargs)


def bare_physical(output_pos='.05 0 0', output_axis='0 0 1'):
    return PhysicalModel.from_xml(f'''<mujoco><compiler angle="radian"/>
      <worldbody>
        <body name="input"><joint name="input_hinge" axis="0 0 1"/></body>
        <body name="output" pos="{output_pos}" quat=".8660254037844386 0 0 .5">
          <joint name="output_hinge" axis="{output_axis}"/>
        </body>
      </worldbody>
    </mujoco>''')


def test_stated_qpos0_frames_must_match_actual_body_poses_before_mutation():
    physical = bare_physical(output_pos='.06 0 0')
    before = physical.to_source()
    with pytest.raises(ValueError, match='qpos0 frame differs'):
        link().attach(physical)
    assert physical.to_source() == before


@pytest.mark.parametrize('axis',['1 0 0','0 1 1'])
def test_socket_pin_axis_cannot_tilt_out_of_planar_bored_strap(axis):
    physical = bare_physical(output_axis=axis)
    before = physical.to_source()
    with pytest.raises(ValueError, match='axes must stay parallel'):
        link().attach(physical)
    assert physical.to_source() == before


@pytest.mark.parametrize('mutation',['free', 'ball', 'default_axis', 'orientation'])
def test_unsupported_ancestor_or_ambiguous_pose_is_rejected_without_mutation(mutation):
    physical = bare_physical()
    output = physical.bodies['output']
    joint = output.find('joint')
    if mutation == 'free':
        output.remove(joint)
        ET.SubElement(output,'freejoint',name='free_output')
    elif mutation == 'ball':
        joint.set('type','ball')
    elif mutation == 'default_axis':
        joint.attrib.pop('axis')
        defaults = ET.SubElement(physical.root,'default')
        ET.SubElement(defaults,'joint',axis='1 0 0')
    else:
        output.attrib.pop('quat')
        output.set('euler','0 0 1.047197551196598')
    before = physical.to_source()
    with pytest.raises(ValueError):
        link().attach(physical)
    assert physical.to_source() == before


def test_ancestor_z_axis_sign_and_explicit_nested_pose_are_supported():
    physical = bare_physical(output_axis='0 0 -1')
    world = physical.root.find('worldbody')
    child = physical.bodies['output']
    world.remove(child)
    parent = ET.SubElement(world,'body',name='carrier',pos='.01 0 0')
    ET.SubElement(parent,'joint',name='carrier_hinge',axis='0 0 -1')
    child.set('pos','.04 0 0')
    parent.append(child)
    rod = link()
    rod.attach(physical)
    recorded = physical.provenance['pinned_links'][0]
    assert np.asarray(recorded['output_frame_mm']) == pytest.approx(frame(50.,math.pi/3))
    assert len(physical.root.findall('equality/connect')) == 1
    assert physical.root.find('actuator') is None


def test_generated_pin_and_closure_are_independent_of_model_defaults():
    import mujoco
    physical = bare_physical()
    defaults = ET.SubElement(physical.root, 'default')
    ET.SubElement(defaults, 'joint', pos='.01 .02 .03', ref='.2', springref='.4',
                  stiffness='17', springdamper='.1 .7', armature='.05',
                  frictionloss='.8', damping='.9', limited='true', range='-2 2',
                  actuatorfrclimited='true', actuatorfrcrange='1 2')
    ET.SubElement(defaults, 'equality', active='false', solref='.1 .3', solimp='.5 .7 .01 .7 5')
    actuators = ET.SubElement(physical.root, 'actuator')
    ET.SubElement(actuators, 'motor', name='default_probe_drive', joint='input_hinge')
    for body in physical.bodies.values():
        ET.SubElement(body, 'inertial', mass='.05', pos='0 0 0',
                      diaginertia='.00001 .00001 .00001')
    rod = link()
    child = rod.attach(physical, damping_Nm_s_rad=.002)
    ET.SubElement(child, 'inertial', mass='.02', pos=f'{rod.length_mm*.0005} 0 0',
                  diaginertia='.0000001 .00001 .00001')
    model = mujoco.MjModel.from_xml_string(physical.xml())
    data = mujoco.MjData(model)
    hinge = model.joint('coupler_hinge')
    dof, qpos = hinge.dofadr[0], hinge.qposadr[0]
    assert model.qpos0[qpos] == 0.
    assert model.qpos_spring[qpos] == 0.
    assert model.jnt_stiffness[hinge.id] == 0.
    assert model.dof_armature[dof] == 0.
    assert model.dof_frictionloss[dof] == 0.
    assert model.dof_damping[dof] == pytest.approx(.002)
    assert not model.jnt_limited[hinge.id]
    closure = model.equality('coupler_closure').id
    assert data.eq_active[closure]
    assert model.eq_solref[closure] == pytest.approx([.002, 1.])
    assert model.eq_solimp[closure] == pytest.approx([.99, .999, .00001, .5, 2.])
    # Turning the generated hinge must keep its bored end on the input pin.
    data.qpos[qpos] += .25
    data.qvel[dof] = 2.
    mujoco.mj_forward(model, data)
    input_id, rod_id = model.body('input').id, model.body('coupler').id
    material_pin = (data.xpos[input_id]
                    + data.xmat[input_id].reshape(3, 3) @ (np.asarray(rod.input_anchor_mm)*.001))
    assert data.xanchor[hinge.id] == pytest.approx(material_pin, abs=1e-12)
    assert data.xpos[rod_id] == pytest.approx(material_pin, abs=1e-12)
    assert data.qfrc_passive[dof] == pytest.approx(-.004)
    assert data.qfrc_actuator[dof] == 0.


@pytest.mark.parametrize('change',[dict(width_mm=.5),dict(length_mm=2.),
                                   dict(input_relative_yaw_rad=0.),dict(thickness_mm=-1.)])
def test_direct_dataclass_reconstruction_cannot_bypass_material_axis_checks(change):
    with pytest.raises(ValueError):
        replace(link(),**change)


def mechanism(disconnected=False):
    physical = PhysicalModel.from_xml('''<mujoco model="single_motor_four_bar">
      <compiler angle="radian"/><option timestep="0.0005" gravity="0 0 0" integrator="implicitfast"/>
      <worldbody>
        <body name="input"><joint name="input_hinge" axis="0 0 1"/>
          <inertial mass=".05" pos=".015 0 0" diaginertia=".000001 .00001 .00001"/>
        </body>
        <body name="output" pos=".05 0 0" quat=".8660254037844386 0 0 .5">
          <joint name="output_hinge" axis="0 0 1" damping=".001"/>
          <inertial mass=".05" pos=".015 0 0" diaginertia=".000001 .00001 .00001"/>
        </body>
      </worldbody>
      <actuator><motor name="drive" joint="input_hinge"/></actuator>
    </mujoco>''')
    rod = link()
    child = rod.attach(physical, damping_Nm_s_rad=.001)
    ET.SubElement(child, 'inertial', mass='.02', pos=f'{rod.length_mm*.0005} 0 0',
                  diaginertia='.0000001 .00001 .00001')
    if disconnected:
        physical.root.find('equality').remove(physical.root.find('equality/connect'))
    motor = dict(gear_ratio=10., gear_efficiency=.9, torque_constant_Nm_A=.02,
                 resistance_ohm=2., inductance_H=.01, bus_voltage_V=24., peak_current_A=1.,
                 gear_efficiency_transition_motor_rad_s=3., thermal_resistance_K_W=10.,
                 thermal_capacity_J_K=20., gear_output_torque_limit_Nm=.1)
    control = dict(drives=[dict(joint='input_hinge', actuator='drive', motor=motor,
                   kp=.4, kd=.015, ki=0., integrator_limit=.1, soft_limits=[-.5, .5])],
                   inertial_feedforward=False, gravity_and_coriolis_feedforward=False)
    return CausalSimulation(physical, control, lambda t: dict(q=[.12], qd=[0.], qdd=[0.]))


def test_real_closed_loop_transmits_force_to_passive_follower():
    connected, disconnected = mechanism(), mechanism(True)
    assert connected.model.nu == 1 and connected.model.nv == 3
    connected.run(.35, output_hz=100)
    disconnected.run(.35, output_hz=100)
    # A free passive follower cannot copy the target once the material link is removed.
    out = connected.model.joint('output_hinge').qposadr[0]
    broken = disconnected.model.joint('output_hinge').qposadr[0]
    assert abs(connected.data.qpos[out]) > .04
    assert abs(disconnected.data.qpos[broken]) < 1e-12
    assert connected.motors[0].state.copper_heat_J > 0
    connected.mj.mj_kinematics(connected.model, connected.data)
    a = connected.data.site_xpos[connected.model.site('coupler_tip').id]
    b = connected.data.site_xpos[connected.model.site('coupler_socket').id]
    assert np.linalg.norm(a-b) < 1e-5
    restored = PhysicalModel.from_source(connected.physical.to_source())
    assert restored.to_source() == connected.physical.to_source()
