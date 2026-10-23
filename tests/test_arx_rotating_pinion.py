"""The revised ARX default gives its native pinion a real rotational state."""
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from examples.engineering.arx_belt.model import attach_rotating_pinion, control_source
from examples.engineering.arx_belt.rack_contract import SOURCE_MODEL, rack_travel_from_source
from mechanism_lab.physical_model import PhysicalModel
from mechanism_lab.qualification import QualificationIntentPlan


DATA = Path('examples/engineering/arx_belt/data')
SPEC = json.loads((DATA / 'spec.json').read_text(encoding='utf-8'))


def _compiled_pair():
    mj = pytest.importorskip('mujoco')
    source = PhysicalModel.from_xml(SOURCE_MODEL.read_text(encoding='utf-8'))
    travel = rack_travel_from_source(SPEC, model_root=source.root)
    derived = PhysicalModel.from_xml(SOURCE_MODEL.read_text(encoding='utf-8'))
    attachment = attach_rotating_pinion(derived, travel)
    # Source archived CAD record is used only to compile a lightweight
    # kinematic fixture. The release path must still use freshly built pinion
    # STEP mass/inertia, not this cached component or the old tool aggregate.
    old = json.loads((DATA / 'mass_properties.json').read_text(encoding='utf-8'))
    component = next(row for row in old['tool']['components']
                     if row['name'].startswith('gripper_pinion_m0p3_z20_'))
    tensor = np.asarray(component['inertia_at_com_kg_m2'])
    com = np.asarray(component['com_m']) - np.array([0., .2015, 0.])
    pinion = derived.bodies['gripper_pinion']
    ET.SubElement(pinion, 'inertial', mass=f"{component['mass_kg']:.15g}",
                  pos=' '.join(f'{value:.15g}' for value in com),
                  fullinertia=' '.join(f'{value:.15g}' for value in
                      (tensor[0, 0], tensor[1, 1], tensor[2, 2],
                       tensor[0, 1], tensor[0, 2], tensor[1, 2])))
    source_model = mj.MjModel.from_xml_string(source.xml())
    derived_model = mj.MjModel.from_xml_string(derived.xml())
    return mj, source_model, derived_model, derived, attachment


def test_derived_pinion_is_real_hinge_with_source_pitch_and_unchanged_home_geometry():
    mj, source, revised, physical, record = _compiled_pair()
    assert record['pitch_radius_m'] == pytest.approx(.003)
    hinge = revised.joint('gripper_pinion').id
    assert revised.jnt_type[hinge] == mj.mjtJoint.mjJNT_HINGE
    assert revised.body('gripper_pinion').id == revised.jnt_bodyid[hinge]
    actuator = revised.actuator('gripper_motor').id
    assert revised.actuator_trntype[actuator] == mj.mjtTrn.mjTRN_JOINT
    assert revised.actuator_trnid[actuator, 0] == hinge
    assert revised.actuator_gear[actuator, 0] == pytest.approx(1.)
    assert revised.neq == source.neq + 1
    assert 'rack_pinion_angle' not in physical.xml()
    assert revised.qpos0[revised.joint('jaw_left').qposadr[0]] == pytest.approx(.038)
    assert revised.qpos0[revised.joint('jaw_right').qposadr[0]] == pytest.approx(.038)
    source_data, revised_data = mj.MjData(source), mj.MjData(revised)
    mj.mj_resetDataKeyframe(source, source_data, source.key('home').id)
    mj.mj_resetDataKeyframe(revised, revised_data, revised.key('home').id)
    mj.mj_forward(source, source_data)
    mj.mj_forward(revised, revised_data)
    for body in ('jaw_left', 'jaw_right'):
        np.testing.assert_allclose(source_data.body(body).xpos,
                                   revised_data.body(body).xpos, atol=1e-12)
        np.testing.assert_allclose(source_data.body(body).xmat,
                                   revised_data.body(body).xmat, atol=1e-12)
    assert revised_data.joint('gripper_pinion').qpos[0] == pytest.approx(0.)


def test_two_source_radius_meshes_cover_full_jaw_travel():
    mj, source, model, physical, _ = _compiled_pair()
    old = mj.MjData(source)
    data = mj.MjData(model)
    mj.mj_resetDataKeyframe(source, old, source.key('home').id)
    mj.mj_resetDataKeyframe(model, data, model.key('home').id)
    for gap in (.010, .019, .038, .040):
        angle = (gap - .038)/.003
        old.joint('jaw_left').qpos[0] = gap
        old.joint('jaw_right').qpos[0] = gap
        data.joint('jaw_left').qpos[0] = gap
        data.joint('jaw_right').qpos[0] = gap
        data.joint('gripper_pinion').qpos[0] = angle
        mj.mj_forward(source, old)
        mj.mj_forward(model, data)
        for name in ('jaw_left', 'jaw_right'):
            np.testing.assert_allclose(old.body(name).xpos, data.body(name).xpos,
                                       atol=1e-12)
        assert model.jnt_range[model.joint('gripper_pinion').id, 0] <= angle + 1e-12
        assert angle <= model.jnt_range[model.joint('gripper_pinion').id, 1] + 1e-12
        for name in ('jaw_left_pinion_mesh', 'jaw_right_pinion_mesh'):
            assert physical.root.find(f".//equality/joint[@name='{name}']") is not None
            row = model.equality(name).id
            assert data.eq_active[row] == 1
            assert min(abs(data.efc_pos[k]) for k in range(data.nefc)
                       if data.efc_type[k] == mj.mjtConstraint.mjCNSTR_EQUALITY
                       and data.efc_id[k] == row) < 1e-10
    # The two CAD rack pitch lines remain respectively 3 mm above/below the
    # same shaft axis; equality constrains kinematics, not flank strength.
    assert math.isclose(model.body('gripper_pinion').pos[1], .2015, abs_tol=1e-12)


def test_revised_control_targets_physical_pinion_and_bounds_solver_coupling():
    p = json.loads((DATA / 'P2-baseline-parameters.json').read_text(encoding='utf-8'))
    p['integration'] = SPEC
    p['model_revision'] = 'P5-derived-rotating-pinion'
    revised = control_source(p)['rack']
    assert revised['pinion_joint'] == 'gripper_pinion'
    assert revised['pinion_body'] == 'gripper_pinion'
    assert revised['pinion_pitch_radius_m'] == pytest.approx(.003)
    assert revised['maximum_coupling_error_m'] == pytest.approx(.0001)
    assert 'not measured tooth backlash' in revised['coupling_budget_scope']
    p['model_revision'] = 'P5'
    assert 'pinion_joint' not in control_source(p)['rack']


def test_diagnostic_intent_tracks_rotating_native_pinion_not_old_tool_member():
    intent = QualificationIntentPlan(json.loads(
        (DATA / 'qualification-intent.json').read_text(encoding='utf-8')))
    parts = intent.document['parts']
    pinion = intent.document['loads']['rack_motor']['part']
    assert pinion == 'pinion__gripper_pinion_m0p3_z20_0'
    assert pinion in parts
    assert 'tool__gripper_pinion_m0p3_z20_12' not in parts
    assert len(parts) == 380
