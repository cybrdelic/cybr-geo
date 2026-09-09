import math
import numpy as np
from mechanism_lab.core import axis_pose
from mechanism_lab.models import drivetrain as drive


def test_belt_pitch_loop_and_differential_rate_identity():
    assert abs(drive.LENGTH-600)<1e-10
    assert drive.DRIVEN_TEETH/drive.DRIVER_TEETH==2.25
    assert abs(18/2.25-8)<1e-12
    assert abs(10.4+5.6-2*8)<1e-12
    assert drive.CENTER>drive.R+drive.r


def test_belt_tangents_and_continuity():
    # Probe every exact straight/arc boundary from both sides, including the seam.
    boundaries=[0,drive.STRAIGHT,drive.STRAIGHT+drive.SMALL_ARC,
                2*drive.STRAIGHT+drive.SMALL_ARC,drive.LENGTH]
    for s in boundaries:
        p0,t0,n0=drive.belt_sample(s-1e-6)
        p1,t1,n1=drive.belt_sample(s+1e-6)
        assert np.linalg.norm(p1-p0)<3e-6
        assert np.linalg.norm(t1-t0)<1e-6
    for s in np.linspace(0,600,301):
        p,t,n=drive.belt_sample(s)
        assert abs(np.linalg.norm(t)-1)<1e-12
        assert abs(np.dot(t,n))<1e-12


def test_motor_rotor_moves_stator_stays_fixed(motor):
    rotor=next(p for p in motor.parts if p.name=='M01_Vented_rotor_bell')
    fixed=next(p for p in motor.parts if p.name=='M10_36_slot_lamination_stack')
    assert np.allclose(motor.pose(rotor,1),axis_pose(math.tau*.14))
    assert np.allclose(motor.pose(fixed,1),np.eye(4))
    assert sum(p.name.startswith('M03_Magnet_') for p in motor.parts)==40
    assert sum(p.name.startswith('M13_Copper_coil_') for p in motor.parts)==36


def test_motor_documented_envelope(motor):
    parts={p.name:p for p in motor.parts}
    bell=parts['M01_Vented_rotor_bell']
    radius=np.linalg.norm(bell.vertices[:,1:],axis=1)
    assert abs(radius.max()-46)<1e-5
    assert abs(bell.bounds[1,0]-46.5)<1e-7
    pin=parts['M08_3mm_locating_pin']
    assert np.allclose(pin.bounds[:,0],[46.5,52.5],atol=1e-7)
    assert abs(np.linalg.norm(pin.vertices[:,1:],axis=1).max()-1.5)<1e-5
    assert abs(parts['M04_Fixed_four_hole_backplate'].bounds[0,0])<1e-7


def test_all_drivetrain_transforms_rigid(drivetrain):
    for t in np.linspace(0,16,33):
        for p in drivetrain.parts:
            T=drivetrain.pose(p,float(t),.25)
            assert np.isfinite(T).all()
            assert np.allclose(T[3],[0,0,0,1])
            assert np.max(abs(T[:3,:3].T@T[:3,:3]-np.eye(3)))<1e-10
            assert abs(np.linalg.det(T[:3,:3])-1)<1e-10


def test_motor_drive_pulley_coaxial_and_same_motion(drivetrain):
    parts={p.name:p for p in drivetrain.parts}
    rotor=parts['Motor_M01_Vented_rotor_bell']
    pulley=parts['Drive_05_32T_motor_pulley']
    for t in [0,.75,2,10]:
        assert np.allclose(drivetrain.pose(rotor,t),drivetrain.pose(pulley,t))
    # Motor front face and adapter nominal X contact planes meet at x=63 mm.
    assert abs(rotor.bounds[1,0]-63)<1e-7
    assert abs(parts['Drive_04_M3_face_adapter'].bounds[0,0]-63)<1e-7


def test_original_reference_vertices_unchanged(root):
    from mechanism_lab.models.differential import build
    a=build('reference')
    data=np.load(root/'assets/differential_v3/geometry/reference_parts.npz')
    assert len(a.parts)==81
    for p in a.parts:
        assert np.array_equal(p.vertices,data[p.name+'__vertices'])
        assert np.array_equal(p.faces,data[p.name+'__faces'])
        assert np.array_equal(p.normals,data[p.name+'__normals'])
