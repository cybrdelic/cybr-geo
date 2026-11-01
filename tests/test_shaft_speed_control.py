import pytest
from mechanism_lab.shaft_speed_control import TorqueLimitedSpeedDrive

def test_torque_is_bounded_not_a_prescribed_motion():
 c=TorqueLimitedSpeedDrive(6,.000833333333,.000833333333)
 assert c.torque(0)==c.torque_limit_Nm
 assert c.torque(6)==0
 assert c.torque(20)==-c.torque_limit_Nm
 with pytest.raises(ValueError):c.torque(float('nan'))
