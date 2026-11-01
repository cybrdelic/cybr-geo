"""Torque-limited proportional shaft-speed control for diagnostic drive models.

This commands force/torque, never joint positions or a gear equality. It assumes
shaft-speed feedback is available. The bounded speed command is not a hard
kinematic constraint: the plant can overshoot. It supplies no physical motor/sensor rating.
"""
from dataclasses import dataclass
import math

@dataclass(frozen=True)
class TorqueLimitedSpeedDrive:
    target_speed_rad_s: float
    torque_limit_Nm: float
    gain_Nm_s_rad: float

    def __post_init__(self):
        if any(not math.isfinite(v) or v<=0 for v in (self.target_speed_rad_s,self.torque_limit_Nm,self.gain_Nm_s_rad)):
            raise ValueError('Finite positive diagnostic drive parameters required')

    def torque(self, shaft_speed_rad_s):
        if not math.isfinite(shaft_speed_rad_s):raise ValueError('Finite shaft-speed feedback required')
        request=self.gain_Nm_s_rad*(self.target_speed_rad_s-shaft_speed_rad_s)
        return max(-self.torque_limit_Nm,min(self.torque_limit_Nm,request))
