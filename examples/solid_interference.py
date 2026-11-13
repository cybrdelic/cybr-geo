import numpy as np
from cybrgeo import extrude
from cybrgeo.interference import SolidQuery, sample_motion
angle = np.linspace(0, 2*np.pi, 20, endpoint=False)
ring = np.c_[np.cos(angle), np.sin(angle)]
flange = extrude(2.2*ring, .30, holes=[1.05*ring])
housing = extrude(1.65*ring, 2.4, holes=[1.05*ring], z=.3)
shaft = extrude(.92*ring, 3.3, z=-.15)
key = extrude(.12*ring + [1.35, 0], .7, z=1)
wall = SolidQuery(housing, assume_valid_solid=True)
rotor = SolidQuery(shaft, assume_valid_solid=True)
insert = SolidQuery(key, assume_valid_solid=True)
running_gap = wall.interference(rotor, clearance=.10)
nested_insert = wall.interference(insert)
assert nested_insert.state == 'contained_overlap'
intentional_insert = wall.interference(insert, allow_reason='cast-in insert')
chords = wall.thickness([80, 88, 96])
poses = np.linspace(-2.7, 2.7, 13)
trajectory = lambda x: SolidQuery(extrude(.92*ring + [x, 0], 3.3, z=-.15), assume_valid_solid=True)
motion = sample_motion(wall, trajectory, poses, clearance=.10)
assert not running_gap.violation
assert not intentional_insert.violation
assert any(report.violation for time, report in motion)
