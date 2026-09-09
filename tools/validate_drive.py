from pathlib import Path
import json,math
import numpy as np
from shapely.geometry import Polygon
from shapely.affinity import rotate,translate
from cybrgeo.features import involute_profile
R=Path(__file__).resolve().parents[1]
def poly(n,phase):
 p=involute_profile(n,1.75,.10,12)
 return Polygon(np.c_[p[:,0]*np.cos(p[:,1]+phase),p[:,0]*np.sin(p[:,1]+phase)])
a=poly(80,math.pi/80);b=poly(20,0);maximum=0.;minimum=1e9
for angle in np.linspace(0,2*math.pi/20,241):
 c=rotate(a,angle,origin=(0,0),use_radians=True)
 d=translate(rotate(b,-4*angle,origin=(0,0),use_radians=True),xoff=87.5)
 maximum=max(maximum,c.intersection(d).area);minimum=min(minimum,c.distance(d))
report={'samples':241,'max_sampled_intersection_mm2':maximum,'min_sampled_gap_mm':minimum,
 'motor_mount_clearance_holes_mm':5.2,'nominal_10_screw_major_diameter_mm':4.826,
 'nominal_diametral_mount_clearance_mm':5.2-4.826,
 'pass_no_sampled_profile_interference':maximum<1e-7,
 'scope':'2D sampled section check, excludes key retention, end face contact, backlash under load and force analysis'}
(R/'examples/motor_drive/gear_profile_checks.json').write_text(json.dumps(report,indent=2));print(report)
assert maximum<1e-7
