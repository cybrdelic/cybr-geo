"""Read preserved baseline geometry without re-tessellating or changing it."""
import json, math
from pathlib import Path
import numpy as np
from ..core import Assembly,Material,Part,View,project_root,axis_pose,rotation_x

RATIO=48/18

def pose(p,t,e):
    c=t*math.tau*.10;d=t*math.tau*.035
    angle=c+d if p.motion=='left' else c-d if p.motion=='right' else c
    T=axis_pose(angle,explode=p.explode,amount=e)
    if p.motion in ['planetA','planetB']:
        q=(-RATIO if p.motion=='planetA' else RATIO)*d
        R=rotation_x(c);Rl=rotation_x(q);center=np.asarray(p.center)
        T[:3,:3]=R@Rl;T[:3,3]=R@(center-Rl@center)+np.asarray(p.explode)*e
    return T

def build(kind='working'):
    root=project_root()/'assets/differential_v3/geometry'
    file={'reference':'reference_parts','core':'kinematic_core_parts','working':'working_variant_parts'}[kind]
    info=json.loads((root/'reference_manifest.json').read_text())
    materials=[Material(m['name'],tuple(m['color']),m['metal'],m['rough']) for m in info['materials']]
    arr=np.load(root/(file+'.npz'));parts=[]
    for meta in json.loads((root/(file+'.json')).read_text()):
        meta.pop('triangles',None)
        meta['provenance']='preserved-v3-geometry';n=meta['name']
        meta.update({k:arr[n+'__'+k].copy() for k in ['vertices','faces','normals']})
        parts.append(Part(**meta))
    arr.close()
    views={
        'hero':View(az=233,el=24,scale=90,target=(0,0,0),title='DIFFERENTIAL / '+kind.upper(),note='Preserved geometry. Working variant is a separate kinematic alternative.'),
        'rear':View(az=46,el=24,scale=90,target=(0,0,0),title='REVERSE INSPECTION'),
        'internal':View(az=234,el=27,scale=65,target=(0,0,0),hide=('carrier','front_flange','rear_flange','front_hub','rear_hub','marking'),title='INTERNAL GEOMETRY',note='Kinematic model, not a loaded-contact or torque-bias simulation.'),
        'section':View(az=259,el=15,scale=88,target=(0,0,0),section=(0,1,0),title='DIFFERENTIAL / LONGITUDINAL SECTION'),
        'exploded':View(az=259,el=23,scale=180,target=(5,0,25),explode=1,title='DIFFERENTIAL / EXPLODED'),
    }
    return Assembly('differential_'+kind,parts,materials,views,metadata=dict(kind=kind,
        warning='Original reference has no connected pinion train. Working/core are the separately labeled v3 alternative. No validated torque or speed rating.',
        source='assets/differential_v3',gear_ratio=RATIO),motion_function=pose if kind!='reference' else None)
