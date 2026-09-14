"""Release wrapper for the native CYBR GEO ORBIT port.

Keeps the native ORBIT geometry from orbit_cybrgeo_native.py and explicitly
rebuilds B01 from the revision-3 dimensions so the native port preserves the
original mounting shoe exactly. No mechanism_lab imports are used.
"""
from __future__ import annotations
import importlib.util
from pathlib import Path
import numpy as np
import cadquery as cq
from cybrgeo import Assembly, from_shape

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'examples'/'orbit_cybrgeo_native.py'
spec=importlib.util.spec_from_file_location('orbit_native_base',BASE)
_base=importlib.util.module_from_spec(spec);assert spec.loader is not None;spec.loader.exec_module(_base)

for _name in ('AXIS','GEAR_TEETH','PINION_TEETH','MODULE','CENTER_DISTANCE','LEAD','JAW_TRAVEL','JAW_CENTER','PERIOD','MATERIALS','wrist_angle','jaw_displacement','local_pose'):
    globals()[_name]=getattr(_base,_name)


def build()->Assembly:
    a=_base.build()
    base=_base.box(72,136,9,(-22,9,4.5),2)
    for x in (-46,2):
        for y in (-45,63):
            base=base.cut(_base.cylinder(3.3,12,(x,y,-1),'Z'))
            base=base.cut(_base.cylinder(6,4,(x,y,5.5),'Z'))
    for y in (-32,60):
        base=base.cut(_base.cylinder(2.2,11,(-17,y,-1),'Z'))
    etch=cq.Workplane('XY',origin=(-34,65,8.88)).text('ORBIT 01',3.2,.3,combine=False,halign='left',valign='center').val()
    base=base.cut(etch)
    replacement=from_shape('B01_Mounting_shoe',base,material=0,tolerance=.018,angular_tolerance=.055,
        group='base',motion='fixed',role='B01 Mounting shoe',
        metadata={'provenance':'designed-concept','tags':['box','fillet','counterbore','pattern'],'finish_axis':(1,0,0),'finish_origin':(0,0,70)})
    parts=[replacement if p.name=='B01_Mounting_shoe' else p for p in a.parts]
    cad=dict(a.cad);cad['B01_Mounting_shoe']=base
    meta=dict(a.metadata);meta['native_port_revision']=2;meta['mounting_shoe_parity']='revision-3 exact B01 bores/counterbores'
    return Assembly('cybr_orbit_inspection_wrist_native_v2',parts,list(a.materials),meta,cad)


def poses(assembly:Assembly,t:float):
    return {p.name:local_pose(p,t) for p in assembly.parts}

if __name__=='__main__':print(build().validate())
