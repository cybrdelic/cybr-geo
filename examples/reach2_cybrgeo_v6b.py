"""Authoritative native CYBR GEO REACH-2 v6b release recipe.

Pins the v6 elbow to the exact-B01 native ORBIT v2 port. Both dependencies use
cybrgeo.Assembly/cybrgeo.from_shape and contain no mechanism_lab imports.
"""
from __future__ import annotations
import importlib.util
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'examples'/'reach2_cybrgeo_v6.py'
spec=importlib.util.spec_from_file_location('reach2_native_v6_base',BASE)
_v6=importlib.util.module_from_spec(spec);assert spec.loader is not None;spec.loader.exec_module(_v6)
_v6.ORBIT_RECIPE=ROOT/'examples'/'orbit_cybrgeo_native_v2.py'

for _name in dir(_v6):
    if _name.isupper():globals()[_name]=getattr(_v6,_name)

elbow_angle=_v6.elbow_angle
rotation=_v6.rotation


def build():
    a=_v6.build()
    a.metadata['schema']='cybrgeo.reach2/6b'
    a.metadata['orbit_source']='examples/orbit_cybrgeo_native_v2.py'
    a.metadata['legacy_model_runtime_dependency']=False
    return a


def poses(assembly,t):
    return _v6.poses(assembly,t)

if __name__=='__main__':print(build().validate())
