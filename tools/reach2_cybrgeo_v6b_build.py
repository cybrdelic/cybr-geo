"""Build/export/render REACH-2 v6b exclusively through CYBR GEO public APIs."""
from __future__ import annotations
import argparse, importlib.util, json, shutil
from pathlib import Path
from cybrgeo.photoreal import render

ROOT=Path(__file__).resolve().parents[1]
RECIPE=ROOT/'examples'/'reach2_cybrgeo_v6b.py'
ENGINEERING=ROOT/'tools'/'reach2_cybrgeo_v6b_engineering.py'

def load(path,name):
    spec=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(spec);assert spec.loader;spec.loader.exec_module(m);return m

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);ap.add_argument('--spp',type=int,default=256);args=ap.parse_args();out=args.out;out.mkdir(parents=True,exist_ok=True)
    r=load(RECIPE,'reach2_v6b_recipe');e=load(ENGINEERING,'reach2_v6b_eng')
    a=r.build();validation=a.validate();(out/'REACH2_v6_validation.json').write_text(json.dumps(validation,indent=2)+'\n')
    eng=e.qualify();(out/'REACH2_v6_engineering.json').write_text(json.dumps(eng,indent=2)+'\n')
    if not eng['qualified']:raise SystemExit('engineering gate failed')
    native=out/'native_scene';a.save(native)
    shutil.copy2(native/'assembly.step',out/'REACH2_v6_ORBIT_analytic.step')
    a.export_glb(out/'REACH2_v6_ORBIT.glb',poses=r.poses(a,1.6))
    cameras={'hero':(31,18,155,(-15,8,-50)),'output':(-46,15,150,(-20,-15,-58)),'rear':(58,12,150,(-20,38,-58))}
    for name,cam in cameras.items():
        render(a,out/f'REACH2_v6_{name}.png',width=1200,height=900,samples=args.spp,depth=14,camera=cam,threads=4,poses=r.poses(a,1.6))
    manifest=dict(model=a.name,generated_imagery=False,
        geometry_api='cybrgeo.Assembly/cybrgeo.from_shape',tessellator='cybrgeo.core.from_shape',
        orbit_source='examples/orbit_cybrgeo_native_v2.py',legacy_model_runtime_dependency=False,
        render_api='cybrgeo.photoreal.render',renderer='V9',engineering_qualified=True,
        parts=len(a.parts),triangles=sum(len(p.faces) for p in a.parts),analytic_brep_parts=len(a.cad),
        aesthetic_family='CYBR ORBIT',selected_gear=r.GEAR_MODEL,selected_motor=r.MOTOR_MODEL,
        support_bearings=r.SUPPORT_BEARING_MODEL)
    (out/'REACH2_v6_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
if __name__=='__main__':main()
