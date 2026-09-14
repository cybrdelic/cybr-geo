"""Build AERIS-E1, retaining exact CAD and normal CYBR GEO export names.

The revision identity lives in assembly metadata and the output directory. Export
basenames intentionally match the established AERIS verification contract so the
same validators exercise both the reference and engineering revisions.
"""
from pathlib import Path
import sys, json, pickle, argparse
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src'))
sys.path.insert(0,str(Path(__file__).resolve().parent))
from dataclasses import replace
from engineering_revision import build_engineered, cutaway_engineered
from mechanism_lab.core import save_cache, validate
from mechanism_lab.exporters import export_glb, export_step, export_bom, export_animated_glb


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--out',type=Path,default=ROOT/'build/aeris-e1')
    ap.add_argument('--exports',action='store_true')
    args=ap.parse_args(); out=args.out; out.mkdir(parents=True,exist_ok=True)
    a=build_engineered(); save_cache(a,out/'cache')
    with (out/'assembly.pkl').open('wb') as f:
        pickle.dump(replace(a,motion_function=None),f,pickle.HIGHEST_PROTOCOL)
    report=validate(a)
    report['feature_map']=a.metadata['feature_map']
    report['limits']=a.metadata['limitations']
    report['analytic_parts']=sum(p.cad is not None for p in a.parts)
    report['mesh_only_parts']=[p.name for p in a.parts if p.cad is None]
    report['engineering_revision']=a.metadata['engineering_revision']
    report['bearing_interface']=a.metadata['bearing_interface']
    (out/'validation.json').write_text(json.dumps(report,indent=2))
    export_glb(a,out/'aeris.glb'); export_bom(a,out)
    print('AERIS_E1_CACHE_AND_GLB_READY',flush=True)
    section=cutaway_engineered(a); save_cache(section,out/'cutaway_cache')
    export_glb(section,out/'aeris_cutaway.glb')
    with (out/'cutaway.pkl').open('wb') as f:
        pickle.dump(replace(section,motion_function=None),f,pickle.HIGHEST_PROTOCOL)
    if args.exports:
        export_step(a,out/'aeris_analytic.step',individual=False)
        export_animated_glb(a,out/'aeris_rotor_motion.glb',duration=4.,fps=24)
        export_animated_glb(a,out/'aeris_service_explosion.glb',duration=6.,fps=24,mode='explode')
    print('AERIS_E1_BUILD_COMPLETE',flush=True)

if __name__=='__main__': main()
