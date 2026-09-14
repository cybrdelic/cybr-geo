"""Build AERIS once, validate it, and retain analytic CAD for all outputs."""
from pathlib import Path
import sys, json, pickle, time, argparse
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src'))
sys.path.insert(0,str(Path(__file__).resolve().parent))
from dataclasses import replace
from recipe import build, cutaway
from mechanism_lab.core import save_cache,validate
from mechanism_lab.exporters import export_glb,export_step,export_bom,export_animated_glb

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,default=ROOT/'build/aeris');ap.add_argument('--exports',action='store_true');args=ap.parse_args()
    out=args.out;out.mkdir(parents=True,exist_ok=True)
    a=build();save_cache(a,out/'cache')
    with (out/'assembly.pkl').open('wb') as f:pickle.dump(replace(a,motion_function=None),f,pickle.HIGHEST_PROTOCOL)
    report=validate(a)
    report['feature_map']=a.metadata['feature_map'];report['limits']=a.metadata['limitations']
    report['analytic_parts']=sum(p.cad is not None for p in a.parts)
    report['mesh_only_parts']=[p.name for p in a.parts if p.cad is None]
    (out/'validation.json').write_text(json.dumps(report,indent=2))
    export_glb(a,out/'aeris.glb');export_bom(a,out)
    print('ASSEMBLY_CACHE_AND_GLB_READY',flush=True)
    section=cutaway(a);save_cache(section,out/'cutaway_cache')
    export_glb(section,out/'aeris_cutaway.glb')
    with (out/'cutaway.pkl').open('wb') as f:pickle.dump(replace(section,motion_function=None),f,pickle.HIGHEST_PROTOCOL)
    print('BOOLEAN_CUTAWAY_READY',flush=True)
    if args.exports:
        export_step(a,out/'aeris_analytic.step',individual=False)
        export_animated_glb(a,out/'aeris_rotor_motion.glb',duration=4.,fps=24)
        export_animated_glb(a,out/'aeris_service_explosion.glb',duration=6.,fps=24,mode='explode')
        print('STEP_AND_ANIMATED_GLB_READY',flush=True)
    print('BUILD_COMPLETE',flush=True)

if __name__=='__main__':main()
