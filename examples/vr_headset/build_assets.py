"""Build, export and validate VISOR using shared CYBR GEO infrastructure."""
from pathlib import Path
import argparse
import json
import sys
import cadquery as cq
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recipe import build,rounded_box,cylinder_y
from spec import Spec
from verify import verify
from mechanism_lab.core import save_cache
from mechanism_lab.exporters import export_glb,export_step,export_stls,export_bom,export_animated_glb

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--out",type=Path,default=Path("build/vr_headset"))
    args=ap.parse_args()
    out=args.out;out.mkdir(parents=True,exist_ok=True)
    a=build()
    print("VISOR_CAD_BUILT",len(a.parts),sum(len(p.faces) for p in a.parts),flush=True)
    verify(a,Spec(),out)
    save_cache(a,out/"cache")
    export_glb(a,out/"visor_m1.glb")
    export_step(a,out/"visor_m1.step",individual=False)
    export_animated_glb(a,out/"visor_m1_ipd.glb",duration=4,fps=24)
    export_bom(a,out/"bom")
    printable=a.select(names=[p.name for p in a.parts if "print" in p.tags])
    export_stls(printable,out/"print_parts")
    # Coupon contains the actual 34 mm lens bore and M3 clearance variants.
    coupon=rounded_box(68,5,46,(0,0,0),3)
    for x,r in [(-14,17.15),(15,1.50),(22,1.65),(29,1.80)]:
        coupon=coupon.cut(cylinder_y(r,-3,3,x,0))
    cq.exporters.export(coupon,str(out/"print_parts"/"fit_coupon.stl"))
    (out/"design.json").write_text(json.dumps(Spec().report(),indent=2)+"\n")
    print("VISOR_EXPORT_COMPLETE",flush=True)

if __name__=="__main__":
    main()
