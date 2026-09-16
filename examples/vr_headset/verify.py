"""Deterministic CAD, aperture and service checks; never claims a built device."""
from pathlib import Path
import argparse
import hashlib
import json
import sys
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parent))
from spec import Spec
from recipe import build
from optics import optical_report,trace_from_eye
from mechanism_lab.core import validate,pose_cad


def aabb_overlap(a,b):
    aa=a.BoundingBox();bb=b.BoundingBox()
    return all(min(v,w)-max(x,y)>1e-5 for x,v,y,w in [
        (aa.xmin,aa.xmax,bb.xmin,bb.xmax),(aa.ymin,aa.ymax,bb.ymin,bb.ymax),
        (aa.zmin,aa.zmax,bb.zmin,bb.zmax)])


def intersection_volume(a,b):
    if not aabb_overlap(a,b):
        return 0.0
    return float(a.intersect(b).Volume())


def verify(a,s,out):
    out=Path(out);out.mkdir(parents=True,exist_ok=True)
    report=validate(a)
    checks=[]
    def check(name,passed,**evidence):
        checks.append({"name":name,"passed":bool(passed),**evidence})
    def get(name):
        return next(p for p in a.parts if p.name==name)

    check("analytic rigid CAD valid",all(p.cad is not None and p.cad.isValid()
          for p in a.parts if p.group!="straps"))
    check("two refractive optics",len([p for p in a.parts if p.group=="lenses"])==2 and
          all(a.materials[p.material].transmission==1 for p in a.parts if p.group=="lenses"))
    check("finite positive lens edge",s.lens_edge_thickness>1,
          edge_mm=s.lens_edge_thickness)
    lens=get("11_L_lens").cad
    check("solid lens has nonzero volume",lens.Volume()>1000,volume_mm3=lens.Volume())

    # Continuous slot envelope is analytically bounded; CAD sampled at 29 settings.
    bridge=get("03_IPD_bridge").cad
    shell=get("01_optical_tunnel").cad
    rows=[]
    moving=[p for p in a.parts if p.motion=="left_ipd"]
    for ipd in np.linspace(58,72,29):
        delta=-(float(ipd)-s.ipd)/2
        worst=0.;pair=None
        for p in moving:
            shifted=p.cad.translate((delta,0,0))
            for target in (bridge,shell):
                v=intersection_volume(shifted,target)
                if v>worst:
                    worst=v;pair=p.name
        rows.append({"ipd_mm":float(ipd),"max_rigid_interference_mm3":worst,"part":pair})
    check("IPD carriers clear bridge and shell",max(x["max_rigid_interference_mm3"]
          for x in rows)<.02,samples=rows)
    # The two radial barrel envelopes cannot meet anywhere in the travel.
    check("ocular barrels cannot collide",s.ipd_min-46>0,
          minimum_envelope_gap_mm=s.ipd_min-46)
    check("lens radial running clearance",17.3-s.lens_diameter/2>=.29,
          nominal_radial_mm=17.3-s.lens_diameter/2,
          worst_case_at_declared_print_bound_mm=17.3-s.lens_diameter/2-2*s.print_error)

    # Lens vs opaque rigid components at assembled nominal pose.
    optical_collisions=[]
    for p in a.parts:
        if p.group=="lenses":
            for q in a.parts:
                if q.cad is None or q is p or q.material in (2,8,9) or q.group=="straps":
                    continue
                v=intersection_volume(p.cad,q.cad)
                if v>.02:
                    optical_collisions.append([p.name,q.name,v])
    check("rigid solids do not invade lenses",not optical_collisions,
          collisions=optical_collisions)

    # Remove the four screws and cover, then extract the intact phone along +Y.
    phone=[p for p in a.parts if p.group=="phone"]
    targets=[p for p in a.parts if p.name in
             ("01_optical_tunnel","30_phone_cradle","31_screen_bezel_stop","03_IPD_bridge")]
    extraction=[]
    for travel in np.linspace(0,110,23):
        worst=0.;pair=None
        for p in phone:
            shape=p.cad.translate((0,float(travel),0))
            for q in targets:
                volume=intersection_volume(shape,q.cad)
                if volume>worst:
                    worst=volume;pair=[p.name,q.name]
        extraction.append({"travel_y_mm":float(travel),"overlap_mm3":worst,"pair":pair})
    check("phone extraction after cover removal",
          max(row["overlap_mm3"] for row in extraction)<.02,samples=extraction)
    check("phone body planar clearance",165.8-s.phone_width>0 and 80.6-s.phone_height>0,
          total_width_clearance_mm=165.8-s.phone_width,
          total_height_clearance_mm=80.6-s.phone_height)
    check("camera cover axial clearance",57-(s.screen_y+s.phone_thickness+s.camera_allowance)>0,
          camera_allowance_mm=s.camera_allowance,
          gap_mm=57-(s.screen_y+s.phone_thickness+s.camera_allowance))

    optical=optical_report(s)
    center=trace_from_eye(s,0)
    check("chief ray reaches screen center",center is not None and abs(center["screen_x_mm"])<1e-8)
    for angle in (5,10,20,25):
        lo=trace_from_eye(s,-angle);hi=trace_from_eye(s,angle)
        check(f"Snell symmetry at {angle} degrees",lo is not None and hi is not None and
              abs(lo["screen_x_mm"]+hi["screen_x_mm"])<1e-7)
    check("virtual image beyond 0.5 m",s.object_distance<s.lens_efl and
          s.report()["paraxial_virtual_image_distance_mm"]>500)
    # Readable, finite transforms at every motion extreme and intermediate.
    for t in np.linspace(0,4,65):
        for p in a.parts:
            T=a.pose(p,float(t),0)
            if not np.isfinite(T).all():
                raise AssertionError(f"Nonfinite pose {p.name}")
    check("finite sampled animation",True,samples=65)

    result={
        "model":a.name,"passed":all(c["passed"] for c in checks),
        "checks":checks,"geometry":report,
        "physical_headset_tested":False,"phone_app_tested_on_device":False,
        "manufacturing_status":"prototype drawings, not production release",
        "explicit_exclusions":[
            "Compliant foam, straps and optical rim compression are not rigid collision gates.",
            "Thread helices, nut torque and material strength are not simulated.",
            "Print-bound radial fit can reach zero clearance: calibrate with included coupon.",
            "Thermal comfort, drop retention, eye box, MTF, distortion and sensor latency require hardware.",
            "Nominal anthropometric cushion, camera and display envelopes require measurements."
        ]
    }
    (out/"verification.json").write_text(json.dumps(result,indent=2)+"\n")
    (out/"optics.json").write_text(json.dumps(optical,indent=2)+"\n")
    for c in checks:
        print(("PASS" if c["passed"] else "FAIL"),c["name"],flush=True)
    if not result["passed"]:
        raise SystemExit("Headset engineering gate failed; see verification.json")
    return result


if __name__=="__main__":
    ap=argparse.ArgumentParser()
    ap.add_argument("--out",type=Path,default=Path("build/vr_headset"))
    args=ap.parse_args()
    verify(build(),Spec(),args.out)
