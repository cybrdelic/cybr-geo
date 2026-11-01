"""Coarser printable tooth trials, not drop-in P5/P6 gripper replacements.
Involute flanks are polygonally sampled. Root transitions are radial straight
segments, not a generated cutter trochoid. No load/life rating is claimed.
"""
from pathlib import Path
import math,json,hashlib
import cadquery as cq
ROOT=Path(__file__).resolve().parents[1]
M=1.2; Z=20; ALPHA=math.radians(20); FACE=12.;BACKLASH_THICKNESS=.10

def inv(r,rb):
    return math.sqrt(max(0.,(r/rb)**2-1))-math.acos(min(1.,rb/r))
def main():
    rp=M*Z/2;rb=rp*math.cos(ALPHA);ra=rp+M;rf=rp-1.25*M
    half=(math.pi*M/2-BACKLASH_THICKNESS)/(2*rp)
    phib=half+inv(rp,rb);phia=phib-inv(ra,rb);points=[]
    def add(r,a):
        v=cq.Vector(r*math.cos(a),r*math.sin(a),0)
        if not points or (v-points[-1]).Length>1e-8:points.append(v)
    for k in range(Z):
        a=2*math.pi*k/Z;add(rf,a-phib);add(rb,a-phib)
        for j in range(1,19):
            r=rb+(ra-rb)*j/18;add(r,a-(phib-inv(r,rb)))
        for j in range(1,9):add(ra,a-phia+2*phia*j/8)
        for j in range(1,19):
            r=ra-(ra-rb)*j/18;add(r,a+(phib-inv(r,rb)))
        add(rf,a+phib)
        end=2*math.pi*(k+1)/Z-phib
        for j in range(1,8):add(rf,a+phib+(end-a-phib)*j/8)
    pinion=cq.Solid.extrudeLinear(cq.Wire.makePolygon(points,close=True),[],cq.Vector(0,0,FACE))
    pinion=pinion.cut(cq.Solid.makeCylinder(2.05,FACE+2,cq.Vector(0,0,-1))).clean()
    pitch=math.pi*M;n=16;length=n*pitch
    hp=(math.pi*M/2-BACKLASH_THICKNESS)/2
    ha=hp-M*math.tan(ALPHA);hf=hp+1.25*M*math.tan(ALPHA)
    outline=[cq.Vector(-length/2,-6,0),cq.Vector(-length/2,-1.25*M,0)]
    for i in range(n):
        x=-length/2+(i+.5)*pitch
        outline += [cq.Vector(x-hf,-1.25*M,0),cq.Vector(x-ha,M,0),cq.Vector(x+ha,M,0),cq.Vector(x+hf,-1.25*M,0)]
    outline += [cq.Vector(length/2,-1.25*M,0),cq.Vector(length/2,-6,0)]
    rack=cq.Solid.extrudeLinear(cq.Wire.makePolygon(outline,close=True),[],cq.Vector(0,0,FACE)).clean()
    rec=[]
    for name,s in [('gripper-gear-tooth-trial-m1p2-z20',pinion),('gripper-rack-tooth-trial-m1p2',rack)]:
        if not s.isValid() or len(s.Solids())!=1:raise RuntimeError(name)
        cq.exporters.export(s,str(ROOT/'cad'/f'{name}.step'))
        t=cq.importers.importStep(str(ROOT/'cad'/f'{name}.step')).val()
        if not t.isValid() or len(t.Solids())!=1:raise RuntimeError('STEP '+name)
        b=s.BoundingBox();p=s.translate((-b.xmin+5,-b.ymin+5,-b.zmin))
        cq.exporters.export(p,str(ROOT/'stl'/f'{name}.stl'),tolerance=.015,angularTolerance=.06)
        rec.append({'name':name,'valid_single_solid':True,'STEP_roundtrip_valid':True,
                    'dimensions_mm':[b.xlen,b.ylen,b.zlen],'volume_mm3':s.Volume(),
                    'solid_PETG_mass_g':s.Volume()*.00125,
                    'STL_sha256':hashlib.sha256((ROOT/'stl'/f'{name}.stl').read_bytes()).hexdigest()})
    report={'status':'PRINTED_TOOTH_TEST_ARTICLES_ONLY_NOT_INTEGRATED_WITH_ROBOT',
            'module_mm':M,'teeth':Z,'pressure_angle_degrees':20,'face_width_mm':FACE,
            'tooth_thickness_reduction_mm':BACKLASH_THICKNESS,'pinion_bore_mm':4.1,
            'original_module_mm':.3,'original_teeth':20,'pitch_radius_ratio':M/.3,
            'additional_torque_ratio_for_same_rack_force':M/.3,
            'fixed_torque_new_rack_force_fraction':.3/M,
            'rack_tip_nominal_width_mm':2*ha,
            'parts':rec,
            'limitations':['Bore is for a 4 mm test mandrel; NOT the original gripper motor shaft interface.',
                           'Larger pinion requires redesigned bridge, racks, jaw passages and controller scaling.',
                           'Tooth roots use simple straight radial transitions; generated trochoid and root relief remain to be designed.',
                           'No rolling contact interference, contact ratio, stress, wear, life or backlash test executed.',
                           'Printed tooth quality and clearance must be measured; STL creation is not a successful print claim.']}
    (ROOT/'reports/printed-gear-trials.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2))
if __name__=='__main__':main()
