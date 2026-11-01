"""P6-B first structural PETG test articles, derived from actual refined P5 BREP.
Not a completed whole-arm conversion or released load-bearing print design.
Retains interface geometry while reinforcing the tube spans and adding a real
split/coupler joint for the 357 mm upper frame. Required hardware and checks are
recorded rather than silently inheriting metal-frame validation.
"""
from pathlib import Path
import json, math, hashlib
import numpy as np
import cadquery as cq
from OCP.gp import gp_Trsf
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
ROOT=Path(__file__).resolve().parents[1]
RHO=1.25e-6  # kg / mm^3, assumed solid PETG continuum

def box(x,y,z,c):
    return cq.Solid.makeBox(x,y,z,cq.Vector(c[0]-x/2,c[1]-y/2,c[2]-z/2))
def shaft(r,length,start,axis):
    return cq.Solid.makeCylinder(r,length,cq.Vector(*start),cq.Vector(*axis))
def xform(s,R):
    t=gp_Trsf();t.SetValues(*np.c_[R,np.zeros(3)].ravel().tolist())
    return cq.Shape.cast(BRepBuilderAPI_Transform(s.wrapped,t,True).Shape())
def rounded_box_tube(w,h,L,t,cy,outer_radius=0.):
    outer=box(w,L,h,[0,cy,0]);inner=box(w-2*t,L+2,h-2*t,[0,cy,0])
    if outer_radius:
        outer=outer.fillet(outer_radius,[e for e in outer.Edges() if abs(e.Length()-L)<1e-5])
    # Interior bend radius is real CAD, not just normal smoothing.
    edges=[e for e in inner.Edges() if abs(e.Length()-(L+2))<1e-5]
    inner=inner.fillet(1.2,edges)
    return outer.cut(inner)
def cover_nut_mounts(s,ys,outer_z):
    # M2.5 nut pockets are intended for actual nut/insert fit tests.
    inner_z=outer_z-6
    for y in ys:
        for x in [-19,19]:
            s=s.cut(shaft(1.45,2*outer_z+4,[x,y,-outer_z-2],[0,0,1]))
            for sign in [-1,1]:
                # Pocket opens inward into the hollow beam. Nominal 5.4 mm square.
                s=s.cut(box(5.4,5.4,2.15,[x,y,sign*(inner_z+1.025)]))
    return s.clean()
def validate(s,label):
    if not s.isValid() or len(s.Solids())!=1 or s.Volume()<=0:
        raise RuntimeError('Invalid single-solid test article: '+label)
def metrics(s):
    bb=s.BoundingBox();vol=s.Volume();mass=vol*RHO
    return {'volume_mm3':vol,'solid_PETG_mass_kg':mass,'com_parent_mm':list(s.Center().toTuple()),
            'inertia_tensor_about_COM_kg_m2':(np.array(cq.Shape.matrixOfInertia(s))*RHO*1e-6).tolist(),
            'bounds_parent_mm':[[bb.xmin,bb.ymin,bb.zmin],[bb.xmax,bb.ymax,bb.zmax]]}
def export(s,name,source_id,description):
    validate(s,name)
    s.exportBrep(str(ROOT/'cad'/f'{name}.brep'))
    cq.exporters.export(s,str(ROOT/'cad'/f'{name}.step'))
    imp=cq.importers.importStep(str(ROOT/'cad'/f'{name}.step')).val()
    validate(imp,name+' STEP roundtrip')
    # Length runs in the XY print plane; original beam Z is the build direction.
    bb=s.BoundingBox();print_s=s.translate((-bb.xmin+5,-bb.ymin+5,-bb.zmin))
    cq.exporters.export(print_s,str(ROOT/'stl'/f'{name}.stl'),tolerance=.025,angularTolerance=.08)
    cq.exporters.export(print_s,str(ROOT/'cad'/f'{name}-print-local.step'))
    size=[bb.xlen,bb.ylen,bb.zlen]
    if max(size)>246:raise RuntimeError('Planning print envelope exceeded '+name)
    out={'name':name,'source_id':source_id,'description':description,**metrics(s),
         'CAD_valid_single_solid':True,'STEP_valid_single_solid':True,
         'STEP_volume_relative_error':abs(imp.Volume()/s.Volume()-1),
         'print_bounding_size_mm':size,'fits_246mm_planning_cube':True,
         'print_translation_mm':[-bb.xmin+5,-bb.ymin+5,-bb.zmin],
         'print_orientation':'Parent Z normal to plate; long beam direction in XY. Support/bridge strategy NOT sliced or qualified.',
         'status':'STRUCTURAL_TEST_ARTICLE_NOT_POWERED_ROBOT_RELEASE',
         'STL_sha256':hashlib.sha256((ROOT/'stl'/f'{name}.stl').read_bytes()).hexdigest()}
    return out

def main():
    original=cq.Shape.importBrep(str(ROOT/'inputs/P5-upper-frame.brep'))
    upper=original.fuse(rounded_box_tube(60,44,168,6,148)).clean()
    upper=cover_nut_mounts(upper,[96,220],22)
    # Relieve four inherited zero-thickness tangencies at the elbow collar.
    # This is a real 0.15 mm radial clearance change over the local 3.4 mm band.
    upper=upper.cut(shaft(36.65,3.4,[0,280,18.8],[0,0,1])).clean()
    # Four crossed, sleeved M4 through-bolts through the lap-splice region.
    bolt_y=[118.,136.,160.,178.]
    for y in bolt_y:upper=upper.cut(shaft(3.1,70,[-35,y,0],[1,0,0]))
    upper=upper.clean()
    lower=upper.intersect(box(300,547.85,300,[0,-126.075,0])).clean()  # y<=147.85
    upper_half=upper.intersect(box(300,551.85,300,[0,424.075,0])).clean() # y>=148.15
    coupler=rounded_box_tube(47.6,31.6,80,5.,148,outer_radius=1.4)
    # Slight fillets on axial outer edges only, not bolt or bearing geometry.
    for y in bolt_y:coupler=coupler.cut(shaft(3.1,60,[-30,y,0],[1,0,0]))
    coupler=coupler.clean()
    fore0=cq.Shape.importBrep(str(ROOT/'inputs/P5-fore-frame.brep'))
    fore=fore0.fuse(rounded_box_tube(60,60,62,6,95)).clean()
    fore=cover_nut_mounts(fore,[96,114],30)
    records=[]
    for s,n,sid,desc in [
        (lower,'upper-frame-proximal',274,'Reinforced proximal upper frame; original joint end retained for subsequent flange redesign/testing.'),
        (upper_half,'upper-frame-distal',274,'Reinforced distal upper frame; 0.30 mm split gap. Original joint end remains an unqualified print interface.'),
        (coupler,'upper-frame-lap-coupler',None,'80 mm internal PETG sleeve, nominal 0.20 mm per-side radial fit, four sleeved M4 cross-bolts.'),
        (fore,'fore-frame-reinforced',275,'Forearm tube span strengthened from 2.5 to 6 mm nominal wall; end/roll interfaces remain unqualified.')]:
        records.append(export(s,n,sid,desc))
    overlap=[]
    for a,b,an,bn in [(lower,upper_half,'proximal','distal'),(lower,coupler,'proximal','coupler'),(upper_half,coupler,'distal','coupler')]:
        overlap.append({'a':an,'b':bn,'intersection_volume_mm3':a.intersect(b).Volume()})
    if any(x['intersection_volume_mm3']>.01 for x in overlap):
        raise RuntimeError('Test-article nominal assembly interference '+str(overlap))
    # Store an actual assembled joint, not a disconnected exploded illustration.
    assy=cq.Assembly(name='upper-print-prototype')
    for s,n in [(lower,'proximal'),(upper_half,'distal'),(coupler,'lap_coupler')]:
        assy.add(s,name=n)
    assy.save(str(ROOT/'cad/upper-frame-prototype-assembly.step'))
    hardware=[{'item':'M4 through bolt, nominal 75 mm length candidate','quantity':4,'scope':'Confirm actual thread engagement and washer/nut stack; no torque released.'},
              {'item':'Metal compression sleeve OD6 ID4.5 length60 mm candidate','quantity':4,'scope':'Not printed. Length/seating tolerance must be measured before use.'},
              {'item':'M4 metal nut and wide washer pair','quantity':4,'scope':'Positive retention plus printed bearing-load tests required.'},
              {'item':'M2.5 square captive nut in 5.4 mm pocket','quantity':16,'scope':'Eight per link, 16 for both. Replaces inherited metal-tapped cover lands.'},
              {'item':'Revised M2.5 cover screw, nominal 16 mm candidate','quantity':16,'scope':'Full P6-A cap/washer/sleeve stack integration not performed.'}]
    report={'scope':'Two actual load-bearing link test-article designs, not an assembled/qualified whole-robot print conversion.',
            'material_model':'Transparent PETG target, 1250 kg/m3 assumed solid fill, no infill homogenization.',
            'source_upper_bbox_mm':[original.BoundingBox().xlen,original.BoundingBox().ylen,original.BoundingBox().zlen],
            'beam_section_old_nominal_wall_mm':2.5,'beam_section_new_nominal_wall_mm':6.,
            'elbow_collar_local_relief':{'original_radius_mm':36.5,'revised_radius_mm':36.65,'z_band_mm':[18.8,22.2],'reason':'Remove inherited tangent zero-thickness topology; new interface fit remains unqualified.'},
            'upper_joint_gap_mm':.3,'upper_sleeve_length_mm':80,'upper_sleeve_radial_clearance_mm':.2,
            'parts':records,'nominal_intersections':overlap,'hardware_candidates':hardware,
            'limitations':['Joint-end flange bending, captive-nut pullout, layer anisotropy, creep and fatigue not qualified.',
                           'No full-arm swept collision, self-collision or hardware load test has been run for these prototypes.',
                           'Original bolt patterns retained at actuator interfaces; not enough to qualify a plastic load path.',
                           'STL build envelope checked, but not sliced or printed; supports, shrinkage and real tolerances remain unverified.',
                           'The four splice sleeves/bolts are specified, not modeled in the new STEP assembly.',
                           'Not a replacement for the archived P6-A simulation or validated dynamics model.']}
    (ROOT/'reports/structural-prototypes.json').write_text(json.dumps(report,indent=2))
    print(json.dumps({'parts':len(records),'total_PETG_g':sum(x['solid_PETG_mass_kg'] for x in records)*1000,
                      'nominal_intersections':overlap,'print_sizes':{r['name']:r['print_bounding_size_mm'] for r in records}},indent=2))
if __name__=='__main__':main()
