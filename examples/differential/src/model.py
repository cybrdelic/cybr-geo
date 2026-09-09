"""Reference-preserving inspection and an explicitly separate kinematic core.

Millimetres; shaft along X, Z up. Original 81 meshes never overwritten.
The alternative core uses 48/18-tooth parallel-axis helical stages and indexed
18/18 spur coupling stages, three planet pairs, and prescribed carrier/output
angles. These are ideal rigid-body kinematics, NOT a contact-force solver.
"""
from __future__ import annotations
import json, math, sys, hashlib
from pathlib import Path
from dataclasses import dataclass, field
import numpy as np
import trimesh
import vtk
from vtk.util.numpy_support import numpy_to_vtk, numpy_to_vtkIdTypeArray, vtk_to_numpy

ROOT = Path(__file__).resolve().parents[1]
GEOM = ROOT/'geometry'
M = json.loads((GEOM/'reference_manifest.json').read_text())
MATERIALS = M['materials']
NS, NP, MODULE, BETA = 48, 18, .94, math.radians(25)
RS, RP = NS*MODULE/2, NP*MODULE/2
ORBIT = RS+RP
DELTA = math.asin(RP/ORBIT)
RATIO=NS/NP

@dataclass
class Part:
    name:str
    vertices:np.ndarray
    faces:np.ndarray
    normals:np.ndarray
    material:int=0
    group:str='carrier'
    motion:str='carrier'
    center:np.ndarray=field(default_factory=lambda:np.zeros(3))
    explode:np.ndarray=field(default_factory=lambda:np.zeros(3))
    role:str='reference visual component'

    @property
    def bounds(self): return np.array([self.vertices.min(0), self.vertices.max(0)])


def load_reference()->list[Part]:
    a=np.load(GEOM/'reference_mesh_arrays.npz')
    result=[]
    for i,item in enumerate(M['components']):
        n=item['name']; p=Part(n,a[n+'__vertices'].copy(),a[n+'__faces'].copy(),a[n+'__normals'].copy(),item['material'],item['group'])
        p.role='unchanged source mesh'
        if item['group'] in ['front_hub'] or n.startswith('Front_shaft') or n.startswith('Front_bore'):p.motion='left'
        if item['group']=='rear_hub' or n.startswith('Rear_inner_output'):p.motion='right'
        if n=='Front_helical_gear' or n=='Front_gear_collar':p.motion='left'
        if n=='Rear_helical_gear' or n=='Rear_gear_collar':p.motion='right'
        if n.startswith('Front_bearing_inner'):p.motion='left'
        if n.startswith('Rear_bearing_inner'):p.motion='right'
        if n.startswith('Rear_clutch_plate_') and int(n[-2:])%2==0:p.motion='right'
        # Carefully separated inspection layout, not assembly travel paths.
        if item['group'] in ['carrier','marking']:p.explode=np.array([0,24,112.])
        elif item['group']=='front_flange':p.explode=np.array([-141.,0,0])
        elif item['group']=='front_hub':p.explode=np.array([-160.,0,0])
        elif n.startswith('Front_bearing'):p.explode=np.array([-111.,0,0])
        elif n=='Front_oil_seal':p.explode=np.array([-131.,0,0])
        elif n=='Front_thrust_ring':p.explode=np.array([-78.,0,0])
        elif n.startswith('Front_'):p.explode=np.array([-57.,0,0])
        elif item['group']=='rear_flange':p.explode=np.array([173.,0,0])
        elif item['group']=='rear_hub' or n=='Rear_inner_output_sleeve':p.explode=np.array([198.,0,0])
        elif n.startswith('Rear_bearing'):p.explode=np.array([144.,0,0])
        elif n=='Rear_oil_seal':p.explode=np.array([159.,0,0])
        elif n.startswith('Rear_clutch_plate'):p.explode=np.array([47.+int(n[-2:])*9.,0,0])
        elif n=='Bronze_wave_preload_spring':p.explode=np.array([64.,-62.,0])
        elif n=='Rear_clutch_retainer':p.explode=np.array([124.,0,0])
        elif n=='Rear_thrust_ring':p.explode=np.array([46.,0,0])
        elif n.startswith('Rear_'):p.explode=np.array([22.,0,0])
        else:p.explode=np.array([0.,0,0])
        result.append(p)
    return result


def normals(v,f,angle=38):
    poly=vtk.vtkPolyData(); pts=vtk.vtkPoints(); pts.SetData(numpy_to_vtk(np.ascontiguousarray(v,np.float64),deep=True)); poly.SetPoints(pts)
    cells=vtk.vtkCellArray(); cells.SetCells(len(f),numpy_to_vtkIdTypeArray(np.column_stack([np.full(len(f),3),f]).astype(np.int64).ravel(),deep=True));poly.SetPolys(cells)
    clean=vtk.vtkCleanPolyData();clean.SetInputData(poly);clean.SetTolerance(1e-9);clean.Update()
    n=vtk.vtkPolyDataNormals();n.SetInputConnection(clean.GetOutputPort());n.SetFeatureAngle(angle);n.SplittingOn();n.ConsistencyOn();n.AutoOrientNormalsOn();n.Update()
    p=n.GetOutput();return (vtk_to_numpy(p.GetPoints().GetData()).copy(),vtk_to_numpy(p.GetPolys().GetData()).reshape(-1,4)[:,1:].copy(),vtk_to_numpy(p.GetPointData().GetNormals()).copy())


def make_part(name,v,f,material=2,motion='carrier',center=(0,0,0),group='kinematic_core',role='new kinematic-core component',angle=35):
    v,f,n=normals(np.asarray(v),np.asarray(f),angle)
    return Part(name,v,f,n,material,group,motion,np.array(center,float),role=role)


def profile(teeth:int,module:float,backlash:float=.03,nflank:int=10):
    """Transverse involute profile with tooth-thickness allowance at pitch circle.
    Root arcs are visualization fillets, not cutter-envelope root certification.
    """
    rp=teeth*module/2; rb=rp*math.cos(math.radians(20)); rr=rp-1.25*module;ra=rp+module
    def inv(r):
        if r<=rb:return 0.
        q=math.sqrt((r/rb)**2-1);return q-math.atan(q)
    h=math.pi/(2*teeth)-backlash/(2*rp)
    ip=inv(rp); pts=[]
    for j in range(teeth):
        c=2*math.pi*j/teeth; hr=h+ip
        pts.append((rr,c-hr-.010))
        pts.append((rr+.10,c-hr-.002))
        for r in np.linspace(max(rb,rr+.10),ra,nflank):pts.append((r,c-(h+ip-inv(r))))
        ha=h+ip-inv(ra)
        for q in np.linspace(-ha,ha,6)[1:]:pts.append((ra,c+q))
        for r in np.linspace(ra,max(rb,rr+.10),nflank)[1:]:pts.append((r,c+h+ip-inv(r)))
        pts.extend([(rr+.10,c+hr+.002),(rr,c+hr+.010)])
        nxt=c+2*math.pi/teeth-hr-.010
        for q in np.linspace(c+hr+.010,nxt,5)[1:-1]:pts.append((rr,q))
    return np.asarray(pts,float)


def gear(name,teeth,x0,x1,helix,phase,center=(0,0),bore=3.05,motion='carrier',mat=2):
    pr=profile(teeth,MODULE); rad=pr[:,0];ang=pr[:,1];N=len(rad);rp=teeth*MODULE/2
    width=x1-x0;bev=min(.16,width*.1)
    layers=[(x0,bev),(x0+.12,.04)]+[(x,0.) for x in np.linspace(x0+.24,x1-.24,27)]+[(x1-.12,.04),(x1,bev)]
    v=[];f=[]
    for x,inset in layers:
        a=ang+phase+x*math.tan(helix)/rp
        v.extend(np.column_stack((np.full(N,x),(rad-inset)*np.cos(a)+center[0],(rad-inset)*np.sin(a)+center[1])))
    for j in range(len(layers)-1):
        for k in range(N):
            a=j*N+k;b=j*N+(k+1)%N;c=(j+1)*N+(k+1)%N;d=(j+1)*N+k
            f.extend([(a,b,c),(a,c,d)])
    inner=[]
    for j in [0,len(layers)-1]:
        x=layers[j][0];aa=ang+phase+x*math.tan(helix)/rp; st=len(v);inner.append(st)
        v.extend(np.column_stack((np.full(N,x),bore*np.cos(aa)+center[0],bore*np.sin(aa)+center[1])))
        for k in range(N):
            a=j*N+k;b=j*N+(k+1)%N;c=st+(k+1)%N;d=st+k
            f.extend([(a,c,b),(a,d,c)] if j==0 else [(a,b,c),(a,c,d)])
    for k in range(N):
        a=inner[0]+k;b=inner[0]+(k+1)%N;c=inner[1]+(k+1)%N;d=inner[1]+k;f.extend([(a,c,b),(a,d,c)])
    return make_part(name,v,f,mat,motion,(0,*center),angle=33)


def lathe(name,ro,ri,x0,x1,material=3,motion='carrier',center=(0,0),bevel=.15,sides=192):
    c=min(bevel,(x1-x0)*.24,(ro-ri)*.24)
    pr=[(x0,ri+c),(x0,ro-c),(x0+c,ro),(x1-c,ro),(x1,ro-c),(x1,ri+c),(x1-c,ri),(x0+c,ri)]
    v=[];f=[]
    for x,r in pr:
        for k in range(sides):
            a=k*2*math.pi/sides;v.append((x,r*math.cos(a)+center[0],r*math.sin(a)+center[1]))
    for j in range(len(pr)):
        for k in range(sides):
            a=j*sides+k;b=j*sides+(k+1)%sides;c=((j+1)%len(pr))*sides+(k+1)%sides;d=((j+1)%len(pr))*sides+k;f.extend([(a,b,c),(a,c,d)])
    return make_part(name,v,f,material,motion,(0,*center))


def phase_partner(n1,n2,gamma,p1=0.):
    return (((n1+n2)*gamma+n2*math.pi-math.pi-n1*p1)/n2)%(2*math.pi/n2)


def build_kinematic_core():
    parts=[]; pairmeta=[]
    parts.append(gear('K01_Left_side_gear_48T',NS,-35,-10.5,-BETA,0,bore=16.12,motion='left'))
    parts.append(gear('K02_Right_side_gear_48T',NS,3,25,BETA,0,bore=17.12,motion='right'))
    for j in range(3):
        base=math.radians(155+120*j);a=base-DELTA;b=base+DELTA
        ca=(ORBIT*math.cos(a),ORBIT*math.sin(a));cb=(ORBIT*math.cos(b),ORBIT*math.sin(b))
        pha=phase_partner(NS,NP,a);phb=phase_partner(NS,NP,b)
        gamma=math.atan2(cb[1]-ca[1],cb[0]-ca[0]);psa=0.;psb=phase_partner(NP,NP,gamma)
        for letter,c,ph,ps,x0,x1,he,motion in [('A',ca,pha,psa,-35,-10.5,BETA,'planetA'),('B',cb,phb,psb,3,25,-BETA,'planetB')]:
            pre=f'K_Pair{j+1}_{letter}'
            parts.append(gear(pre+'_helical_18T',NP,x0,x1,he,ph,c,motion=motion,mat=3))
            parts.append(gear(pre+'_coupling_spur_18T',NP,-8.5,1.5,0,ps,c,motion=motion,mat=2))
            parts.append(lathe(pre+'_rigid_journal',4.4,3.05,-35.25,25.25,3,motion,c,.12,128))
            parts.append(lathe(pre+'_carrier_pin',2.95,.01,-38.3,28.1,4,'carrier',c,.15,96))
            parts.append(lathe(pre+'_front_bush',4.8,3.0,-38.5,-36.4,5,'carrier',c,.12,128))
            parts.append(lathe(pre+'_rear_bush',4.8,3.0,26.2,28.3,5,'carrier',c,.12,128))
        pairmeta.append(dict(pair=j+1,a=a,b=b,ca=ca,cb=cb,phase_ha=pha,phase_hb=phb,phase_sa=psa,phase_sb=psb,gamma=gamma))
    # Support plates have actual pin holes rather than interpenetrating disks.
    import cadquery as cq
    for side,x0,x1 in [('Front',-38.5,-36.4),('Rear',26.2,28.3)]:
        sh=cq.Workplane('YZ').circle(39.6).circle(18.4).extrude(x1-x0).translate((x0,0,0))
        for pair in pairmeta:
            for key in ['ca','cb']:
                y,z=pair[key]
                cut=cq.Workplane('YZ').center(y,z).circle(4.82).extrude(x1-x0+2).translate((x0-1,0,0));sh=sh.cut(cut)
        try:sh=sh.edges().chamfer(.16)
        except Exception:pass
        vv,ff=sh.val().tessellate(.03,.06)
        parts.append(make_part('K_'+side+'_six_bore_support_plate',[[q.x,q.y,q.z] for q in vv],ff,0))
        cq.exporters.export(sh,str(GEOM/f'K_{side}_support_plate.step'))
    parts.append(lathe('K_Left_inner_thrust_collar',18.3,16.10,-10.2,-8.75,3,'left'))
    parts.append(lathe('K_Right_inner_thrust_collar',18.3,17.10,1.8,2.75,3,'right'))
    parts.append(lathe('K_Front_bearing_shaft_adapter',18.45,16.03,-44.8,-38.6,3,'left'))
    parts.append(lathe('K_Rear_bearing_shaft_adapter',18.75,17.03,41.7,48.7,3,'right'))
    for p in parts:
        p.explode=np.array([(-42 if p.motion=='left' else 42 if p.motion=='right' else 0),0,0.])
        if p.motion in ['planetA','planetB']:p.explode=np.array([0,p.center[1]*.8,p.center[2]*.8])
    return parts,pairmeta


def rotation_x(a):
    c,s=math.cos(a),math.sin(a);return np.array([[1,0,0],[0,c,-s],[0,s,c]],float)


def transform(part:Part,c=0.,d=0.,explode=0.):
    """theta_L=c+d, theta_R=c-d; local planets A=-Ns/Np*d, B=+Ns/Np*d."""
    R=rotation_x(c);t=np.zeros(3)
    if part.motion=='left':R=rotation_x(c+d)
    elif part.motion=='right':R=rotation_x(c-d)
    elif part.motion in ['planetA','planetB']:
        q=(-RATIO if part.motion=='planetA' else RATIO)*d
        Rlocal=rotation_x(q);t=R@(part.center-Rlocal@part.center);R=R@Rlocal
    t+=part.explode*explode
    T=np.eye(4);T[:3,:3]=R;T[:3,3]=t;return T


def save_parts(parts,path):
    np.savez_compressed(path,**{p.name+'__'+k:getattr(p,k) for p in parts for k in ['vertices','faces','normals']})
    meta=[]
    for p in parts:
        meta.append(dict(name=p.name,material=p.material,group=p.group,motion=p.motion,center=p.center.tolist(),explode=p.explode.tolist(),role=p.role,triangles=len(p.faces)))
    path.with_suffix('.json').write_text(json.dumps(meta,indent=2))


def load_parts(path):
    a=np.load(path);info=json.loads(path.with_suffix('.json').read_text());res=[]
    for p in info:
        p.pop('triangles',None);n=p['name'];p.update({k:a[n+'__'+k] for k in ['vertices','faces','normals']});p['center']=np.array(p['center']);p['explode']=np.array(p['explode']);res.append(Part(**p))
    return res


def export_glb(parts,path,poses=None):
    sc=trimesh.Scene(); mats=[]
    for m in MATERIALS:
        rgb=np.uint8(np.clip(np.power(m['color'],1/2.2)*255,0,255));mats.append(trimesh.visual.material.PBRMaterial(name=m['name'],baseColorFactor=[*rgb,255],metallicFactor=m['metal'],roughnessFactor=m['rough']))
    for i,p in enumerate(parts):
        mesh=trimesh.Trimesh(p.vertices,p.faces,vertex_normals=p.normals,process=False);mesh.visual=trimesh.visual.TextureVisuals(material=mats[p.material]);sc.add_geometry(mesh,node_name=p.name,geom_name=p.name,transform=np.eye(4) if poses is None else poses[i])
    T=np.array([[.001,0,0,0],[0,0,.001,0],[0,-.001,0,0],[0,0,0,1]],float);sc.apply_transform(T);sc.export(path)


def compatible_reference(ref):
    # Retain carrier, flanges, hubs, bearings, seals and visual clutch stack.
    # Remove original unmatched gears and collars, whose radii collide with planets.
    return [p for p in ref if p.group!='gears' and p.name not in ['Front_thrust_ring']]


def export_meshbin(parts,path,poses=None):
    rec=[]
    for i,p in enumerate(parts):
        T=np.eye(4) if poses is None else poses[i];v=p.vertices@T[:3,:3].T+T[:3,3];n=p.normals@T[:3,:3].T
        rec.append(np.c_[v[p.faces].reshape(-1,9),n[p.faces].reshape(-1,9),np.full(len(p.faces),p.material),np.full(len(p.faces),10)])
    arr=np.concatenate(rec).astype('<f4')
    with open(path,'wb') as f:np.array([len(arr)],dtype='<u4').tofile(f);arr.tofile(f)


def validate(core,pairs):
    from shapely.geometry import Polygon
    from shapely.affinity import rotate,translate
    # Static cross sections at seven axial positions and 25 distinct motions,
    # evaluated against independently transformed polygons (not angle residuals only).
    pp=profile(NP,MODULE,nflank=18);sp=profile(NS,MODULE,nflank=18)
    def poly(pr,angle,center):
        return Polygon(np.c_[pr[:,0]*np.cos(pr[:,1]+angle)+center[0],pr[:,0]*np.sin(pr[:,1]+angle)+center[1]])
    worst=0.;closest=1e9;cases=0
    for pair in pairs:
        for d in np.linspace(0,2*math.pi/NS,25,endpoint=False):
            for x in [-34.,-26.,-18.,-11.]:
                s=poly(sp,-x*math.tan(BETA)/RS+d,(0,0));a=poly(pp,pair['phase_ha']+x*math.tan(BETA)/RP-RATIO*d,pair['ca'])
                ar=s.intersection(a).area;worst=max(worst,ar);closest=min(closest,s.distance(a));cases+=1
            for x in [3.5,12.,24.5]:
                s=poly(sp,x*math.tan(BETA)/RS-d,(0,0));b=poly(pp,pair['phase_hb']-x*math.tan(BETA)/RP+RATIO*d,pair['cb'])
                ar=s.intersection(b).area;worst=max(worst,ar);closest=min(closest,s.distance(b));cases+=1
            a=poly(pp,pair['phase_sa']-RATIO*d,pair['ca']);b=poly(pp,pair['phase_sb']+RATIO*d,pair['cb']);worst=max(worst,a.intersection(b).area);closest=min(closest,a.distance(b));cases+=1
    max_res=0.
    for c in np.linspace(-9,9,401):
        for d in [-1.2,-.7,0,.2,1.]:
            L=c+d;R=c-d;pA=c-RATIO*d;pB=c+RATIO*d
            res=[L+R-2*c,NS*(L-c)+NP*(pA-c),NS*(R-c)+NP*(pB-c),NP*(pA-c)+NP*(pB-c)]
            max_res=max(max_res,*map(abs,res))
    checks=[]
    for p in core:
        t=trimesh.Trimesh(p.vertices,p.faces,process=True);checks.append(dict(name=p.name,finite=bool(np.isfinite(p.vertices).all()),watertight=bool(t.is_watertight),triangles=len(p.faces),bounds_mm=p.bounds.tolist()))
    result=dict(source_reference_component_count=len(M['components']),kinematic_core_component_count=len(core),gear_design=dict(side_teeth=NS,pinion_teeth=NP,transverse_module_mm=MODULE,transverse_pressure_deg=20,helix_deg=25,side_pitch_radius_mm=RS,pinion_pitch_radius_mm=RP,pinion_orbit_radius_mm=ORBIT,maximum_gear_tip_radius_mm=ORBIT+RP+MODULE,tooth_thickness_allowance_mm=.03,pinion_pairs=3),independent_cross_section_tests=cases,maximum_sampled_intersection_area_mm2=worst,minimum_sampled_mating_clearance_mm=closest,max_angular_constraint_residual_rad_times_teeth=max_res,meshes=checks,limitations=['Rigid-body prescribed kinematics; no contact force integration or load response.','Sampled transverse polygon checks do not certify full 3D collision-free manufacturing geometry.','Original bearings and clutch are retained visualization geometry, not engineered tolerances or torque-bias elements.','Pin and bore fits, lubrication, bearing raceways, preload and stress require further engineering.','Exterior is unchanged from the prior reference reconstruction, not an exact recovery of the generated concept.'])
    (ROOT/'validation'/'kinematic_checks.json').write_text(json.dumps(result,indent=2))
    print(json.dumps({k:v for k,v in result.items() if k not in ['meshes','limitations']},indent=2),flush=True)
    if worst>1e-5:raise AssertionError(f'Gear profile intersections: {worst}')
    if max_res>1e-10:raise AssertionError('Kinematic residual failure')
    if not all(p['watertight'] for p in checks):print('WARNING: non-watertight mesh found',flush=True)
    return result


def main():
    ref=load_reference();save_parts(ref,GEOM/'reference_parts.npz')
    core,pairs=build_kinematic_core();save_parts(core,GEOM/'kinematic_core_parts.npz')
    (GEOM/'kinematic_pairs.json').write_text(json.dumps(pairs,indent=2));validate(core,pairs)
    combined=compatible_reference(ref)+core
    save_parts(combined,GEOM/'working_variant_parts.npz')
    export_glb(core,GEOM/'kinematic_core.glb');export_glb(combined,GEOM/'working_variant.glb')
    export_glb(ref,GEOM/'reference_exploded.glb',[transform(p,explode=1) for p in ref])
    print('COMPLETE',len(ref),'reference;',len(core),'core;',len(combined),'working variant',flush=True)

if __name__=='__main__':main()
