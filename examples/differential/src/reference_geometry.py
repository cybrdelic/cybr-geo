"""Reference-led TORSEN-X visual reconstruction.

Everything seen in the renderer is polygon geometry, not an image plane.
Coordinate conventions: CAD uses Z as shaft axis; exported meshes use X.
Dimensions are inferred from the principal reference image, not its inconsistent
printed dimensions. This is a visual concept, NOT a production differential.

Dependencies: numpy, cadquery, vtk, trimesh, scipy, Pillow.
"""
from __future__ import annotations
import json, math, os, time
from pathlib import Path
import numpy as np
import cadquery as cq
from cadquery import exporters
import trimesh
import vtk
from vtk.util.numpy_support import numpy_to_vtk, vtk_to_numpy, numpy_to_vtkIdTypeArray

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'geometry'
OUT.mkdir(parents=True, exist_ok=True)
CACHE=OUT/'cache'; CACHE.mkdir(exist_ok=True)
PARTS=[]
MATERIALS=[
    dict(name='Satin machined carrier',color=[.53,.55,.56],metal=1.,rough=.27,pattern=2),
    dict(name='Turned flange steel',color=[.64,.66,.67],metal=1.,rough=.22,pattern=1),
    dict(name='Ground gear steel',color=[.32,.33,.34],metal=1.,rough=.25,pattern=3),
    dict(name='Polished bearing steel',color=[.69,.71,.73],metal=1.,rough=.16,pattern=1),
    dict(name='Blackened retainer steel',color=[.095,.11,.125],metal=.88,rough=.26,pattern=1),
    dict(name='Bronze clutch separator',color=[.29,.215,.105],metal=.93,rough=.29,pattern=1),
    dict(name='Engraved dark marking',color=[.022,.026,.029],metal=.55,rough=.39,pattern=0),
    dict(name='Black elastomer seal',color=[.014,.017,.019],metal=0.,rough=.48,pattern=0),
]

def log(s): print(s,flush=True)

def mm_transform(v):
    # [radial CAD X, radial CAD Y, shaft CAD Z] -> [shaft X, radial Y, height Z]
    return np.asarray(v)[:,[2,0,1]]

def calc_normals(vertices,faces,feature_angle=38):
    poly=vtk.vtkPolyData(); pts=vtk.vtkPoints()
    pts.SetData(numpy_to_vtk(np.asarray(vertices,dtype=np.float64),deep=True)); poly.SetPoints(pts)
    cs=vtk.vtkCellArray()
    packed=np.column_stack([np.full(len(faces),3,dtype=np.int64),np.asarray(faces,dtype=np.int64)]).ravel()
    cs.SetCells(len(faces),numpy_to_vtkIdTypeArray(packed,deep=True)); poly.SetPolys(cs)
    clean=vtk.vtkCleanPolyData(); clean.SetInputData(poly); clean.SetTolerance(1e-9); clean.Update()
    norm=vtk.vtkPolyDataNormals(); norm.SetInputConnection(clean.GetOutputPort()); norm.SplittingOn()
    norm.SetFeatureAngle(feature_angle); norm.ConsistencyOn(); norm.AutoOrientNormalsOn()
    norm.ComputePointNormalsOn(); norm.ComputeCellNormalsOff(); norm.Update()
    p=norm.GetOutput()
    v=vtk_to_numpy(p.GetPoints().GetData()).copy()
    f=vtk_to_numpy(p.GetPolys().GetData()).reshape(-1,4)[:,1:].copy()
    n=vtk_to_numpy(p.GetPointData().GetNormals()).copy()
    return v,f,n

def add_mesh(name,vertices,faces,material=0,group='carrier',cad_coordinates=True,angle=38):
    v=np.asarray(vertices,dtype=np.float64)
    if cad_coordinates: v=mm_transform(v)
    v,f,n=calc_normals(v,faces,angle)
    PARTS.append(dict(name=name,vertices=v,faces=f,normals=n,material=material,group=group))
    log(f'{name}: {len(v):,} vertices / {len(f):,} triangles')
    return PARTS[-1]

def add_cad(name,shape,material=0,group='carrier',save_step=False):
    sh=shape.val() if isinstance(shape,cq.Workplane) else shape
    if not sh.isValid(): raise ValueError('Invalid CAD solid: '+name)
    vs,fs=sh.tessellate(.045,.055)
    p=add_mesh(name,[[v.x,v.y,v.z] for v in vs],fs,material,group)
    if save_step: exporters.export(sh,str(OUT/(name+'.step')))
    return p

def fillet(shape,r=.6):
    for rr in [r,r*.65,r*.35]:
        try:
            s=shape.edges().fillet(rr)
            if s.val().isValid(): return s
        except Exception: pass
    log(f'No global fillet at radius {r}; retaining existing engineered profile edges')
    return shape

def annulus(ro,ri,z0,z1,bevel=0.):
    sh=cq.Workplane('XY').circle(ro).circle(ri).extrude(z1-z0).translate((0,0,z0))
    if bevel:
        try: sh=sh.edges().chamfer(bevel)
        except Exception: sh=fillet(sh,bevel)
    return sh

def revolve_profile(profile):
    return cq.Workplane('XZ').polyline(profile).close().revolve(360,(0,0),(0,1))

def lathe_mesh(name,profile,material,group='hub',sides=256):
    verts=[]; faces=[]
    for r,z in profile:
        for i in range(sides):
            t=2*math.pi*i/sides; verts.append((r*math.cos(t),r*math.sin(t),z))
    K=len(profile)
    for j in range(K):
        for i in range(sides):
            a=j*sides+i; b=j*sides+(i+1)%sides
            c=((j+1)%K)*sides+(i+1)%sides; d=((j+1)%K)*sides+i
            faces.extend([(a,b,c),(a,c,d)])
    return add_mesh(name,verts,faces,material,group,angle=33)

def ring_profile(ro,ri,z0,z1,c=.25):
    c=min(c,(z1-z0)*.4,(ro-ri)*.3)
    return [(ri+c,z0),(ro-c,z0),(ro,z0+c),(ro,z1-c),(ro-c,z1),
            (ri+c,z1),(ri,z1-c),(ri,z0+c)]

def lathe_ring(name,ro,ri,z0,z1,material,group,c=.25):
    return lathe_mesh(name,ring_profile(ro,ri,z0,z1,c),material,group)


def make_carrier():
    cache=CACHE/'sculpted_carrier_top_window_fillet_first.brep'
    if cache.exists(): return cq.Workplane('XY').newObject([cq.Shape.importBrep(str(cache))])
    # The waist and rear shoulder are deliberate: no added center flange.
    outer=[(51.7,-46),(51.4,-43),(49.3,-37),(47.4,-27),(47.0,-14),
           (47.9,-7),(50.8,0),(53.0,6),(52.6,16),(52.9,29),(55.2,40),(57.8,47)]
    inner=[(51.1,47),(48.6,40),(46.4,29),(46.2,16),(46.6,6),(44.4,0),
           (41.6,-7),(40.8,-14),(41.2,-27),(43.0,-37),(45.1,-43),(45.4,-46)]
    wp=cq.Workplane('XZ').moveTo(*outer[0]).spline(outer[1:],includeCurrent=True)
    wp=wp.lineTo(*inner[0]).spline(inner[1:],includeCurrent=True).close()
    sh=wp.revolve(360,(0,0),(0,1))
    # Broad axial apertures, with a dogleg through the shoulder.
    pts=[(-19,-41.6),(19,-41.6),(27.0,-37.2),(27.0,-22),(23.8,-9),
         (25.6,1),(29.2,10),(30.8,34),(27.3,40.8),(-27.3,40.8),
         (-30.8,34),(-29.2,10),(-25.6,1),(-23.8,-9),(-27.,-22),(-27.,-37.2)]
    cutter=cq.Workplane('XZ',origin=(0,85,0)).polyline(pts).close().extrude(72)
    cutter=cutter.edges('|Y').fillet(3.1)
    for theta in [85,145,235,325]:
        log(f'Cut carrier window {theta}')
        active=cutter
        if theta==85:
            # The reference has a separate narrow TOP opening, not four
            # identical angularly spaced slots hidden on the far side.
            tp=[(t*.60,z*.85+4.) for t,z in pts]
            active=cq.Workplane('XZ',origin=(0,85,0)).polyline(tp).close().extrude(72).edges('|Y').fillet(2.2)
        sh=sh.cut(active.rotate((0,0,0),(0,0,1),theta-90))
    sh=fillet(sh,.85)
    # Continue the rear mounting bores through the cage lip. Their angular
    # phase accounts for the assembly's subsequent -12-degree clocking.
    for i in range(8):
        a=math.radians(i*45+34.5)
        hole=cq.Workplane('XY').center(49.8*math.cos(a),49.8*math.sin(a)).circle(4.9).extrude(10.).translate((0,0,39.))
        sh=sh.cut(hole)
    sh.val().exportBrep(str(cache))
    return sh


def flange(name,ro,ri,z0,z1,bolt_r,hole_r,side='front'):
    sh=annulus(ro,ri,z0,z1)
    for i in range(8):
        a=2*math.pi*(i/8)+math.radians(22.5)
        x,y=bolt_r*math.cos(a),bolt_r*math.sin(a)
        sh=sh.cut(cq.Workplane('XY').center(x,y).circle(hole_r).extrude(z1-z0+4).translate((0,0,z0-2)))
    for i in range(8):
        a=2*math.pi*i/8; x,y=(bolt_r+.15)*math.cos(a),(bolt_r+.15)*math.sin(a)
        sh=sh.cut(cq.Workplane('XY').center(x,y).circle(1.32 if side=='front' else 1.5).extrude(z1-z0+4).translate((0,0,z0-2)))
    try: sh=sh.edges().chamfer(.42)
    except Exception: sh=fillet(sh,.42)
    add_cad(name,sh,1,'front_flange' if side=='front' else 'rear_flange',True)
    return sh


def spline_socket():
    # A short hollow internal-spline opening, NOT the long solid stub in v1.
    ro=19.8; r_minor=14.9; r_major=15.85; teeth=32
    pts=[]
    for i in range(teeth):
        c=2*math.pi*i/teeth; pitch=2*math.pi/teeth
        for f,r in [(-.50,r_major),(-.25,r_major),(-.19,r_minor),(.19,r_minor),(.25,r_major)]:
            t=c+f*pitch; pts.append((r*math.cos(t),r*math.sin(t)))
    sh=annulus(ro,14.7,-74,-64.8)
    cut=cq.Workplane('XY').polyline(pts).close().extrude(12).translate((0,0,-75))
    sh=sh.cut(cut)
    try: sh=sh.faces('<Z').edges().chamfer(.38)
    except Exception: pass
    try: sh=sh.faces('>Z').edges().chamfer(.2)
    except Exception: pass
    add_cad('Front_hollow_32_spline_socket',sh,1,'front_hub',True)
    return sh


def involute_gear(name,teeth,module,width,zc,helix_deg,material=2,phase=0.,offset=(0,0),bore=15.):
    rp=teeth*module*.5; rb=rp*math.cos(math.radians(20)); rr=rp-1.25*module; ra=rp+module
    inv=lambda r: (math.sqrt(max((r/rb)**2-1,0))-math.acos(min(rb/r,1))) if r>=rb else 0.
    invp=inv(rp); h=math.pi/(2*teeth)-.0013
    pts=[]
    for i in range(teeth):
        center=2*math.pi*i/teeth+phase
        root_angle=h+invp
        # Flank sampled from base/root through the exact involute to addendum.
        rstart=max(rr,rb)
        pts.append((rr,center-root_angle-.006))
        pts.append((rr+.22,center-root_angle-.002))
        for r in np.linspace(rstart,ra,9):
            pts.append((float(r),center-(h+invp-inv(float(r)))))
        ht=h+invp-inv(ra)
        for a in np.linspace(-ht,ht,7)[1:]: pts.append((ra,center+float(a)))
        for r in np.linspace(ra,rstart,9)[1:]:
            pts.append((float(r),center+(h+invp-inv(float(r)))))
        pts.extend([(rr+.22,center+root_angle+.002),(rr,center+root_angle+.006)])
        nextroot=center+2*math.pi/teeth-root_angle-.006
        for a in np.linspace(center+root_angle+.006,nextroot,5)[1:-1]: pts.append((rr,float(a)))
    pts=np.array(pts); N=len(pts)
    # Multi-layer twist; axial tooth-end chamfers are true geometry.
    w=width
    sections=[(-w/2,.43),(-w/2+.13,.16),(-w/2+.43,0.)]
    sections += [(float(z),0.) for z in np.linspace(-w/2+.8,w/2-.8,35)]
    sections += [(w/2-.43,0.),(w/2-.13,.16),(w/2,.43)]
    vertices=[]; faces=[]; slope=math.tan(math.radians(helix_deg))/rp
    for z,inset in sections:
        aa=pts[:,1]+slope*z; rad=pts[:,0]-inset
        vertices.extend(np.column_stack((rad*np.cos(aa)+offset[0],rad*np.sin(aa)+offset[1],np.full(N,zc+z))))
    for j in range(len(sections)-1):
        for i in range(N):
            a=j*N+i;b=j*N+(i+1)%N;c=(j+1)*N+(i+1)%N;d=(j+1)*N+i
            faces.extend([(a,b,c),(a,c,d)])
    # Annular end faces / continuous bored interior.
    inner=[]
    for j in [0,len(sections)-1]:
        start=len(vertices); z,inset=sections[j]
        aa=pts[:,1]+slope*z; br=bore+.3
        for a in aa: vertices.append((br*math.cos(a)+offset[0],br*math.sin(a)+offset[1],zc+z))
        inner.append(start)
        for i in range(N):
            a=j*N+i;b=j*N+(i+1)%N;c=start+(i+1)%N;d=start+i
            faces.extend([(a,c,b),(a,d,c)] if j==0 else [(a,b,c),(a,c,d)])
    for i in range(N):
        a=inner[0]+i;b=inner[0]+(i+1)%N;c=inner[1]+(i+1)%N;d=inner[1]+i
        faces.extend([(a,c,b),(a,d,c)])
    return add_mesh(name,vertices,faces,material,'gears',angle=31)


def bearing(prefix,zc,outer=34.,inner=18.,balls=20):
    lathe_ring(prefix+'_outer_race',outer,outer-3.9,zc-3.6,zc+3.6,3,'bearing')
    lathe_ring(prefix+'_inner_race',inner+3.9,inner,zc-3.6,zc+3.6,3,'bearing')
    rr=(outer+inner)/2
    for i in range(balls):
        a=2*math.pi*i/balls
        m=trimesh.creation.icosphere(subdivisions=2,radius=min(3.45,(outer-inner)/4.5))
        v=np.asarray(m.vertices)+[rr*math.cos(a),rr*math.sin(a),zc]
        add_mesh(prefix+f'_ball_{i:02}',v,m.faces,3,'bearing',angle=70)
    # Thin dust-shield rings without hiding the steel balls in exploded view.
    lathe_ring(prefix+'_retainer',outer-4.3,inner+4.3,zc+.7,zc+1.25,5,'bearing',.08)


def wave_spring(name,r,zc,turns=4.,pitch=.95,amp=.32,wire=.32):
    ns=int(220*turns); sides=10; v=[]; f=[]
    for i in range(ns+1):
        t=2*math.pi*turns*i/ns; z=zc+pitch*(t/(2*math.pi)-turns/2)+amp*math.sin(t*5)
        c=np.array((r*math.cos(t),r*math.sin(t),z))
        radial=np.array((math.cos(t),math.sin(t),0.))
        tangent=np.array((-r*math.sin(t),r*math.cos(t),pitch/(2*math.pi)+amp*5*math.cos(t*5)))
        tangent/=np.linalg.norm(tangent); other=np.cross(tangent,radial)
        for j in range(sides):
            q=2*math.pi*j/sides; v.append(c+wire*(math.cos(q)*radial+math.sin(q)*other))
    for i in range(ns):
        for j in range(sides):
            a=i*sides+j;b=i*sides+(j+1)%sides;c=(i+1)*sides+(j+1)%sides;d=(i+1)*sides+j
            f.extend([(a,b,c),(a,c,d)])
    # End caps, so the spring is a closed mesh.
    for i in [0,ns]:
        base=i*sides; ci=len(v);v.append(np.mean(np.asarray(v)[base:base+sides],axis=0))
        for j in range(sides): f.append((ci,base+(j+1)%sides,base+j) if i==0 else (ci,base+j,base+(j+1)%sides))
    add_mesh(name,v,f,5,'clutch',angle=60)


def make_marking():
    # Individual font outlines are triangulated solids and wrapped onto the
    # housing's lower/near longitudinal rail, never pasted on as a texture.
    from scipy.interpolate import CubicSpline
    zpts=np.array([-46,-43,-37,-27,-14,-7,0,6,16,29,40,47.])
    rr=np.array([51.7,51.4,49.3,47.4,47.,47.9,50.8,53.,52.6,52.9,55.2,57.8])
    radius=CubicSpline(zpts,rr)
    theta=math.radians(178)
    specs=[('TORSEN-X',6.0,5.0),('TB-250',4.1,-.6),('S/N 0047',3.8,-5.0)]
    for label,size,offset in specs:
        sh=cq.Workplane('XY').text(label,size,.04,font='DejaVu Sans',kind='regular',halign='center',valign='center',combine=False).val()
        vs,fs=sh.tessellate(.035,.1)
        coords=[]
        for v in vs:
            axial=v.x-15.
            # Positive text Y goes upward on the lower near-side rail.
            arc=offset+v.y; th=theta-arc/float(radius(axial)); r=float(radius(axial))+.04+v.z
            coords.append((r*math.cos(th),r*math.sin(th),axial))
        add_mesh('Marking_'+label.replace('/','-').replace(' ','_'),coords,fs,6,'marking')


def export_all():
    scene=trimesh.Scene()
    materials=[]
    for m in MATERIALS:
        rgb=np.clip(np.power(m['color'],1/2.2)*255,0,255).astype(np.uint8)
        materials.append(trimesh.visual.material.PBRMaterial(name=m['name'],baseColorFactor=[*rgb,255],metallicFactor=m['metal'],roughnessFactor=m['rough']))
    # All objects remain separate and carry stable component names.
    for p in PARTS:
        mesh=trimesh.Trimesh(vertices=p['vertices'],faces=p['faces'],vertex_normals=p['normals'],process=False)
        mesh.visual=trimesh.visual.TextureVisuals(material=materials[p['material']])
        scene.add_geometry(mesh,geom_name=p['name'],node_name=p['name'])
    # glTF is Y-up and meters; the renderer / CAD intentionally remain mm.
    T=np.array([[.001,0,0,0],[0,0,.001,0],[0,-.001,0,0],[0,0,0,1]],float)
    scene.apply_transform(T)
    scene.export(OUT/'TORSEN_X_reference_rebuild.glb')
    # Compact binary with per-vertex normals, material IDs, group IDs.
    groups=list(dict.fromkeys(p['group'] for p in PARTS))
    records=[]
    for p in PARTS:
        v=p['vertices'][p['faces']].astype(np.float32)
        n=p['normals'][p['faces']].astype(np.float32)
        pack=np.concatenate([v.reshape(-1,9),n.reshape(-1,9),np.full((len(v),1),p['material'],np.float32),np.full((len(v),1),groups.index(p['group']),np.float32)],axis=1)
        records.append(pack)
    array=np.vstack(records).astype('<f4')
    with open(OUT/'scene.meshbin','wb') as f:
        np.array([len(array)],dtype='<u4').tofile(f); array.tofile(f)
    np.savez_compressed(OUT/'mesh_arrays.npz',**{p['name']+'__'+k:p[k] for p in PARTS for k in ['vertices','faces','normals']})
    manifest=dict(title='TORSEN-X: principal-reference visual reconstruction',units='millimeters',axis='X',
                  source_of_shape='Principal hero rendering in the user-provided generated board',
                  status='Visual concept mesh, not a validated differential or a reverse-engineered commercial product',
                  changes_from_previous=['Removed center mounting ring','Removed front solid spline stub','New asymmetric flanges and short bored hub','Sculpted continuous cage and rounded windows','Larger exposed helical gears','Path-traced studio metal lighting'],
                  materials=MATERIALS,groups=groups,components=[dict(name=p['name'],material=p['material'],group=p['group'],vertices=len(p['vertices']),triangles=len(p['faces'])) for p in PARTS],
                  triangle_count=len(array),vertex_count=sum(len(p['vertices']) for p in PARTS))
    (OUT/'manifest.json').write_text(json.dumps(manifest,indent=2))
    log('EXPORTED '+str(len(PARTS))+' components; '+str(len(array))+' triangles')


def main():
    t=time.time()
    carrier=make_carrier().rotate((0,0,0),(0,0,1),-12);add_cad('Sculpted_windowed_carrier',carrier,0,'carrier',True)
    front=flange('Front_eight_hole_flange',52,17.5,-54,-46.0,42.65,5.65,'front')
    rear=flange('Rear_eight_hole_drive_flange',58.3,20,47,55,49.8,4.9,'rear')
    # Fine machined rings at the flange's outer edge, not extra end plates.
    lathe_ring('Front_outer_witness_groove',52.035,51.89,-48.7,-48.47,4,'front_flange',.04)
    lathe_ring('Rear_outer_witness_groove',58.325,58.18,50.65,50.88,4,'rear_flange',.04)
    profile=[(16.0,-54.05),(32.6,-54.05),(32.6,-55.8),(32.2,-56.4),(28.6,-56.4),
             (28.6,-58.0),(28.2,-58.4),(26.3,-58.4),(26.3,-61.2),(25.9,-61.6),
             (22.1,-61.6),(22.1,-63.0),(21.8,-63.3),(20.0,-63.3),(20.,-65.4),(16.,-65.4)]
    hub=revolve_profile(profile);add_cad('Front_stepped_hollow_hub',hub,1,'front_hub',True)
    spline_socket()
    for i,r in enumerate([29.25,30.0,31.0]):
        lathe_ring(f'Front_turning_line_{i}',r+.075,r,-56.46,-56.39,4,'front_hub',.01)
    lathe_ring('Front_shaft_inner_sleeve',16.0,14.6,-65,-7.0,4,'shaft')
    lathe_ring('Front_bore_rear_rim',15.9,13.9,-7.4,-6.8,3,'shaft')
    rearprofile=[(16,54.8),(28.6,54.8),(28.6,57.8),(28.2,58.4),(25.7,58.4),(25.7,64.2),
                 (25.3,64.7),(21.,64.7),(21.,71.4),(20.5,71.9),(16.,71.9)]
    rh=revolve_profile(rearprofile);add_cad('Rear_short_output_hub',rh,1,'rear_hub',True)
    lathe_ring('Rear_inner_output_sleeve',17.0,14.6,4.,75.5,4,'shaft')
    # Two principal visible helical masses reproduce the reference, rather
    # than substituting a generic planetary cage.
    involute_gear('Front_helical_gear',24,2.35,28.,-22.,-28.,phase=.02)
    involute_gear('Rear_helical_gear',26,2.45,30.,13.4,29.,phase=.04)
    lathe_ring('Front_gear_collar',25.3,15.,-9.,-4.,3,'gears',.4)
    lathe_ring('Central_black_separator',25.0,15.,-4.,-2.65,4,'gears',.25)
    lathe_ring('Rear_gear_collar',26.8,15.,-2.6,.2,3,'gears',.4)
    # Subtle collars and thrust washers behind each visible tooth field.
    lathe_ring('Front_thrust_ring',32.1,17.0,-38.,-36.15,4,'clutch')
    lathe_ring('Rear_thrust_ring',36.8,17.0,28.7,31.25,4,'clutch')
    for i in range(7):
        z=31.8+i*1.5
        lathe_ring(f'Rear_clutch_plate_{i+1:02}',36.8 if i%2==0 else 35.9,19.0,z,z+.95,5 if i%2 else 4,'clutch',.12)
    wave_spring('Bronze_wave_preload_spring',37.45,36.8)
    lathe_ring('Rear_clutch_retainer',38.1,18.,42.6,44.2,3,'clutch',.3)
    bearing('Front_bearing',-41.8,outer=34.,inner=18.5)
    bearing('Rear_bearing',45.2,outer=36.,inner=18.8)
    lathe_ring('Front_oil_seal',19.0,16.2,-46.1,-43.6,7,'bearing')
    lathe_ring('Rear_oil_seal',20.0,17.1,49.,52.,7,'bearing')
    make_marking()
    export_all()
    log(f'Build complete in {time.time()-t:.1f}s')

if __name__=='__main__': main()
