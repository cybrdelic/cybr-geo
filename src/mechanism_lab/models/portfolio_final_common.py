"""Shared high-fidelity geometry primitives for the portfolio machine.

This deliberately follows the level used by CYBR ORBIT / ROAM / CYBR SCENES:
- manufactured bodies are real OpenCascade BReps with bores, counterbores,
  fasteners, split interfaces, bearings and serviceable subassemblies;
- environments are actual dense meshes with authored relief, implicit rocks,
  branch geometry, foliage and water surfaces;
- no image projection, scan-derived mesh, billboard vegetation or image-gen mesh.

Coordinates are millimetres throughout.
"""
from __future__ import annotations

import math
from dataclasses import replace
import numpy as np
import cadquery as cq
import trimesh
import vtk
from vtk.util.numpy_support import numpy_to_vtk, vtk_to_numpy

from mechanism_lab.core import Material, cad_part, mesh_part
from mechanism_lab.geometry import tube_mesh
from mechanism_lab.advanced_geometry import spline_sweep_tube


MATERIALS = [
    Material("Graphite anodized aluminium", (.072,.082,.098), 1.0,.31, coat=.20, coat_rough=.22, anisotropy=.18, microfinish="anodized"),
    Material("Satin machined aluminium", (.55,.59,.63), 1.0,.24, anisotropy=.58, microfinish="brushed"),
    Material("Hardened steel", (.39,.43,.47), 1.0,.19, anisotropy=.32, microfinish="machined"),
    Material("Warm bronze", (.58,.34,.115), 1.0,.26, anisotropy=.28, microfinish="turned"),
    Material("Copper enamel", (.84,.30,.055), .80,.24, coat=.18, coat_rough=.15, microfinish="copper-wire"),
    Material("Black elastomer", (.012,.014,.017), 0.0,.70, ior=1.46, microfinish="polymer"),
    Material("Optical glass", (.76,.87,.98), 0.0,.035, ior=1.49, coat=.50, coat_rough=.03, opacity=.22),
    Material("Spring / process water", (.075,.42,.58), 0.0,.045, ior=1.333, coat=.22, coat_rough=.025, opacity=.48),
    Material("Hot ceramic / lava", (.98,.16,.018), .10,.22),
    Material("Basalt", (.15,.16,.17), .06,.78, microfinish="concrete"),
    Material("Weathered carbonate", (.62,.51,.34), .02,.82, microfinish="concrete"),
    Material("Sand / mineral sediment", (.56,.42,.26), .0,.88, microfinish="concrete"),
    Material("Oak", (.42,.23,.09), .0,.40, anisotropy=.38, microfinish="wood"),
    Material("Maple", (.70,.54,.32), .0,.36, anisotropy=.32, microfinish="wood"),
    Material("Dark stone", (.22,.23,.24), .05,.46, microfinish="concrete"),
    Material("Light stone", (.58,.56,.51), .02,.43, microfinish="concrete"),
    Material("Forest bark", (.27,.14,.055), .0,.82, microfinish="wood"),
    Material("Forest foliage", (.075,.30,.085), .0,.72),
    Material("Moss", (.10,.34,.075), .0,.88),
    Material("Dry branch", (.30,.16,.065), .0,.87, microfinish="wood"),
    Material("Dry foliage", (.38,.36,.16), .0,.82),
    Material("Laser beam", (.16,.46,1.0), .0,.08, ior=1.01, opacity=.82),
    Material("Ceramic white", (.76,.77,.74), .0,.30, coat=.08, coat_rough=.14),
]


def box(dx,dy,dz,center=(0,0,0),fillet=0.0):
    q=cq.Workplane("XY").box(float(dx),float(dy),float(dz))
    if fillet:
        try:q=q.edges().fillet(float(fillet))
        except Exception:pass
    return q.val().translate(tuple(map(float,center)))


def cylinder(radius,length,origin,axis="X"):
    axes={"X":(1,0,0),"Y":(0,1,0),"Z":(0,0,1)}
    return cq.Solid.makeCylinder(float(radius),float(length),cq.Vector(*map(float,origin)),cq.Vector(*axes[axis]))


def annulus(ro,ri,start,length,axis="X"):
    q=cylinder(ro,length,start,axis)
    if ri:
        axes={"X":(1,0,0),"Y":(0,1,0),"Z":(0,0,1)}
        p=np.asarray(start,float)-np.asarray(axes[axis],float)
        q=q.cut(cylinder(ri,length+2,p,axis))
    return q


def sphere(r,center=(0,0,0)):
    return cq.Solid.makeSphere(float(r),cq.Vector(*map(float,center)))


def capsule_plate(length,width,thickness,center=(0,0,0),axis="Z"):
    if axis=="Z":
        q=cq.Workplane("XY").slot2D(length,width).extrude(thickness,both=True).val()
    elif axis=="X":
        q=cq.Workplane("YZ").slot2D(length,width).extrude(thickness,both=True).val()
    else:
        q=cq.Workplane("XZ").slot2D(length,width).extrude(thickness,both=True).val()
    return q.translate(tuple(map(float,center)))


def add_cad(parts,name,shape,material=0,group="structure",motion="fixed",explode=(0,0,0),
            role="",tags=(),tol=.030,ang=.065,finish_axis=(1,0,0),finish_origin=None):
    p=cad_part(name,shape,material,tolerance=tol,angular=ang,analytic_normals=True,
               group=group,motion=motion,explode=np.asarray(explode,float),
               role=role or name.replace("_"," "),provenance="designed-concept",
               tags=tuple(tags),finish_axis=finish_axis,finish_origin=finish_origin)
    parts.append(p)
    return p


def add_mesh(parts,name,v,f,n,material=0,group="environment",role="",explode=(0,0,0),tags=()):
    p=mesh_part(name,np.asarray(v,float),np.asarray(f,np.int64),material,
                group=group,explode=np.asarray(explode,float),
                role=role or name.replace("_"," "),provenance="designed-concept",tags=tuple(tags))
    # Preserve authored normals when topology was not changed by smoothing.
    if len(p.normals)==len(v):
        p=replace(p,normals=np.asarray(n,float))
    # Mesh parts are first-class assembly members. The earlier implementation
    # accidentally replaced the previous CAD part, which silently discarded
    # terrain, rocks, water and vegetation one mesh at a time.
    parts.append(p)
    return p


def cylinder_between(a,b,r):
    a=np.asarray(a,float);b=np.asarray(b,float);d=b-a
    L=float(np.linalg.norm(d))
    if L<=1e-8:raise ValueError("zero-length cylinder")
    return cq.Solid.makeCylinder(float(r),L,cq.Vector(*a),cq.Vector(*(d/L)))


def socket_screw_x(x,y,z,length=20,diam=5,head_scale=1.72,material=2):
    r=diam/2
    shaft=cylinder(r,length,(x,y,z),"X")
    head=cylinder(r*head_scale,diam*.82,(x-diam*.82,y,z),"X")
    socket=cq.Workplane("YZ",origin=(x-diam*.83,y,z)).polygon(6,diam*.62).extrude(diam*.48).val()
    return shaft.fuse(head).cut(socket).clean()


def socket_screw_z(x,y,z,length=20,diam=5,head_scale=1.72):
    r=diam/2
    shaft=cylinder(r,length,(x,y,z),"Z")
    head=cylinder(r*head_scale,diam*.82,(x,y,z-diam*.82),"Z")
    socket=cq.Workplane("XY",origin=(x,y,z-diam*.83)).polygon(6,diam*.62).extrude(diam*.48).val()
    return shaft.fuse(head).cut(socket).clean()


def hex_nut_x(x,y,z,diam=6,thickness=5):
    r=diam/2
    q=cq.Workplane("YZ",origin=(x,y,z)).polygon(6,diam*1.82).extrude(thickness).val()
    return q.cut(cylinder(r*.84,thickness+2,(x-1,y,z),"X"))


def fluted_knob_x(x,y,z,ro=22,ri=5,length=14,flutes=32):
    q=annulus(ro,ri,(x,y,z),length,"X")
    cutters=[]
    for a in np.linspace(0,math.tau,flutes,endpoint=False):
        cutters.append(cylinder(ro*.075,length+2,(x-1,y+ro*math.cos(a),z+ro*math.sin(a)),"X"))
    q=q.cut(cq.Compound.makeCompound(cutters))
    try:q=cq.Workplane(obj=q).edges().chamfer(.8).val()
    except Exception:pass
    return q


def flange_x(x,ro,ri,width,bolt_radius=0,bolt_count=0,bolt_hole=0):
    q=annulus(ro,ri,(x-width/2,0,0),width,"X")
    if bolt_count and bolt_hole:
        for a in np.linspace(0,math.tau,bolt_count,endpoint=False):
            y=bolt_radius*math.cos(a);z=bolt_radius*math.sin(a)
            q=q.cut(cylinder(bolt_hole,width+2,(x-width/2-1,y,z),"X"))
    try:q=cq.Workplane(obj=q).edges("|X").chamfer(min(1.2,width*.12)).val()
    except Exception:pass
    return q


def bearing_x(parts,prefix,x,r_outer,r_inner,width,explode=0,motion_inner="fixed"):
    center=x+width/2
    ball_r=(r_outer-r_inner)*.145
    pitch=(r_outer+r_inner)/2
    torus=cq.Solid.makeTorus(pitch,ball_r*1.05,(center,0,0),(1,0,0))
    outer=annulus(r_outer,pitch+ball_r*.78,(x,0,0),width,"X").cut(torus)
    inner=annulus(pitch-ball_r*.78,r_inner,(x,0,0),width,"X").cut(torus)
    add_cad(parts,prefix+"_Outer_race",outer,2,"bearings",explode=(-explode,0,0),role="Hardened outer bearing race")
    add_cad(parts,prefix+"_Inner_race",inner,2,"bearings",motion=motion_inner,explode=(explode,0,0),role="Hardened inner bearing race")
    balls=[]
    count=max(10,int(2*math.pi*pitch/(ball_r*2.8)))
    for i,a in enumerate(np.linspace(0,math.tau,count,endpoint=False)):
        c=(center,pitch*math.cos(a),pitch*math.sin(a))
        s=sphere(ball_r*.92,c)
        balls.append(sphere(ball_r*1.02,c))
        add_cad(parts,f"{prefix}_Ball_{i:02}",s,2,"bearing_balls",motion=motion_inner,
                explode=(explode*.5,0,0),role="Rolling element",tol=.06,ang=.12)
    # A boolean-subtracted one-piece cage becomes topologically fragile for
    # small bearings. ORBIT-grade construction uses separate retained cage
    # rings and bridges, which is also closer to a serviceable bearing cage.
    cage_x0=x+width*.30
    cage_x1=x+width*.64
    for ci,cx in enumerate((cage_x0,cage_x1)):
        cage_ring=annulus(pitch+ball_r*.46,pitch-ball_r*.46,(cx,0,0),width*.065,"X")
        add_cad(parts,f"{prefix}_Cage_ring_{ci}",cage_ring,3,"bearings",motion=motion_inner,
                explode=(explode*.7,0,0),role="Separate bearing cage retainer ring",tol=.05,ang=.09)
    for i,a in enumerate(np.linspace(0,math.tau,count,endpoint=False)):
        aa=a+math.pi/count
        y=pitch*math.cos(aa);z=pitch*math.sin(aa)
        bridge=cylinder( max(1.0,ball_r*.16), cage_x1-cage_x0+width*.065,
                         (cage_x0,y,z),"X")
        add_cad(parts,f"{prefix}_Cage_bridge_{i:02}",bridge,3,"bearings",motion=motion_inner,
                explode=(explode*.7,0,0),role="Bearing cage bridge between rolling elements",
                tol=.07,ang=.12)


def bolt_circle_x(parts,prefix,x,radius,count,diam=5,length=12,phase=0,explode=0):
    for i,a in enumerate(np.linspace(0,math.tau,count,endpoint=False)+phase):
        y=radius*math.cos(a);z=radius*math.sin(a)
        add_cad(parts,f"{prefix}_{i:02}",socket_screw_x(x,y,z,length,diam),2,"fasteners",
                explode=(explode,0,0),role="Socket flange fastener",tol=.05,ang=.10)


def bolt_circle_z(parts,prefix,z,radius,count,diam=5,length=12,phase=0,explode=0):
    for i,a in enumerate(np.linspace(0,math.tau,count,endpoint=False)+phase):
        x=radius*math.cos(a);y=radius*math.sin(a)
        add_cad(parts,f"{prefix}_{i:02}",socket_screw_z(x,y,z,length,diam),2,"fasteners",
                explode=(0,0,explode),role="Socket ring fastener",tol=.05,ang=.10)


def pipe(parts,name,points,radius=4.5,material=3,group="service",role="Routed service line",explode=(0,0,0),analytic=True):
    pts=[tuple(map(float,p)) for p in points]
    if analytic and len(pts)>=3:
        try:
            q=spline_sweep_tube(pts,radius)
            return add_cad(parts,name,q,material,group,explode=explode,role=role,tol=.08,ang=.11)
        except Exception:
            pass
    v,f=tube_mesh(np.asarray(pts,float),radius=radius,sides=12)
    n=mesh_normals(v,f)
    return add_mesh(parts,name,v,f,n,material,group,role,explode)


# --------------------- authored mesh geometry ---------------------

def mesh_normals(v,f):
    v=np.asarray(v,float);f=np.asarray(f,np.int64)
    n=np.zeros_like(v)
    face=np.cross(v[f[:,1]]-v[f[:,0]],v[f[:,2]]-v[f[:,0]])
    for j in range(3):np.add.at(n,f[:,j],face)
    n/=np.maximum(np.linalg.norm(n,axis=1)[:,None],1e-20)
    return n


def join_mesh(items):
    vs=[];fs=[];ns=[];offset=0
    for v,f,n in items:
        vs.append(np.asarray(v,float));fs.append(np.asarray(f,np.int64)+offset);ns.append(np.asarray(n,float));offset+=len(v)
    if not vs:return np.zeros((0,3)),np.zeros((0,3),np.int64),np.zeros((0,3))
    return np.concatenate(vs),np.concatenate(fs),np.concatenate(ns)


def gridmesh(xs,ys,h):
    xs=np.asarray(xs,float);ys=np.asarray(ys,float);h=np.asarray(h,float)
    xx,yy=np.meshgrid(xs,ys)
    v=np.column_stack((xx.ravel(),yy.ravel(),h.ravel()))
    ny,nx=xx.shape
    ind=np.arange(nx*ny).reshape(ny,nx)
    a=ind[:-1,:-1].ravel();b=ind[:-1,1:].ravel();c=ind[1:,1:].ravel();d=ind[1:,:-1].ravel()
    f=np.vstack((np.column_stack((a,b,c)),np.column_stack((a,c,d))))
    gy,gx=np.gradient(h,ys,xs)
    n=np.column_stack((-gx.ravel(),-gy.ravel(),np.ones(nx*ny)))
    n/=np.maximum(np.linalg.norm(n,axis=1)[:,None],1e-20)
    return v,f,n


def _hash2(ix,iy,seed):
    x=np.sin(ix*127.1+iy*311.7+seed*74.71)*43758.5453123
    return (x-np.floor(x))*2-1


def noise2(x,y,seed=0):
    x,y=np.broadcast_arrays(np.asarray(x,float),np.asarray(y,float))
    ix=np.floor(x);iy=np.floor(y);fx=x-ix;fy=y-iy
    ux=fx*fx*(3-2*fx);uy=fy*fy*(3-2*fy)
    a=_hash2(ix,iy,seed);b=_hash2(ix+1,iy,seed);c=_hash2(ix,iy+1,seed);d=_hash2(ix+1,iy+1,seed)
    return (a*(1-ux)+b*ux)*(1-uy)+(c*(1-ux)+d*ux)*uy


def fbm2(x,y,octaves=6,seed=0,lacunarity=2.03,gain=.5):
    out=np.zeros(np.broadcast(x,y).shape,float);amp=1.;freq=1.
    for i in range(octaves):
        out+=amp*noise2(np.asarray(x)*freq+11.3*i,np.asarray(y)*freq-7.1*i,seed+i*19)
        amp*=gain;freq*=lacunarity
    return out


def terrain_mesh(kind="desert",radius=365,z0=285,res=321,seed=20261004):
    xs=np.linspace(-radius,radius,res);ys=np.linspace(-radius,radius,res)
    x,y=np.meshgrid(xs,ys)
    rr=np.sqrt((x/radius)**2+(y/radius)**2)
    if kind=="desert":
        # Low-amplitude eroded shelf. Large-scale silhouette comes from explicit
        # flat-topped mesa fields below, not from smooth spherical hills.
        h=12*fbm2(x/250,y/250,6,seed)+3.5*fbm2(x/58,y/58,5,seed+3)
        mesas=[
            (-205,105,150,100,82), (190,120,178,112,88),
            (248,-132,102,78,66), (-242,-145,82,82,68), (22,225,74,72,58),
        ]
        for mi,(cx,cy,amp,sx,sy) in enumerate(mesas):
            q=np.sqrt(((x-cx)/sx)**2+((y-cy)/sy)**2)
            # Broad top, steep shoulder, then eroded talus. This produces the
            # characteristic table/mesa silhouette visible in the reference.
            shoulder=np.clip((q-.46)/.28,0,1);shoulder=shoulder*shoulder*(3-2*shoulder)
            body=amp*(1-shoulder)
            talus=amp*.14*np.clip((1.06-q)/.32,0,1)
            cliff_band=np.exp(-((q-.66)/.12)**2)
            strata=(4.5*np.sin(q*54+mi*.8)+2.0*np.sin(q*109-mi))*cliff_band
            erosion=6.0*fbm2((x-cx)/34,(y-cy)/34,4,seed+31+mi)*cliff_band
            h+=np.where(q<1.06,body+talus+strata+erosion,0)
        # Irregular geothermal spring basin with a narrow overflow channel.
        q=np.sqrt(((x+5)/192)**2+((y+28)/146)**2)
        basin=np.power(np.clip(1-q,0,1),1.7)
        h-=92*basin
        terrace=6*np.sin(np.minimum(q,1.15)*58+.5*fbm2(x/45,y/45,3,seed+91))
        terrace*=np.exp(-((q-.89)/.13)**2)
        h+=terrace
        channel=x-(108+28*np.sin(y/64)+7*np.sin(y/21))
        h-=28*np.exp(-(channel/19)**2)*np.exp(-((y+165)/175)**4)
        h+=z0
    elif kind=="forest":
        h=26*fbm2(x/220,y/220,6,seed)+9*fbm2(x/58,y/58,5,seed+11)
        for cx,cy,amp,s in [(-170,80,120,135),(150,120,105,130),(225,-120,80,95),(-220,-140,68,100),(0,220,75,105)]:
            q=((x-cx)/s)**2+((y-cy)/s)**2
            h+=amp*np.power(np.clip(1-q,0,1),.72)
        # stream ravine.
        channel=x-(35+45*np.sin((y+80)/120))
        h-=42*np.exp(-(channel/34)**2)
        h+=z0
    else:
        h=z0+18*fbm2(x/180,y/180,6,seed)
    # bend terrain down near vessel wall.
    edge=np.clip((rr-.78)/.20,0,1);edge=edge*edge*(3-2*edge)
    h=h*(1-edge)+(z0-18)*edge
    mask=rr<=1.0
    # Keep a square grid but push corners down below the machine deck; dome hides them.
    h=np.where(mask,h,z0-60-90*np.clip(rr-1,0,None))
    return gridmesh(xs,ys,h)


def water_pool_mesh(cx=0,cy=0,rx=185,ry=135,z=250,nr=36,nt=160,seed=19):
    verts=[(cx,cy,z)];faces=[]
    for ir in range(1,nr+1):
        r=ir/nr
        for j in range(nt):
            a=math.tau*j/nt
            edge=1+.07*math.sin(3*a+.5)+.035*math.sin(7*a-1.1)
            x=cx+rx*r*edge*math.cos(a);y=cy+ry*r*edge*math.sin(a)
            zz=z+1.8*math.sin(x*.045+y*.022)+.9*math.sin(x*.09-y*.075)
            verts.append((x,y,zz))
    for j in range(nt):
        faces.append((0,1+j,1+(j+1)%nt))
    for ir in range(1,nr):
        a0=1+(ir-1)*nt;a1=1+ir*nt
        for j in range(nt):
            k=(j+1)%nt
            faces.extend(((a0+j,a1+j,a1+k),(a0+j,a1+k,a0+k)))
    v=np.asarray(verts,float);f=np.asarray(faces,np.int64);n=mesh_normals(v,f)
    return v,f,n


def implicit_rock_template(seed=0,resolution=54):
    axes=np.linspace(-1.12,1.12,resolution)
    x,y,z=np.meshgrid(axes,axes,axes,indexing="ij")
    d=np.sqrt((x/1.04)**2+(y/.92)**2+(z/.98)**2)-.82
    rng=np.random.default_rng(seed)
    normals=rng.normal(size=(7,3));normals/=np.linalg.norm(normals,axis=1)[:,None]
    for n,dist in zip(normals,rng.uniform(.58,.86,len(normals))):
        d=np.maximum(d,x*n[0]+y*n[1]+z*n[2]-dist)
    d+=.020*np.sin(x*11.1+seed*.21)*np.sin(y*9.2-seed*.13)*np.sin(z*10.4)
    d+=.009*np.sin(x*28+y*21-z*25+seed)
    for _ in range(6):
        direction=rng.normal(size=3);direction/=np.linalg.norm(direction)
        c=direction*rng.uniform(.64,.83);r=rng.uniform(.06,.14)
        cav=np.sqrt((x-c[0])**2+(y-c[1])**2+(z-c[2])**2)-r
        d=np.maximum(d,-cav)
    image=vtk.vtkImageData();image.SetDimensions(resolution,resolution,resolution)
    image.SetOrigin(float(axes[0]),float(axes[0]),float(axes[0]))
    pitch=float(axes[1]-axes[0]);image.SetSpacing(pitch,pitch,pitch)
    image.GetPointData().SetScalars(numpy_to_vtk(d.astype(np.float32).ravel(order="F"),deep=True))
    contour=vtk.vtkFlyingEdges3D();contour.SetInputData(image);contour.SetValue(0,0.);contour.ComputeNormalsOff();contour.Update()
    poly=contour.GetOutput()
    v=vtk_to_numpy(poly.GetPoints().GetData()).copy()
    f=vtk_to_numpy(poly.GetPolys().GetConnectivityArray()).reshape(-1,3).copy()
    signed=np.einsum("ij,ij->i",v[f[:,0]],np.cross(v[f[:,1]],v[f[:,2]])).sum()/6
    if signed<0:f=f[:,[0,2,1]]
    return v,f,mesh_normals(v,f)


def fractured_block_mesh(seed=0,resolution=(78,58,92)):
    """Closed irregular rounded-cuboid rock core with real cracks and cavities.

    The field starts from a high-order box SDF, adds multiscale relief, then
    subtracts several narrow fault sheets and surface cavities before
    extracting one connected FlyingEdges surface.
    """
    nx,ny,nz=map(int,resolution)
    xs=np.linspace(-1.15,1.15,nx);ys=np.linspace(-.88,.88,ny);zs=np.linspace(-1.28,1.28,nz)
    x,y,z=np.meshgrid(xs,ys,zs,indexing="ij")
    # Smooth super-ellipsoid approximates a chipped rectangular earth block.
    p=8.0
    d=((np.abs(x)/.90)**p+(np.abs(y)/.62)**p+(np.abs(z)/1.03)**p)**(1/p)-1.0
    # Deterministic geometric surface relief.
    d+=.030*np.sin(x*8.3+seed*.17)*np.sin(y*10.7-seed*.11)*np.sin(z*7.1)
    d+=.014*np.sin(x*23.0-y*17.0+z*19.0+seed)
    rng=np.random.default_rng(seed)
    # Open fracture sheets localized to the interior block.
    for _ in range(7):
        n=rng.normal(size=3);n/=np.linalg.norm(n)
        off=rng.uniform(-.38,.38);width=rng.uniform(.022,.050)
        plane=np.abs(x*n[0]+y*n[1]+z*n[2]-off)
        local=np.exp(-((x-rng.uniform(-.4,.4))**2+(z-rng.uniform(-.5,.5))**2)/rng.uniform(.45,.95))
        d=np.maximum(d,(width-plane)*(.65+.55*local))
    # Surface chips/pits, concentrated toward the front/side faces.
    for _ in range(18):
        c0=np.array([rng.uniform(-.82,.82),rng.uniform(-.58,.58),rng.uniform(-.95,.95)])
        r=rng.uniform(.055,.16)
        cav=np.sqrt((x-c0[0])**2+(y-c0[1])**2+(z-c0[2])**2)-r
        d=np.maximum(d,-cav)
    image=vtk.vtkImageData();image.SetDimensions(nx,ny,nz)
    image.SetOrigin(float(xs[0]),float(ys[0]),float(zs[0]))
    image.SetSpacing(float(xs[1]-xs[0]),float(ys[1]-ys[0]),float(zs[1]-zs[0]))
    image.GetPointData().SetScalars(numpy_to_vtk(d.astype(np.float32).ravel(order="F"),deep=True))
    contour=vtk.vtkFlyingEdges3D();contour.SetInputData(image);contour.SetValue(0,0.);contour.ComputeNormalsOff();contour.Update()
    poly=contour.GetOutput()
    if poly.GetNumberOfPoints()==0:raise ValueError("Empty fractured block")
    v=vtk_to_numpy(poly.GetPoints().GetData()).copy()
    f=vtk_to_numpy(poly.GetPolys().GetConnectivityArray()).reshape(-1,3).copy()
    signed=np.einsum("ij,ij->i",v[f[:,0]],np.cross(v[f[:,1]],v[f[:,2]])).sum()/6
    if signed<0:f=f[:,[0,2,1]]
    return v,f,mesh_normals(v,f)


def transform_mesh(mesh,scale=(1,1,1),translate=(0,0,0),az=0,tilt=0):
    v,f,n=mesh
    scale=np.asarray(scale,float)
    ca,sa=math.cos(az),math.sin(az);ct,st=math.cos(tilt),math.sin(tilt)
    rz=np.array([[ca,-sa,0],[sa,ca,0],[0,0,1]],float)
    rx=np.array([[1,0,0],[0,ct,-st],[0,st,ct]],float)
    R=rx@rz
    vv=(v*scale)@R.T+np.asarray(translate,float)
    nn=(n/np.maximum(scale,1e-9))@R.T;nn/=np.maximum(np.linalg.norm(nn,axis=1)[:,None],1e-20)
    return vv,f.copy(),nn


def rock_field(parts,prefix,placements,material=9,seed=100,templates=8,resolution=48,group="environment",explode=(0,0,0)):
    cache=[implicit_rock_template(seed+i*31,resolution) for i in range(templates)]
    items=[]
    for i,p in enumerate(placements):
        x,y,z,sx,sy,sz,az,tilt=p
        items.append(transform_mesh(cache[i%templates],(sx,sy,sz),(x,y,z),az,tilt))
    if items:
        v,f,n=join_mesh(items)
        add_mesh(parts,prefix,v,f,n,material,group,"Closed non-convex implicit rock field",explode,tags=("implicit-surface","actual-mesogeometry"))


def tree_mesh(seed,base=(0,0,0),height=260,radius=13,levels=3,leaf=True,leaf_radius=24):
    rng=np.random.default_rng(seed);wood=[];fol=[]
    base=np.asarray(base,float)
    def branch(a,b,r,level):
        a=np.asarray(a,float);b=np.asarray(b,float);axis=b-a;L=np.linalg.norm(axis)
        if L<2:return
        side=rng.normal(size=3);side[2]*=.35;side/=max(np.linalg.norm(side),1e-9)
        pts=np.array([a,a+axis*.26+side*L*.045,a+axis*.55-side*L*.035,a+axis*.80+side*L*.025,b])
        v,f=tube_mesh(pts,max(r,1.0),sides=9)
        wood.append((v,f,mesh_normals(v,f)))
        if level<levels:
            count=4 if level==0 else 3
            for k in range(count):
                t=.36+.53*(k+1)/(count+1)
                idx=min(3,int(t*4));attach=pts[idx]
                phi=rng.uniform(0,math.tau)
                spread=(.45 if level==0 else .34)*L
                rise=(.42 if level==0 else .30)*L
                end=attach+np.array([math.cos(phi)*spread,math.sin(phi)*spread,rise])*rng.uniform(.72,1.08)
                branch(attach,end,r*.56,level+1)
        elif leaf:
            # Hundreds of actual pointed leaves per tree rather than sphere
            # clumps. Each leaf is a shallow ridged 3D blade with its own
            # orientation, attached around a terminal twig.
            for j in range(13):
                outward=rng.normal(size=3);outward[2]=abs(outward[2])*.62+.18
                outward/=max(np.linalg.norm(outward),1e-9)
                center=b+rng.normal(0,leaf_radius*.38,3)+outward*leaf_radius*rng.uniform(.05,.48)
                L=leaf_radius*rng.uniform(.42,.72)
                W=L*rng.uniform(.24,.38)
                ref=np.array([0.,0.,1.]) if abs(outward[2])<.86 else np.array([0.,1.,0.])
                sidev=np.cross(outward,ref);sidev/=max(np.linalg.norm(sidev),1e-9)
                ridge=np.cross(sidev,outward);ridge/=max(np.linalg.norm(ridge),1e-9)
                basep=center-outward*L*.46
                tipp=center+outward*L*.54
                mid=center+ridge*rng.uniform(-.08,.08)*L
                vv=np.array([
                    basep,
                    mid+sidev*W*.50,
                    tipp,
                    mid-sidev*W*.50,
                    mid+ridge*W*.14,
                    mid-ridge*W*.10,
                ])
                ff=np.array([
                    [0,1,4],[1,2,4],[2,3,4],[3,0,4],
                    [1,0,5],[2,1,5],[3,2,5],[0,3,5],
                ],dtype=np.int64)
                fol.append((vv,ff,mesh_normals(vv,ff)))
    top=base+np.array([rng.uniform(-.04,.04)*height,rng.uniform(-.04,.04)*height,height])
    branch(base,top,radius,0)
    w=join_mesh(wood);l=join_mesh(fol) if fol else (np.zeros((0,3)),np.zeros((0,3),np.int64),np.zeros((0,3)))
    return w,l


def shrub_mesh(seed,base=(0,0,0),height=70):
    rng=np.random.default_rng(seed);wood=[];leaves=[]
    base=np.asarray(base,float)
    for j in range(rng.integers(6,10)):
        phi=rng.uniform(0,math.tau);end=base+np.array([math.cos(phi)*height*rng.uniform(.25,.55),math.sin(phi)*height*rng.uniform(.25,.55),height*rng.uniform(.55,1.0)])
        pts=np.array([base+np.r_[rng.normal(0,4,2),0],(base+end)/2+np.r_[rng.normal(0,5,2),height*.06],end])
        v,f=tube_mesh(pts,2.1,sides=6);wood.append((v,f,mesh_normals(v,f)))
        axis=end-pts[-2];axis/=max(np.linalg.norm(axis),1e-9)
        for k in range(7):
            t=(k+.4)/7;p=pts[-2]*(1-t)+end*t
            side=np.cross(axis,[0,0,1])
            if np.linalg.norm(side)<1e-5:side=np.array([1.,0,0])
            side/=np.linalg.norm(side);up=np.cross(side,axis)
            L=rng.uniform(8,15);W=L*rng.uniform(.22,.35)
            vv=np.array([p,p+axis*L*.5+side*W,p+axis*L,p+axis*L*.5-side*W,p+axis*L*.5+up*W*.20])
            ff=np.array([[0,1,4],[1,2,4],[2,3,4],[3,0,4]])
            leaves.append((vv,ff,mesh_normals(vv,ff)))
    return join_mesh(wood),join_mesh(leaves)


def ribbon_surface(centerline,width=55,segments=120,cross=14,waves=3.0):
    pts=np.asarray(centerline,float)
    # interpolate polyline by arc length
    d=np.linalg.norm(np.diff(pts,axis=0),axis=1);s=np.r_[0,np.cumsum(d)]
    ss=np.linspace(0,s[-1],segments)
    c=np.column_stack([np.interp(ss,s,pts[:,k]) for k in range(3)])
    tangent=np.gradient(c,axis=0);tangent/=np.maximum(np.linalg.norm(tangent,axis=1)[:,None],1e-9)
    side=np.cross(tangent,np.array([0,0,1.]))
    bad=np.linalg.norm(side,axis=1)<1e-5
    side[bad]=np.cross(tangent[bad],np.array([0,1.,0]))
    side/=np.maximum(np.linalg.norm(side,axis=1)[:,None],1e-9)
    us=np.linspace(-1,1,cross)
    v=(c[:,None,:]+side[:,None,:]*us[None,:,None]*width/2)
    v[:,:,2]+=waves*np.sin(np.linspace(0,8*math.pi,segments))[:,None]*(1-us[None,:]**2)
    v=v.reshape(-1,3)
    f=[]
    for i in range(segments-1):
        for j in range(cross-1):
            a=i*cross+j;b=a+1;c0=a+cross;d0=c0+1
            f.extend(((a,c0,d0),(a,d0,b)))
    f=np.asarray(f,np.int64)
    return v,f,mesh_normals(v,f)
