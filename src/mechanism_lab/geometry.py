"""Shared CAD and mesh primitives. No model-specific output paths or renderer state."""
from __future__ import annotations
import math
import numpy as np
import cadquery as cq
from .core import cad_part, mesh_part, rotation_x

def ring(ro,ri,x0,x1):
    q=cq.Workplane('YZ').circle(ro)
    if ri>0:q=q.circle(ri)
    return q.extrude(x1-x0).translate((x0,0,0)).val()

def bolt_circle(radius,count=4,phase=0):
    return [(radius*math.cos(phase+2*math.pi*i/count),radius*math.sin(phase+2*math.pi*i/count)) for i in range(count)]

def drill(shape, points, radius, x0,x1):
    holes=cq.Workplane('YZ').pushPoints(points).circle(radius).extrude(x1-x0).translate((x0,0,0))
    return cq.Workplane(obj=shape).cut(holes).val()

def sector(ro,ri,x0,x1,a0,a1):
    p=lambda r,a:(r*math.cos(a),r*math.sin(a))
    w=cq.Workplane('YZ').moveTo(*p(ro,a0)).threePointArc(p(ro,(a0+a1)/2),p(ro,a1)).lineTo(*p(ri,a1)).threePointArc(p(ri,(a0+a1)/2),p(ri,a0)).close()
    return w.extrude(x1-x0).translate((x0,0,0)).val()

def fillet(shape,r=.3):
    # Deliberately don't hide CAD errors: a failed cosmetic fillet is recorded by caller.
    return cq.Workplane(obj=shape).edges().fillet(r).val()

def tube_mesh(points,radius=.3,sides=8,closed=False):
    """Parallel-transport tube, with flat end caps for open cables/coils."""
    pts=np.asarray(points,float);n=len(pts)
    tang=np.gradient(pts,axis=0);tang/=np.linalg.norm(tang,axis=1)[:,None]
    normals=np.zeros_like(tang)
    ref=np.array([1.,0,0]) if abs(tang[0,0])<.8 else np.array([0.,0,1.])
    normals[0]=np.cross(tang[0],ref);normals[0]/=np.linalg.norm(normals[0])
    for i in range(1,n):
        v=normals[i-1]-tang[i]*np.dot(normals[i-1],tang[i]);l=np.linalg.norm(v)
        if l<1e-8:v=np.cross(tang[i],ref);l=np.linalg.norm(v)
        normals[i]=v/l
    binorm=np.cross(tang,normals);angle=np.arange(sides)*2*np.pi/sides
    vertices=(pts[:,None,:]+radius*(normals[:,None,:]*np.cos(angle)[None,:,None]+binorm[:,None,:]*np.sin(angle)[None,:,None])).reshape(-1,3)
    faces=[]
    for i in range(n-1):
        for j in range(sides):
            a=i*sides+j;b=i*sides+(j+1)%sides;c=b+sides;d=a+sides;faces.extend([[a,b,c],[a,c,d]])
    vertices=np.vstack([vertices,pts[0],pts[-1]])
    for j in range(sides):faces.extend([[n*sides,(j+1)%sides,j],[n*sides+1,(n-1)*sides+j,(n-1)*sides+(j+1)%sides]])
    return vertices,np.asarray(faces)

def winding(name,angle,material,turns=14):
    # Representative winding envelope. Actual winding schedule is not published.
    t=np.linspace(0,2*np.pi*turns,turns*48+1)
    x=22.5+15.0*np.sign(np.cos(t))*np.abs(np.cos(t))**.28
    z=2.22*np.sign(np.sin(t))*np.abs(np.sin(t))**.55
    y=28.7+9.4*t/(2*np.pi*turns)
    pts=np.stack([x,y,z],axis=1)@rotation_x(angle).T
    v,f=tube_mesh(pts,.255,8)
    return mesh_part(name,v,f,material,group='windings',motion='fixed',
                     explode=np.array([-30,14*math.cos(angle),14*math.sin(angle)]),
                     role='Representative wound copper tooth coil; turns and dimensions assumed',
                     provenance='inferred-internal')

def cable(name,control_points,material,radius=1.7):
    from scipy.interpolate import CubicSpline
    p=np.asarray(control_points,float);t=np.r_[0,np.cumsum(np.linalg.norm(np.diff(p,axis=0),axis=1))]
    tt=np.linspace(0,t[-1],120);pts=CubicSpline(t,p,bc_type='natural')(tt)
    v,f=tube_mesh(pts,radius,12)
    return mesh_part(name,v,f,material,group='cables',role='Photo-estimated cable routing; connector interface unqualified',provenance='photo-estimate')

def mesh_label(text,x0=10.,theta=-.65,radius=46.06,size=2.0,material=0):
    import vtk
    from vtk.util.numpy_support import vtk_to_numpy
    vt=vtk.vtkVectorText();vt.SetText(text);vt.Update();pd=vt.GetOutput()
    v=vtk_to_numpy(pd.GetPoints().GetData()).copy().astype(float)*size
    # Text horizontal direction follows the circumference; vertical follows X.
    a=theta+v[:,0]/radius
    vv=np.column_stack([x0+v[:,1],radius*np.cos(a),radius*np.sin(a)])
    ff=vtk_to_numpy(pd.GetPolys().GetConnectivityArray()).reshape(-1,3)
    return mesh_part('Identification_marking',vv,ff,material,group='markings',motion='rotor',provenance='illustrative-label',role='Identification text; not a manufacturer logo')
