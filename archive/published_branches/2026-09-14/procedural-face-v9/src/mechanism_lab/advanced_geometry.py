"""Advanced geometry helpers retained from CYBR GEO's morphic-wrist branch.

Source: cybrdelic/cybr-geo, e99df0dd87bdda2a176850e94a57a948d6161243,
original advanced_geometry.py blob 8f4d4336fce111eee23e6648fd365f7d78a082d7.
BREP helpers return CadQuery/OpenCascade shapes; the gyroid is intentionally
an implicit triangle mesh, not an analytic CAD solid. GPL-2.0, like upstream.
"""
from __future__ import annotations
from typing import Sequence
import cadquery as cq


def lofted_elliptic_shell(sections: Sequence[tuple[float,float,float,float,float]], wall: float):
    """Hollow multisection BREP loft along X: x, y/z radii, y/z offsets."""
    if len(sections)<2 or wall<=0:
        raise ValueError('Need >=2 sections and a positive wall')
    def make(shrink):
        x0,ry,rz,oy,oz=sections[0]
        if min(ry-shrink,rz-shrink)<=0: raise ValueError('Collapsed wall')
        wp=cq.Workplane('YZ',origin=(x0,oy,oz)).ellipse(ry-shrink,rz-shrink)
        px,py,pz=x0,oy,oz
        for x,ry,rz,oy,oz in sections[1:]:
            if min(ry-shrink,rz-shrink)<=0: raise ValueError('Collapsed wall')
            wp=wp.workplane(offset=x-px).center(oy-py,oz-pz).ellipse(ry-shrink,rz-shrink)
            px,py,pz=x,oy,oz
        return wp.loft(combine=True,ruled=False).val()
    return make(0.).cut(make(wall))


def lofted_solid(sections):
    """Asymmetric multisection elliptic BREP loft along X."""
    if len(sections)<2: raise ValueError('Need >=2 sections')
    x0,ry,rz,oy,oz=sections[0]
    wp=cq.Workplane('YZ',origin=(x0,oy,oz)).ellipse(ry,rz)
    px,py,pz=x0,oy,oz
    for x,ry,rz,oy,oz in sections[1:]:
        wp=wp.workplane(offset=x-px).center(oy-py,oz-pz).ellipse(ry,rz)
        px,py,pz=x,oy,oz
    return wp.loft(combine=True,ruled=False).val()


def spline_sweep_tube(points,radius):
    """Analytic circular sweep along an interpolating 3D spline."""
    if len(points)<3 or radius<=0: raise ValueError('Invalid spline sweep')
    v=[cq.Vector(*p) for p in points]
    tangent=v[1]-v[0]
    if tangent.Length<1e-9: raise ValueError('Degenerate first segment')
    wire=cq.Wire.assembleEdges([cq.Edge.makeSpline(v)])
    return cq.Workplane(cq.Plane(origin=v[0],normal=tangent)).circle(radius).sweep(wire,isFrenet=True,transition='round').val()


def helical_sweep(*,x0,length,helix_radius,pitch,section_radius,lefthand=False):
    """Analytic round-section helix sweep along X."""
    if min(length,helix_radius,pitch,section_radius)<=0: raise ValueError('Invalid helix')
    helix=cq.Wire.makeHelix(pitch,length,helix_radius,center=cq.Vector(x0,0,0),dir=cq.Vector(1,0,0),lefthand=lefthand)
    return cq.Workplane('XY',origin=(x0,helix_radius,0)).circle(section_radius).sweep(helix,isFrenet=True,transition='round').val()


def drafted_cylinder(x0,length,radius,taper_deg):
    if min(length,radius)<=0: raise ValueError('Invalid drafted extrusion')
    return cq.Workplane('YZ',origin=(x0,0,0)).circle(radius).extrude(length,taper=taper_deg).val()


def strut(p0,p1,radius):
    a,b=cq.Vector(*p0),cq.Vector(*p1);d=b-a
    if d.Length<=1e-9 or radius<=0: raise ValueError('Invalid strut')
    return cq.Solid.makeCylinder(radius,d.Length,a,d.normalized())


def bcc_lattice(*,origin,cells,pitch,strut_radius,node_radius=None):
    nx,ny,nz=cells;sx,sy,sz=pitch
    if min(nx,ny,nz)<1 or min(sx,sy,sz,strut_radius)<=0: raise ValueError('Invalid lattice')
    if node_radius is None: node_radius=strut_radius*1.25
    ox,oy,oz=origin;solids=[];nodes=set()
    for i in range(nx):
        for j in range(ny):
            for k in range(nz):
                corners=[(ox+(i+dx)*sx,oy+(j+dy)*sy,oz+(k+dz)*sz) for dx in (0,1) for dy in (0,1) for dz in (0,1)]
                center=(ox+(i+.5)*sx,oy+(j+.5)*sy,oz+(k+.5)*sz)
                nodes.add(center);nodes.update(corners)
                solids.extend(strut(center,c,strut_radius) for c in corners)
    solids.extend(cq.Solid.makeSphere(node_radius,cq.Vector(*p)) for p in nodes)
    return cq.Compound.makeCompound(solids)


def toroidal_groove(major_radius,minor_radius,center=(0,0,0),axis=(1,0,0)):
    if min(major_radius,minor_radius)<=0: raise ValueError('Invalid torus')
    return cq.Solid.makeTorus(major_radius,minor_radius,cq.Vector(*center),cq.Vector(*axis))


def gyroid_sheet_mesh(bounds,resolution=(42,30,30),periods=(1.4,1.2,1.2),thickness=.26):
    """Closed sampled TPMS sheet extracted with VTK FlyingEdges."""
    import numpy as np
    import vtk
    from vtk.util.numpy_support import numpy_to_vtk,vtk_to_numpy
    xmin,xmax,ymin,ymax,zmin,zmax=bounds;nx,ny,nz=resolution
    if min(nx,ny,nz)<8 or min(xmax-xmin,ymax-ymin,zmax-zmin)<=0: raise ValueError('Invalid implicit grid')
    X,Y,Z=np.meshgrid(np.linspace(xmin,xmax,nx),np.linspace(ymin,ymax,ny),np.linspace(zmin,zmax,nz),indexing='ij')
    px=(X-xmin)/(xmax-xmin)*2*np.pi*periods[0]
    py=(Y-ymin)/(ymax-ymin)*2*np.pi*periods[1]
    pz=(Z-zmin)/(zmax-zmin)*2*np.pi*periods[2]
    g=np.sin(px)*np.cos(py)+np.sin(py)*np.cos(pz)+np.sin(pz)*np.cos(px)
    field=np.abs(g)-thickness;outside=float(np.max(field)+1.)
    field[[0,-1],:,:]=outside;field[:,[0,-1],:]=outside;field[:,:,[0,-1]]=outside
    image=vtk.vtkImageData();image.SetDimensions(nx,ny,nz);image.SetOrigin(xmin,ymin,zmin)
    image.SetSpacing((xmax-xmin)/(nx-1),(ymax-ymin)/(ny-1),(zmax-zmin)/(nz-1))
    image.GetPointData().SetScalars(numpy_to_vtk(field.astype(np.float32).ravel(order='F'),deep=True))
    contour=vtk.vtkFlyingEdges3D();contour.SetInputData(image);contour.SetValue(0,0.);contour.ComputeNormalsOff();contour.Update()
    poly=contour.GetOutput()
    if poly.GetNumberOfPoints()==0: raise ValueError('Empty implicit surface')
    return vtk_to_numpy(poly.GetPoints().GetData()).copy(),vtk_to_numpy(poly.GetPolys().GetConnectivityArray()).reshape(-1,3).copy()
