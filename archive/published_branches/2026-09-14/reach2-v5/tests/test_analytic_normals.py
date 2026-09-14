"""Surface-derived normals must preserve curved interiors and real hard edges."""
import cadquery as cq
import numpy as np
from mechanism_lab.core import cad_part


def test_located_sphere_normals_are_outward():
    center=np.array([14.,-8.,29.])
    shape=cq.Solid.makeSphere(5.,angleDegrees1=-90).rotate((0,0,0),(1,2,3),37).translate(tuple(center))
    part=cad_part('sphere',shape,analytic_normals=True)
    expected=(part.vertices-center)/5.
    assert np.max(np.linalg.norm(part.normals-expected,axis=1))<1e-6
    points=part.vertices[part.faces]
    winding=np.cross(points[:,1]-points[:,0],points[:,2]-points[:,0])
    assert np.min(np.einsum('ij,ij->i',winding,points.mean(axis=1)-center))>-1e-10


def test_cylindrical_bore_normals_face_the_hole():
    shape=cq.Workplane('XY').circle(7).circle(3).extrude(12).val()
    part=cad_part('bore',shape,analytic_normals=True)
    radial=np.linalg.norm(part.vertices[:,:2],axis=1)
    side=np.abs(part.normals[:,2])<.01
    inner=side&(np.abs(radial-3)<1e-5)
    outer=side&(np.abs(radial-7)<1e-5)
    assert inner.sum()>20 and outer.sum()>20
    dots=np.einsum('ij,ij->i',part.vertices[:,:2],part.normals[:,:2])
    assert np.allclose(dots[inner],-3,atol=1e-5)
    assert np.allclose(dots[outer],7,atol=1e-5)
    # End faces retain their planar normals at the exact same rim positions.
    assert np.count_nonzero(np.abs(part.normals[:,2])>.999)>20


def test_moving_geometry_moves_authored_finish_origin():
    p=cad_part('box',cq.Workplane('XY').box(2,3,4),analytic_normals=True,finish_origin=(1.,2.,3.))
    q=p.moved((10,-3,2))
    assert q.finish_origin==(11.,-1.,5.)
    assert np.allclose(np.sort(np.abs(q.normals),axis=1),[0,0,1])
