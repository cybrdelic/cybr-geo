"""Explicit skinning and rigid transforms. No anatomy or dynamics claims."""
import numpy as np
from .model import Mesh, Attribute


def skin(mesh,weights,transforms):
    """Linear-blend deformation from NxB weights and Bx4x4 rigid transforms."""
    w=mesh.attributes[weights].values if isinstance(weights,str) else np.asarray(weights,float)
    t=np.asarray(transforms,float)
    if w.ndim!=2 or w.shape[0]!=len(mesh.vertices) or t.shape!=(w.shape[1],4,4):
        raise ValueError('Skin weights/transforms have incompatible dimensions')
    if not np.isfinite(w).all() or not np.isfinite(t).all() or np.any(w<0) or not np.allclose(w.sum(axis=1),1,atol=1e-7):
        raise ValueError('Skin weights must be finite, nonnegative and normalized')
    if not np.allclose(t[:,3,:],[0,0,0,1],atol=1e-9): raise ValueError('Transforms must be affine')
    rotations=t[:,:3,:3]
    if not np.allclose(np.swapaxes(rotations,1,2)@rotations,np.eye(3),atol=1e-7) or np.any(np.linalg.det(rotations)<0):
        raise ValueError('Bone transforms must be rigid rotations and translations')
    points=np.column_stack((mesh.vertices,np.ones(len(w))))
    transformed=np.einsum('bij,nj->nbi',t,points)[:,:,:3]
    vertices=np.einsum('nb,nbi->ni',w,transformed)
    attributes=dict(mesh.attributes);attributes['skin_weights']=Attribute(w)
    return Mesh(vertices,mesh.faces,attributes,mesh.closed,mesh.connected)


def rotation(axis,angle_degrees,pivot=(0,0,0)):
    axis=np.asarray(axis,float);pivot=np.asarray(pivot,float)
    if axis.shape!=(3,) or pivot.shape!=(3,) or not np.isfinite(np.r_[axis,pivot,angle_degrees]).all() or np.linalg.norm(axis)<1e-12:
        raise ValueError('Rotation needs a finite nonzero axis and pivot')
    axis/=np.linalg.norm(axis);x,y,z=axis;t=np.deg2rad(angle_degrees)
    cross=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
    R=np.eye(3)*np.cos(t)+(1-np.cos(t))*np.outer(axis,axis)+np.sin(t)*cross
    result=np.eye(4);result[:3,:3]=R;result[:3,3]=pivot-R@pivot
    return result


def transform(mesh,matrix):
    """Rigid placement preserves authored regions and any existing skin bindings."""
    t=np.asarray(matrix,float)
    if t.shape!=(4,4) or not np.isfinite(t).all() or not np.allclose(t[3],[0,0,0,1]) or not np.allclose(t[:3,:3].T@t[:3,:3],np.eye(3),atol=1e-7) or np.linalg.det(t[:3,:3])<0:
        raise ValueError('Expected a finite proper rigid transform')
    return Mesh(mesh.vertices@t[:3,:3].T+t[:3,3],mesh.faces,mesh.attributes,mesh.closed,mesh.connected)
