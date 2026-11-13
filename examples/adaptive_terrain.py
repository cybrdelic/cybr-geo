"""Local terrain refinement uses the same generic operations as any surface."""
import numpy as np
from cybrgeo import heightfield, refine, SurfaceAnchor, export_obj

def sample(points):
    result=points.copy()
    result[:,2]=.18*np.exp(-((points[:,0]-.55)**2+(points[:,1]-.5)**2)/.025)
    return result

axis=np.linspace(0,1,9)
x,y=np.meshgrid(axis,axis,indexing='ij')
points=np.stack((x,y,np.zeros_like(x)),axis=-1)
mesh=heightfield(axis,axis,sample(points.reshape(-1,3))[:,2].reshape(x.shape))
anchor=SurfaceAnchor.on(mesh,60,(.2,.4,.4))
for size in (.08,.04,.02):
    centers=mesh.vertices[mesh.faces].mean(axis=1)
    selected=np.flatnonzero(np.linalg.norm(centers[:,:2]-[.55,.5],axis=1)<.25)
    step=refine(mesh,size,selected,evaluate=sample)
    anchor=step.follow(anchor)
    mesh=step.mesh
if __name__=='__main__':
    export_obj(mesh,'adaptive_terrain.obj')
    print(mesh.report(), 'material anchor:',anchor.point(mesh))
