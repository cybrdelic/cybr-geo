"""Small runnable LOD example; explicit budget, measured result, no magic factory."""
import numpy as np
from cybrgeo import heightfield, simplify, SurfaceAnchor, export_obj

axis=np.linspace(-1,1,5)
x,y=np.meshgrid(axis,axis,indexing='ij')
source=heightfield(axis,axis,.12*np.cos(x*2)*np.cos(y*2))
anchor=SurfaceAnchor.on(source,12,(.2,.3,.5))
result=simplify(source,target_faces=24,max_error=.05)
rebound=result.rebind(anchor,source,max_distance=.05)
if __name__=='__main__':
    export_obj(result.mesh,'lod_surface.obj')
    print('faces:',len(source.faces),'->',len(result.mesh.faces))
    print('sampled error:',result.sampled_error,'target reached:',result.reached_target)
    print('rebound surface point:',rebound.point(result.mesh))
