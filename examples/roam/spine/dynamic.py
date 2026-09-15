"""Viewer updates tessellated from the analytic, evaluated CAD shapes."""
import numpy as np
from model import cad_part
def dynamic_geometry(m,q):
 out=[];fs,_=m.poses(q)
 for p in m.parts:
  if not (p['name'].endswith('_gas_rod_envelope') or p['name'].endswith('_harness_route') or p['name'].endswith('_service_loop')):continue
  part=cad_part(p['name'],m.world_shape(p,q,fs),p['mat'],tolerance=.15,angular=.18)
  xyz=np.asarray(part.vertices);normal=np.asarray(part.normals)
  out.append({'name':p['name'],'positions':np.round(xyz[:,[0,2,1]]*np.array([.001,.001,-.001]),7).ravel().tolist(),'normals':np.round(normal[:,[0,2,1]]*np.array([1,1,-1]),6).ravel().tolist(),'indices':np.asarray(part.faces).ravel().tolist()})
 from wiring import cached_pipe
 cached_pipe.cache_clear()
 return out
