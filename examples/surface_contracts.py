"""Four-patch compound surface, explicit shared edges and a trimmed opening.
Not a car template. Run from the repository root: python -m examples.surface_contracts
"""
import numpy as np
from cybrgeo import BezierPatch, Seam, fit_network, trim_patch, export_obj
from pathlib import Path

def patch(x,y):
    return BezierPatch(np.array([[[x+i/3,y+j/3,.2*np.sin(x+i/3)+.15*(y+j/3)**2]
                                  for j in range(4)] for i in range(4)]))
patches={name:patch(x,y) for name,x,y in [('a',0,0),('b',1.1,0),('c',0,1.1),('d',1.1,1.1)]}
seams=[Seam('a','u1','b','u0',2),Seam('a','v1','c','v0',2),
       Seam('b','v1','d','v0',2),Seam('c','u1','d','u0',2)]
network=fit_network(patches,seams,fairness=.005)
out=Path('surface_contract_example');out.mkdir(exist_ok=True)
for name,mesh in network.meshes().items():export_obj(mesh,out/(name+'.obj'))
# An arbitrary concave boundary with a rectangular aperture in UV.
mesh=trim_patch(network.patches['a'],[[0,0],[1,0],[1,.7],[.8,.7],[.8,1],[0,1]],
                holes=[[[.25,.25],[.55,.25],[.55,.55],[.25,.55]]],spacing=.04)
export_obj(mesh,out/'a_trimmed.obj')
print(network.report())
try:network.with_patch('a',patch(10,10))
except ValueError as e:print('Rejected incompatible edit:',e)
