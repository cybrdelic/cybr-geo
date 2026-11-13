import numpy as np
from unittest import TestCase
from cybrgeo import Mesh, heightfield, SurfaceAnchor, refine
from cybrgeo import Correspondence, CorrespondenceError, project_reference
x = np.linspace(-1.4, 1.4, 13)
y = np.linspace(-1., 1., 11)
xx, yy = np.meshgrid(x, y, indexing="ij")
z = .4*(1-xx**2/2)+.12*np.cos(3*yy)
panel = heightfield(x, y, z)
anchor = SurfaceAnchor.on(panel, 117, (.2, .3, .5))
refined = refine(panel, .2)
tracked = Correspondence.refinement(panel, refined).follow(anchor)
positions = refined.mesh.vertices.copy()
positions[:, 2] += .3*positions[:, 0]
curved = Mesh(positions, refined.mesh.faces, closed=False)
tracked = Correspondence.deformation(refined.mesh, curved).follow(tracked)
outer = Mesh(curved.vertices + (0,0,.18), curved.faces, closed=False)
insert = Mesh(curved.vertices + (0,0,.06), curved.faces, closed=False)
vertices = np.vstack((insert.vertices, outer.vertices))
faces = np.vstack((insert.faces, outer.faces+len(insert.vertices)))
replacement = Mesh(vertices, faces, closed=False, connected=False)
check = TestCase()
with check.assertRaises(CorrespondenceError): project_reference(tracked, curved, replacement, .3)
old_regions = np.full(len(curved.faces), 'outer')
new_regions = np.array(['insert']*len(insert.faces)+['outer']*len(outer.faces))
resolved = project_reference(tracked, curved, replacement, .3, source_regions=old_regions, target_regions=new_regions)
assert resolved.anchor.face >= len(insert.faces)
assert resolved.confidence == 'conditional-projection'
