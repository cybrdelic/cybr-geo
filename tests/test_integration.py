"""Cross-module checks: new domains cannot silently be treated as vertex data."""
import unittest
import tempfile
from pathlib import Path
import numpy as np
from cybrgeo import (Mesh, Attribute, Editable, MeshBVH, extrude, subdivide,
                     export_obj, local_displace, crease_subdivide)

class IntegrationTests(unittest.TestCase):
    def test_corner_obj_indices_and_rejection(self):
        m=extrude([(-1,-1),(1,-1),(1,1),(-1,1)],2)
        m=Mesh(m.vertices,m.faces,{'uv':Attribute(np.zeros((3*len(m.faces),2)),'corner')})
        with tempfile.TemporaryDirectory() as d:
            p=export_obj(m,Path(d)/'mesh.obj')
            lines=p.read_text().splitlines();fs=[line for line in lines if line.startswith('f ')]
            self.assertEqual(fs[1].split()[1].split('/')[1],'4')
        for operation in (subdivide,crease_subdivide):
            with self.assertRaisesRegex(ValueError,'attributes|transfer'):operation(m)
    def test_edit_query_local_chain(self):
        m=extrude([(-1,-1),(1,-1),(1,1),(-1,1)],2)
        top=np.where(np.all(m.vertices[m.faces][...,2]==2,axis=1))[0]
        mesh=Editable(m).inset_faces(top,.25).extrude_faces(top,[0,0,1]).mesh
        hit=MeshBVH(mesh).closest_point([0,0,4])
        self.assertAlmostEqual(hit.distance,1)
        high=np.where(mesh.vertices[:,2]>2)[0]
        edit=local_displace(mesh,high,[0,0,.5])
        self.assertAlmostEqual(MeshBVH(edit.mesh).closest_point([0,0,4]).distance,.5)
