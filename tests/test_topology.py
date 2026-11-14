import unittest
import numpy as np
from cybrgeo import Mesh, Attribute, extrude
from cybrgeo.model import mesh_edges,save_snapshot,load_snapshot
from cybrgeo.topology import Editable

class TopologyTests(unittest.TestCase):
    def cube(self):return extrude([(-1,-1),(1,-1),(1,1),(-1,1)],2)
    def test_split_identity_and_attributes(self):
        m=self.cube()
        attrs={'uv':Attribute(np.arange(3*len(m.faces)*2).reshape(-1,2),'corner'),
               'crease':Attribute(np.ones(len(mesh_edges(m.faces))),'edge','nearest'),
               'x':Attribute(m.vertices[:,0])}
        e=Editable(Mesh(m.vertices,m.faces,attrs));ref=e.ref('face',0);edge=mesh_edges(m.faces)[0]
        touched=np.flatnonzero(np.sum(np.isin(m.faces,edge),axis=1)==2); ref=e.ref('face',int(touched[0]))
        with self.assertRaises(ValueError):e.split_edge(*edge)
        r=e.split_edge(*edge,edge_defaults={'crease':0})
        self.assertEqual(len(r.mesh.faces),len(m.faces)+2)
        self.assertEqual(r.mesh.report()['boundary_edges'],0)
        self.assertEqual(len(r.descendants(ref)),2)
        with self.assertRaises(KeyError):r.resolve(ref)
        np.testing.assert_allclose(r.mesh.attributes['x'].values,r.mesh.vertices[:,0])
        self.assertEqual(r.mesh.attributes['uv'].values.shape,(3*len(r.mesh.faces),2))
        self.assertEqual(e.resolve(e.ref('vertex',0)),r.resolve(e.ref('vertex',0)))
        with self.assertRaises(ValueError):Editable(m).resolve(ref)
    def test_patch(self):
        m=self.cube(); top=np.where(np.all(m.vertices[m.faces,:,][...,2]==2,axis=1))[0]
        e=Editable(m);r=e.inset_faces(top,.3)
        self.assertEqual(r.mesh.report()['boundary_edges'],0)
        # Cap triangles remain first in output; subsequent extrusion uses new evaluated geometry.
        r=r.extrude_faces(top,[0,0,1])
        self.assertEqual(r.mesh.report()['boundary_edges'],0)
        self.assertAlmostEqual(r.mesh.vertices[:,2].max(),3)
        self.assertTrue(np.array_equal(e.mesh.vertices,m.vertices))
    def test_bridge(self):
        # Two separate outward oriented disks; connect their perimeter.
        v=np.array([[0,0,0],[1,0,0],[0,1,0],[0,0,1],[1,0,1],[0,1,1]])
        m=Mesh(v,np.array([[0,2,1],[3,4,5]]),closed=False,connected=False)
        e=Editable(m);loops=e.boundaries();r=e.bridge(*loops)
        self.assertEqual(r.mesh.report()['boundary_edges'],0)
        self.assertEqual(r.mesh.report()['components'],1)
    def test_weld(self):
        v=np.array([[0,0,0],[1,0,0],[1,1,0],[0,0,0],[1,1,0],[0,1,0]])
        e=Editable(Mesh(v,np.array([[0,1,2],[3,4,5]]),closed=False,connected=False))
        r=e.weld([(0,3),(2,4)])
        self.assertEqual(len(r.mesh.vertices),4)
        self.assertEqual(r.mesh.report()['components'],1)
        with self.assertRaises(ValueError):e.weld([(0,1)])
    def test_duplicate_vertex_sources_do_not_duplicate_identity(self):
        v=np.array([[0,0,0],[1,0,0],[0,1,0]],float)
        e=Editable(Mesh(v,[[0,1,2]],closed=False,connected=False))
        r=e._finish(np.concatenate((v,v)),[[0,1,2],[3,4,5]],
                    [{i:1.} for i in range(3)]*2,[0,0],[True,False])
        self.assertEqual(len(set(r._ids['vertex'])),6)
        for i in range(3):
            ref=e.ref('vertex',i)
            self.assertEqual(r.resolve(ref),i)
            self.assertEqual(r.descendants(ref),(i,i+3))
        np.testing.assert_array_equal(r.mesh.vertices,np.concatenate((v,v)))
        # Moving an exact-weight vertex must still replace its identity.
        moved=v.copy();moved[0,2]=1
        r=e._finish(moved,[[0,1,2]],[{i:1.} for i in range(3)],[0],[False])
        with self.assertRaises(KeyError):r.resolve(e.ref('vertex',0))
        self.assertEqual(r.descendants(e.ref('vertex',0)),(0,))

    def test_snapshot_domains(self):
        import tempfile, pathlib
        m=self.cube(); attrs={'uv':Attribute(np.zeros((3*len(m.faces),2)),'corner'), 'crease':Attribute(np.ones(len(mesh_edges(m.faces))),'edge')}
        m=Mesh(m.vertices,m.faces,attrs)
        with tempfile.TemporaryDirectory() as d:
            p=pathlib.Path(d)/'a.npz';save_snapshot(m,p);r=load_snapshot(p)
            self.assertEqual(r.attributes['uv'].domain,'corner')
            self.assertEqual(r.attributes['crease'].domain,'edge')

if __name__=='__main__': unittest.main()
