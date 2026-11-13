import tempfile
from pathlib import Path
import unittest
import numpy as np
from cybrgeo import *

class CoreChecks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sphere=field(lambda p:np.linalg.norm(p,axis=1)-1,((-1.3,)*3,(1.3,)*3),16)

    def test_field_closed_connected_and_root_refinement(self):
        self.assertEqual(self.sphere.report()['components'],1)
        self.assertEqual(self.sphere.report()['boundary_edges'],0)
        self.assertLess(np.max(np.abs(np.linalg.norm(self.sphere.vertices,axis=1)-1)),4e-5)

    def test_positive_field_rescaling_does_not_move_roots(self):
        other=field(lambda p:(np.linalg.norm(p,axis=1)-1)*np.exp(2*p[:,0]),((-1.3,)*3,(1.3,)*3),16)
        np.testing.assert_array_equal(self.sphere.vertices,other.vertices)

    def test_disconnected_geometry_is_rejected(self):
        with self.assertRaisesRegex(ValueError,'connected'):
            field(lambda p:np.minimum(np.linalg.norm(p-[-1,0,0],axis=1),np.linalg.norm(p-[1,0,0],axis=1))-.4,((-2,-1,-1),(2,1,1)),18)

    def test_clipped_solid_is_rejected(self):
        with self.assertRaisesRegex(ValueError,'boundary'):
            field(lambda p:np.linalg.norm(p,axis=1)-2,((-1,)*3,(1,)*3),12)

    def test_vertex_only_connection_is_rejected(self):
        with self.assertRaisesRegex(ValueError,'topology'):
            Mesh(np.array([[0,0,0],[1,0,0],[0,1,0],[-1,0,0],[0,-1,0]]),np.array([[0,1,2],[0,3,4]]),closed=False)

    def test_region_sculpt_and_subdivision_transfer(self):
        m=region(self.sphere,'upper',lambda p:np.clip(p[:,2],0,1))
        changed=sculpt(m,[0,0,.2],region='upper')
        weights=m.attributes['region:upper'].values
        np.testing.assert_allclose(changed.vertices-self.sphere.vertices,weights[:,None]*[0,0,.2],atol=1e-12)
        refined=subdivide(changed)
        self.assertEqual(len(refined.faces),4*len(m.faces))
        w=refined.attributes['region:upper'].values
        self.assertTrue(np.all((w>=0)&(w<=1)))
        self.assertEqual(refined.report()['components'],1)

    def test_skin_identity_and_transferred_bindings(self):
        w=np.column_stack((np.full(len(self.sphere.vertices),.4),np.full(len(self.sphere.vertices),.6)))
        mesh=Mesh(self.sphere.vertices,self.sphere.faces,{'weights':Attribute(w)})
        mesh=subdivide(mesh)
        posed=skin(mesh,'weights',np.array([np.eye(4),np.eye(4)]))
        np.testing.assert_allclose(posed.vertices,mesh.vertices)
        np.testing.assert_allclose(mesh.attributes['weights'].values.sum(axis=1),1)

    def test_bad_skin_weights_fail(self):
        with self.assertRaises(ValueError):skin(self.sphere,np.zeros((len(self.sphere.vertices),2)),[np.eye(4),np.eye(4)])

    def test_concave_loft_and_heightfield(self):
        ring=np.array([[0,0],[2,0],[2,1],[1,1],[1,2],[0,2]])
        sections=np.array([np.column_stack((ring,np.full(6,z))) for z in (0,1)])
        self.assertEqual(loft(sections).report()['boundary_edges'],0)
        patch=heightfield(np.arange(4),np.arange(5),np.zeros((4,5)))
        self.assertEqual(patch.report()['boundary_edges'],14)
        self.assertEqual(patch.attributes['uv'].values.shape,(20,2))

    def test_snapshot_roundtrip_and_immutable_inputs(self):
        mesh=region(self.sphere,'upper',lambda p:np.clip(p[:,2],0,1))
        with tempfile.TemporaryDirectory() as directory:
            path=save_snapshot(mesh,Path(directory)/'mesh.npz');other=load_snapshot(path)
            np.testing.assert_array_equal(mesh.vertices,other.vertices)
            np.testing.assert_array_equal(mesh.attributes['region:upper'].values,other.attributes['region:upper'].values)
        with self.assertRaises(ValueError):mesh.vertices[0]=0

if __name__=='__main__':unittest.main()
