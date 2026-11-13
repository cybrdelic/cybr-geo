import unittest
import numpy as np
from cybrgeo import Solid, Mesh, Attribute
from cybrgeo.resolution import refine_to_size


def longest(mesh):
    p=mesh.vertices[mesh.faces]
    return np.linalg.norm(p-np.roll(p,-1,axis=1),axis=2).max()


class ResolutionTests(unittest.TestCase):
    def test_achieves_bound_and_preserves_shape(self):
        mesh=Solid.box((10,6,2)).mesh()
        refined=refine_to_size(mesh,1.5)
        self.assertLessEqual(longest(refined),1.5*(1+1e-12))
        np.testing.assert_allclose(refined.vertices.min(0),mesh.vertices.min(0))
        np.testing.assert_allclose(refined.vertices.max(0),mesh.vertices.max(0))
        self.assertTrue(refined.closed)

    def test_explicit_limits(self):
        mesh=Solid.box((10,6,2)).mesh()
        with self.assertRaisesRegex(ValueError,'max_passes'):
            refine_to_size(mesh,.01,max_passes=1)
        with self.assertRaisesRegex(ValueError,'max_vertices'):
            refine_to_size(mesh,.01,max_vertices=10)

    def test_noop_preserves_identity(self):
        mesh=Solid.box((1,1,1)).mesh()
        self.assertIs(refine_to_size(mesh,2),mesh)

    def test_open_mesh_and_attributes(self):
        v=np.array([[0.,0.,0.],[3.,0.,0.],[0.,2.,0.]])
        f=np.array([[0,1,2]])
        attrs={'position':Attribute(v),'id':Attribute(np.array([7]),'face','nearest'),'uv':Attribute(v[:,:2],'corner')}
        result=refine_to_size(Mesh(v,f,attrs,False),.6)
        self.assertFalse(result.closed)
        np.testing.assert_allclose(result.attributes['position'].values,result.vertices)
        np.testing.assert_array_equal(result.attributes['id'].values,np.full(len(result.faces),7))
        np.testing.assert_allclose(result.attributes['uv'].values,result.vertices[result.faces].reshape(-1,3)[:,:2])

    def test_invalid_limits(self):
        mesh=Solid.box((1,1,1)).mesh()
        for size in [0,-1,float('nan'),float('inf')]:
            with self.assertRaises(ValueError):refine_to_size(mesh,size)
        for passes in [-1,True,1.2]:
            with self.assertRaises(ValueError):refine_to_size(mesh,1,max_passes=passes)
        for count in [2,True,1.2]:
            with self.assertRaises(ValueError):refine_to_size(mesh,1,max_vertices=count)

if __name__=='__main__':unittest.main()
