import unittest
import numpy as np
from cybrgeo import Mesh,Attribute,heightfield,simplify,sampled_surface_error,SurfaceAnchor

class LODTests(unittest.TestCase):
    def plane(self,n=4):
        x=np.linspace(0,1,n)
        return heightfield(x,x,np.zeros((n,n)))

    def test_planar_reduction_and_boundaries(self):
        m=self.plane();r=simplify(m,14,1e-8)
        self.assertTrue(r.reached_target);self.assertEqual(len(r.mesh.faces),14)
        self.assertLess(r.sampled_error,1e-8)
        boundary=np.flatnonzero(np.any((m.vertices[:,:2]==0)|(m.vertices[:,:2]==1),axis=1))
        for i in boundary:self.assertTrue(any(np.array_equal(m.vertices[i],p) for p in r.mesh.vertices))
        self.assertEqual(r.mesh.report()['nonmanifold_edges'],0)
        np.testing.assert_array_equal(m.vertices[:,2],0)

    def test_attributes_and_provenance(self):
        m=self.plane();m=Mesh(m.vertices,m.faces,{'value':Attribute(m.vertices[:,0]+2*m.vertices[:,1]),'face':Attribute(np.arange(len(m.faces)),'face','nearest')},False)
        r=simplify(m,16,.001)
        np.testing.assert_allclose(r.mesh.attributes['value'].values,r.mesh.vertices[:,0]+2*r.mesh.vertices[:,1])
        np.testing.assert_array_equal(r.mesh.attributes['face'].values,r.source_face_indices)
        flattened=[i for group in r.source_vertex_groups for i in group]
        self.assertEqual(sorted(flattened),list(range(len(m.vertices))))
        a=SurfaceAnchor.on(m,8,[.2,.3,.5]);b=r.rebind(a,m,.001)
        np.testing.assert_allclose(a.point(m),b.point(r.mesh),atol=1e-12)

    def test_error_budget_rejects_curvature_loss(self):
        x=np.linspace(-1,1,4);xx,yy=np.meshgrid(x,x,indexing='ij')
        m=heightfield(x,x,.3*(xx*xx+yy*yy))
        r=simplify(m,14,1e-8)
        self.assertFalse(r.reached_target);self.assertEqual(r.collapses,0)
        loose=simplify(m,16,.2)
        self.assertTrue(loose.reached_target)
        self.assertLessEqual(sampled_surface_error(m,loose.mesh),.2)

    def test_pins_and_unsupported_data(self):
        m=self.plane();r=simplify(m,2,.1,protected_vertices=range(len(m.vertices)))
        self.assertEqual(r.collapses,0);self.assertFalse(r.reached_target)
        uv=Attribute(m.vertices[m.faces].reshape(-1,3)[:,:2],'corner')
        s=Mesh(m.vertices,m.faces,{'uv':uv},False)
        with self.assertRaises(ValueError):simplify(s,10,.1)
        for error in (-1,np.inf,np.nan):
            with self.assertRaises(ValueError):simplify(m,10,error)

    def test_closed_mesh_and_noop(self):
        from cybrgeo import crease_subdivide
        v=np.array([[0,0,0],[1,0,0],[0,1,0],[0,0,1]],float)
        base=Mesh(v,np.array([[0,2,1],[0,1,3],[1,2,3],[2,0,3]]))
        m=crease_subdivide(base,smooth=False)
        r=simplify(m,14,.2)
        self.assertTrue(r.reached_target)
        self.assertEqual(r.mesh.report()['boundary_edges'],0)
        unchanged=simplify(m,100,0)
        self.assertIs(unchanged.mesh,m)
