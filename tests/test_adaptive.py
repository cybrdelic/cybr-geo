import unittest
import tempfile
import numpy as np
from cybrgeo import (Mesh, Attribute, heightfield, refine, SurfaceAnchor,
                     save_snapshot, load_snapshot, fit_surface, reproject_anchor)

class AdaptiveTests(unittest.TestCase):
    def plane(self,n=5):
        x=np.linspace(0,1,n)
        return heightfield(x,x,np.zeros((n,n)))

    def test_local_conforming_and_exact_unchanged(self):
        m=self.plane(); r=refine(m,.2,faces=[0])
        self.assertLess(len(r.changed_faces),len(m.faces))
        np.testing.assert_array_equal(r.mesh.vertices[:len(m.vertices)],m.vertices)
        self.assertEqual(r.mesh.report()['nonmanifold_edges'],0)
        for i in set(range(len(m.faces)))-set(r.changed_faces):
            np.testing.assert_array_equal(r.mesh.faces[r.parent_faces==i][0],m.faces[i])
        for f,p,b in zip(r.mesh.faces,r.parent_faces,r.corner_coordinates):
            np.testing.assert_allclose(r.mesh.vertices[f],b@m.vertices[m.faces[p]])
        # An interior split edge is used by four child boundary edges total,
        # and manifold validation detects accidental one-sided splits.
        self.assertEqual(r.mesh.report()['boundary_edges'],m.report()['boundary_edges']+1)

    def test_closed_refinement(self):
        v=np.array([[0,0,0],[1,0,0],[0,1,0],[0,0,1]])
        m=Mesh(v,np.array([[0,2,1],[0,1,3],[1,2,3],[2,0,3]]))
        r=refine(m,.8,[0]); self.assertEqual(r.mesh.report()['boundary_edges'],0)

    def test_anchor_tracks_and_roundtrips_snapshot(self):
        m=self.plane(); a=SurfaceAnchor.on(m,0,[.2,.3,.5]);r=refine(m,.1,[0])
        b=r.follow(a);np.testing.assert_allclose(a.point(m),b.point(r.mesh),atol=1e-14)
        r2=refine(r.mesh,.05,[b.face]); c=r2.follow(b)
        np.testing.assert_allclose(c.point(r2.mesh),a.point(m),atol=1e-14)
        with tempfile.TemporaryDirectory() as d:
            save_snapshot(r2.mesh,d+'/m.npz');copy=load_snapshot(d+'/m.npz')
            np.testing.assert_array_equal(SurfaceAnchor.from_dict(c.to_dict()).point(copy),c.point(r2.mesh))
        with self.assertRaises(ValueError): a.point(r.mesh)

    def test_vertex_face_corner_transfer(self):
        m=self.plane(); attrs={'v':Attribute(m.vertices[:,0]*2+m.vertices[:,1]),
          'f':Attribute(np.arange(len(m.faces)),'face','nearest'),
          'uv':Attribute(m.vertices[m.faces].reshape(-1,3)[:,:2],'corner')}
        m=Mesh(m.vertices,m.faces,attrs,False);r=refine(m,.2,[0])
        np.testing.assert_allclose(r.mesh.attributes['v'].values,r.mesh.vertices[:,0]*2+r.mesh.vertices[:,1])
        np.testing.assert_array_equal(r.mesh.attributes['f'].values,r.parent_faces)
        np.testing.assert_allclose(r.mesh.attributes['uv'].values,r.mesh.vertices[r.mesh.faces].reshape(-1,3)[:,:2])

    def test_new_samples_only_and_material_anchor(self):
        m=self.plane();calls=[]
        def evaluate(p):
            calls.append(len(p));p[:,2]=p[:,0]*.1;return p
        r=refine(m,.2,[0],evaluate)
        self.assertEqual(calls,[len(r.new_vertices)])
        np.testing.assert_array_equal(r.mesh.vertices[:len(m.vertices)],m.vertices)
        a=SurfaceAnchor.on(m,0,[.2,.3,.5]); b=r.follow(a)
        self.assertGreater(b.point(r.mesh)[2],0)

    def test_retopology_fit_projection_and_anchor(self):
        m=self.plane();m=Mesh(m.vertices,m.faces,{'x':Attribute(m.vertices[:,0])},False)
        cage=self.plane(3);cage=Mesh(cage.vertices+[0,0,.02],cage.faces,cage.attributes,False)
        p=fit_surface(m,cage,.03,names=['x'])
        np.testing.assert_allclose(p.mesh.vertices[:,2],0)
        np.testing.assert_allclose(p.mesh.attributes['x'].values,p.mesh.vertices[:,0])
        np.testing.assert_array_equal(p.mesh.faces,cage.faces)
        a=SurfaceAnchor.on(m,1,[.1,.3,.6]);b=reproject_anchor(a,m,p.mesh,.01)
        np.testing.assert_allclose(a.point(m),b.point(p.mesh),atol=1e-14)
        with self.assertRaises(ValueError):fit_surface(m,cage,.001)
        with self.assertRaises(ValueError):fit_surface(m,cage,np.inf)

    def test_noop_and_invalid(self):
        m=self.plane();r=refine(m,100)
        np.testing.assert_array_equal(r.mesh.faces,m.faces)
        with self.assertRaises(ValueError):refine(m,0)
        with self.assertRaises(ValueError):refine(m,.1,[-1])
        with self.assertRaises(ValueError):SurfaceAnchor.on(m,0,[-1,1,1])
        with self.assertRaises(ValueError):refine(m,.1,evaluate=lambda p: p[:1])

    def test_local_bvh_refit_equals_rebuild(self):
        from cybrgeo import MeshBVH, local_displace
        m=self.plane(8); tree=MeshBVH(m,leaf_size=2)
        moved=local_displace(m,[0],[0,0,.12]).mesh
        updated=tree.refit(moved); rebuilt=MeshBVH(moved,leaf_size=2)
        for p in ([.01,.01,.2],[.4,.8,.2],[1,1,-.1]):
            a,b=updated.closest_point(p),rebuilt.closest_point(p)
            self.assertAlmostEqual(a.distance,b.distance,places=12)
            np.testing.assert_allclose(a.point,b.point,atol=1e-14)
        self.assertTrue(updated.root.left is tree.root.left or updated.root.right is tree.root.right)
        self.assertIs(tree.refit(m).root,tree.root)
        with self.assertRaises(ValueError):tree.refit(refine(m,.1).mesh)
