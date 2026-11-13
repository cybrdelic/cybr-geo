import unittest
import numpy as np
from cybrgeo.model import Mesh
from cybrgeo.spatial import MeshBVH


def triangle(points):
    return Mesh(np.array(points,float),np.array([[0,1,2]]),closed=False)


def cube(center=(0,0,0),size=2):
    v=np.array([[-1,-1,-1],[1,-1,-1],[1,1,-1],[-1,1,-1],
                [-1,-1,1],[1,-1,1],[1,1,1],[-1,1,1]],float)*size/2+center
    f=np.array([[0,2,1],[0,3,2],[4,5,6],[4,6,7],[0,1,5],[0,5,4],
                [1,2,6],[1,6,5],[2,3,7],[2,7,6],[3,0,4],[3,4,7]])
    return Mesh(v,f)


class SpatialChecks(unittest.TestCase):
    def setUp(self):
        self.tri=MeshBVH(triangle([[0,0,0],[2,0,0],[0,2,0]]))

    def test_closest_witness(self):
        h=self.tri.closest_point([.5,.5,3])
        self.assertAlmostEqual(h.distance,3)
        np.testing.assert_allclose(h.barycentric,[.5,.25,.25])
        np.testing.assert_allclose(h.point,h.barycentric@self.tri.triangles[h.face])

    def test_closest_edge(self):
        h=self.tri.closest_point([2,2,0])
        np.testing.assert_allclose(h.point,[1,1,0]);self.assertAlmostEqual(h.distance,2**.5)

    def test_ray_units_and_limits(self):
        h=self.tri.raycast([.5,.5,3],[0,0,-10])
        self.assertAlmostEqual(h.distance,3)
        self.assertIsNone(self.tri.raycast([.5,.5,3],[0,0,-10],2))
        self.assertIsNone(self.tri.raycast([.5,.5,3],[1,0,0]))
        self.assertIsNone(self.tri.raycast([.5,.5,3],[0,0,1]))

    def test_bad_inputs(self):
        with self.assertRaises(ValueError):self.tri.raycast([0,0,0],[0,0,0])
        with self.assertRaises(ValueError):self.tri.closest_point([np.nan,0,0])
        with self.assertRaises(ValueError):self.tri.contacts(self.tri,-1)

    def test_coplanar_crossing(self):
        b=MeshBVH(triangle([[1,-1,0],[1,2,0],[3,0,0]]))
        self.assertAlmostEqual(self.tri.distance(b).distance,0)
        self.assertEqual(len(self.tri.contacts(b)),1)

    def test_transverse_crossing(self):
        b=MeshBVH(triangle([[.5,.5,-1],[.5,.5,1],[2,.5,0]]))
        self.assertAlmostEqual(self.tri.distance(b).distance,0)

    def test_known_clearance_and_witness(self):
        a=MeshBVH(cube());b=MeshBVH(cube((3,0,0)))
        hit=a.distance(b)
        self.assertAlmostEqual(hit.distance,1)
        self.assertAlmostEqual(np.linalg.norm(hit.point_a-hit.point_b),1)
        self.assertFalse(a.clearance(b,.9).violations)
        self.assertTrue(a.clearance(b,1.1).violations)

    def test_containment_is_not_surface_overlap(self):
        outer=MeshBVH(cube(size=4));inner=MeshBVH(cube(size=2))
        self.assertAlmostEqual(outer.distance(inner).distance,1)
        self.assertEqual(outer.contacts(inner),())
        self.assertLess(outer.signed_distance([0,0,0],assume_valid_solid=True),0)
        self.assertGreater(outer.signed_distance([3,0,0],assume_valid_solid=True),0)
        with self.assertRaises(ValueError):outer.signed_distance([0,0,0])
        with self.assertRaises(ValueError):self.tri.signed_distance([0,0,0],assume_valid_solid=True)

    def test_intentional_contact_explicit(self):
        self.assertTrue(self.tri.contacts(self.tri))
        self.assertEqual(self.tri.contacts(self.tri,allowed_pairs=[(0,0)]),())
        self.assertIsNone(self.tri.distance(self.tri,allowed_pairs=[(0,0)]))

    def test_self_crossing_and_seams(self):
        self.assertFalse(MeshBVH(cube()).self_intersections())
        a=self.tri.triangles[0];b=np.array([[.5,.5,-1],[.5,.5,1],[2,.5,0]])
        m=Mesh(np.vstack((a,b)),np.array([[0,1,2],[3,4,5]]),closed=False,connected=False)
        hits=MeshBVH(m).self_intersections()
        self.assertEqual(len(hits),1);self.assertEqual((hits[0].face_a,hits[0].face_b),(0,1))

    def test_bvh_matches_brute_closest(self):
        b=MeshBVH(cube(),leaf_size=1)
        for p in np.random.default_rng(42).uniform(-4,4,(25,3)):
            h=b.closest_point(p)
            expected=np.linalg.norm(np.maximum(np.abs(p)-1,0)) if np.any(np.abs(p)>1) else 1-np.max(np.abs(p))
            self.assertAlmostEqual(h.distance,expected)

    def test_scale_invariance(self):
        for scale in (1e-6,1.,1e6):
            a=MeshBVH(triangle(np.array([[0,0,0],[2,0,0],[0,2,0]])*scale))
            h=a.raycast(np.array([.5,.5,3])*scale,[0,0,-1])
            self.assertAlmostEqual(h.distance/scale,3)
            self.assertAlmostEqual(a.closest_point(np.array([.5,.5,3])*scale).distance/scale,3)

    def test_signed_distance_reversed_winding(self):
        m=cube();reversed_mesh=Mesh(m.vertices,m.faces[:,::-1])
        self.assertAlmostEqual(MeshBVH(reversed_mesh).signed_distance([0,0,0],assume_valid_solid=True),-1)

if __name__=='__main__':unittest.main()
