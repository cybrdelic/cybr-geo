import unittest
from cybrgeo.solid import Solid

class SolidTests(unittest.TestCase):
    def test_boolean_volumes_and_unchanged_inputs(self):
        a=Solid.box((2,2,2)); b=a.move((1,0,0))
        self.assertAlmostEqual(a.union(b).volume,12)
        self.assertAlmostEqual(a.subtract(b).volume,4)
        self.assertAlmostEqual(a.intersect(b).volume,4)
        self.assertAlmostEqual(a.volume,8)
    def test_empty_and_disconnected(self):
        a=Solid.box((2,2,2)); b=a.move((5,0,0))
        self.assertTrue(a.intersect(b).is_empty)
        with self.assertRaises(ValueError): a.intersect(b).mesh()
        self.assertEqual(a.union(b).solid_count,2)
        self.assertEqual(a.union(b).mesh().report()['components'],2)
    def test_thin_wall_shell(self):
        body=Solid.box((10,10,10)).shell(-1,opening='>Z')
        self.assertAlmostEqual(body.volume,1000-8*8*9,places=5)
        self.assertEqual(body.mesh().report()['boundary_edges'],0)
    def test_round_and_bevel(self):
        a=Solid.box((10,10,10))
        for b in [a.fillet(1,'|Z'),a.chamfer(1,'|Z')]:
            self.assertLess(b.volume,a.volume)
            self.assertEqual(b.mesh().report()['nonmanifold_edges'],0)
    def test_errors(self):
        with self.assertRaises(ValueError): Solid.cylinder(-1,2)
        with self.assertRaises(ValueError): Solid.box((1,1,1)).fillet(99,'|Z')
        with self.assertRaises(ValueError): Solid.box((1,1,1)).shell(0)
    def test_hole_and_mesh_volume(self):
        import numpy as np, math
        s=Solid.box((10,10,4)).subtract(Solid.cylinder(2,8).move((0,0,-4)))
        self.assertAlmostEqual(s.volume,400-math.pi*4*4,places=5)
        m=s.mesh(.01)
        v=m.vertices;f=m.faces
        volume=np.einsum('ij,ij->i',v[f[:,0]],np.cross(v[f[:,1]],v[f[:,2]])).sum()/6
        self.assertLess(abs(volume-s.volume)/s.volume,.001)

if __name__=='__main__': unittest.main()
