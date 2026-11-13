import unittest
import numpy as np
from cybrgeo import Solid

class SolidOffsetTests(unittest.TestCase):
    def test_inset_is_distance_not_scale(self):
        shape=Solid.extrude([[-10,-5],[10,-5],[10,5],[-10,5]],3,offset=-1)
        mesh=shape.mesh()
        np.testing.assert_allclose(mesh.vertices.min(0),[-9,-4,0],atol=1e-7)
        np.testing.assert_allclose(mesh.vertices.max(0),[9,4,3],atol=1e-7)
        self.assertAlmostEqual(shape.volume,18*8*3,places=6)

    def test_outset_and_invalid(self):
        profile=[[-10,-5],[10,-5],[10,5],[-10,5]]
        self.assertGreater(Solid.extrude(profile,3,offset=1).volume,600)
        with self.assertRaises(ValueError):Solid.extrude(profile,3,offset=float('nan'))
        with self.assertRaises(Exception):Solid.extrude(profile,3,offset=-100)

if __name__=='__main__':unittest.main()
