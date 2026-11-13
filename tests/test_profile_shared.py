import unittest
import numpy as np
from cybrgeo.profiles import validate_profile, validate_planar_ring
from cybrgeo import Solid, Frame

class SharedProfileTests(unittest.TestCase):
    def test_validation_does_not_mutate_input(self):
        ring=np.array([[0,0],[0,2],[2,2],[2,0]],float)
        before=ring.copy();self.assertEqual(len(validate_profile(ring)),1)
        np.testing.assert_array_equal(ring,before)
    def test_analytic_extrude_rejects_crossing(self):
        with self.assertRaises(ValueError):Solid.extrude([(0,0),(3,2),(0,3),(2,0)],1)
    def test_backtracking_adjacent_edges_rejected(self):
        with self.assertRaisesRegex(ValueError,'adjacent'):
            validate_profile([[0,0],[2,0],[1,0],[2,2],[0,2]])
    def test_planar_rejects_crossing_and_warp(self):
        with self.assertRaises(ValueError):validate_planar_ring([[0,0,0],[3,2,0],[0,3,0],[2,0,0]])
        with self.assertRaises(ValueError):validate_planar_ring([[0,0,0],[1,0,0],[1,1,.1],[0,1,0]])
    def test_oriented_through_cuts(self):
        host=Solid.box((10,12,14)).move((5,-2,3))
        m=np.eye(4);m[:3,:3]=[[0,0,1],[1,0,0],[0,1,0]];m[:3,3]=[5,-2,3]
        ring=[[-1,-1],[1,-1],[1,1],[-1,1]]
        result=host.cut_through(ring,frame=Frame(m))
        self.assertAlmostEqual(host.volume-result.volume,40,places=6)
        larger=Solid.box((20,12,14)).move((5,-2,3))
        self.assertAlmostEqual(larger.volume-larger.cut_through(ring,frame=Frame(m)).volume,80,places=6)
        self.assertAlmostEqual(host.volume,1680,places=6)
    def test_rotated_host_and_frame(self):
        host=Solid.box((10,12,14)).rotate((0,0,0),(0,0,1),37)
        a=np.deg2rad(37);m=np.eye(4)
        m[:3,:3]=[[0,-np.sin(a),np.cos(a)],[0,np.cos(a),np.sin(a)],[-1,0,0]]
        result=host.cut_through([[-1,-1],[1,-1],[1,1],[-1,1]],frame=Frame(m))
        self.assertAlmostEqual(host.volume-result.volume,40,places=5)
    def test_invalid_frame_and_miss(self):
        host=Solid.box((2,2,2));ring=[[-.2,-.2],[.2,-.2],[.2,.2],[-.2,.2]]
        with self.assertRaises(TypeError):host.cut_through(ring,frame=np.eye(4))
        m=np.eye(4);m[0,3]=10
        with self.assertRaises(ValueError):host.cut_through(ring,frame=Frame(m))
