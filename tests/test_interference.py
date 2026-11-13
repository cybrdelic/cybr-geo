import unittest
import numpy as np
from test_spatial import cube
from cybrgeo.model import Mesh
from cybrgeo.interference import SolidQuery, sample_motion

def q(m):return SolidQuery(m,assume_valid_solid=True)
def join(a,b):return Mesh(np.vstack([a.vertices,b.vertices]),np.vstack([a.faces,b.faces+len(a.vertices)]),connected=False)
class InterferenceTests(unittest.TestCase):
    def test_nested(self):
        a=q(cube(size=4));b=q(cube(size=1));r=a.interference(b)
        self.assertEqual(r.state,'contained_overlap');self.assertAlmostEqual(r.surface_gap,1.5);self.assertTrue(r.violation)
        self.assertLess(a.signed_distance(r.inside_point),0)
        self.assertEqual(b.interference(a).state,'contained_overlap')
    def test_gap_and_witness(self):
        r=q(cube()).interference(q(cube((4,0,0))),clearance=2.1)
        self.assertEqual(r.state,'separated');self.assertAlmostEqual(r.surface_gap,2);self.assertTrue(r.violation)
        self.assertAlmostEqual(np.linalg.norm(r.witness.point_a-r.witness.point_b),2)
    def test_touch_and_cross_are_not_false_passes(self):
        for shift in (1,2,0):
            r=q(cube()).interference(q(cube((shift,0,0))))
            self.assertEqual(r.state,'contact_or_overlap');self.assertTrue(r.violation)
    def test_allow_retains_evidence(self):
        r=q(cube(size=4)).interference(q(cube()),allow_reason='press fit')
        self.assertFalse(r.violation);self.assertEqual(r.state,'contained_overlap');self.assertIsNotNone(r.inside_point)
    def test_disconnected_components(self):
        a=join(cube((10,0,0)),cube(size=1))
        self.assertEqual(q(a).interference(q(cube(size=4))).state,'contained_overlap')
    def test_cavity_not_solid(self):
        inner=cube(size=4);shell=join(cube(size=6),Mesh(inner.vertices,inner.faces[:,::-1]))
        self.assertEqual(q(shell).interference(q(cube(size=1))).state,'separated')
        self.assertEqual(q(shell).interference(q(cube((2.5,0,0),size=.2))).state,'contained_overlap')
    def test_thickness_and_no_mutation(self):
        m=cube(size=3);v=m.vertices.copy();f=m.faces.copy()
        for face,p,end,length in q(m).thickness([0,3,5]):self.assertAlmostEqual(length,3)
        np.testing.assert_array_equal(v,m.vertices);np.testing.assert_array_equal(f,m.faces)
    def test_motion_and_validation(self):
        reports=sample_motion(q(cube()),lambda t:q(cube((t,0,0))),[-4,0,4])
        self.assertEqual([r.violation for _,r in reports],[False,True,False])
        with self.assertRaises(ValueError):SolidQuery(cube())
        with self.assertRaises(ValueError):sample_motion(q(cube()),None,[0,0])
        with self.assertRaises(ValueError):q(cube()).interference(q(cube()),allow_reason='')
    def test_scale_reversed(self):
        for s in (1e-6,1,1e6):
            a=cube(size=4*s);a=Mesh(a.vertices,a.faces[:,::-1]);r=q(a).interference(q(cube(size=s)))
            self.assertEqual(r.state,'contained_overlap');self.assertAlmostEqual(r.surface_gap/s,1.5)
    def test_threshold_decision_retains_containment_and_matches_exact(self):
        for x in (0.,.7,2.,2.1,4.):
            for clearance in (0.,.2,3.):
                a=q(cube());b=q(cube((x,0,0),size=1))
                exact=a.interference(b,clearance=clearance)
                fast=a.check_clearance(b,clearance=clearance)
                self.assertEqual(fast.violation,exact.violation)
                if fast.gap_kind=='threshold_lower_bound':
                    self.assertIsNone(fast.witness)
                    self.assertLessEqual(fast.surface_gap,exact.surface_gap+1e-12)
        fast=q(cube(size=4)).check_clearance(q(cube(size=1)))
        self.assertEqual(fast.state,'contained_overlap')
        self.assertIsNotNone(fast.inside_point)
        with self.assertRaises(ValueError):q(cube()).check_clearance(q(cube()),clearance=-1)

if __name__=='__main__':unittest.main()
