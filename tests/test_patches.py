import unittest
import numpy as np
from cybrgeo import BezierPatch,join_patch,join_metrics,stitch_patches,checked_sweep,offset_surface,surface_safety

class PatchTests(unittest.TestCase):
    def patch(self,start=0.):
        return BezierPatch(np.array([[[start+i,j,.15*i*i+.1*j*j] for j in range(4)] for i in range(4)],float))
    def test_exact_trim(self):
        p=self.patch();q=p.trim((.2,.8),(.1,.9))
        for u in np.linspace(0,1,7):
            for v in np.linspace(0,1,7):np.testing.assert_allclose(q.point(u,v),p.point(.2+.6*u,.1+.8*v),atol=1e-12)
    def test_c2_join_and_weld(self):
        a=self.patch();b=join_patch(a,self.patch(4),2);r=join_metrics(a,b)
        self.assertLess(r['max_gap'],1e-12);self.assertLess(r['max_normal_angle_radians'],3e-8);self.assertLess(r['max_normal_curvature_difference'],1e-10)
        for v in np.linspace(0,1,15):
            for d in (0,1,2):np.testing.assert_allclose(a.point(1,v,d,0),b.point(0,v,d,0),atol=1e-12)
        m=stitch_patches(a,b,7,9);self.assertEqual(len(m.vertices),2*7*9-9);self.assertEqual(m.report()['components'],1)
    def test_discontinuous_reject(self):
        with self.assertRaises(ValueError):stitch_patches(self.patch(),self.patch(6))
    def test_normal_crease_detected(self):
        a=self.patch();b=join_patch(a,self.patch(4),0)
        self.assertGreater(join_metrics(a,b)['max_normal_angle_radians'],.1)
    def test_singular_seam(self):
        with self.assertRaises(ValueError):join_metrics(BezierPatch(np.zeros((4,4,3))),self.patch())
    def test_offset(self):
        p=self.patch().mesh(8,8);m,r=offset_surface(p,.05)
        self.assertEqual(m.faces.shape,p.faces.shape);self.assertFalse(r['certified'])
    def test_sweep_crossing_rejected(self):
        t=np.linspace(0,2*np.pi,12,endpoint=False);profile=.13*np.c_[np.cos(t),np.sin(t)]
        path=np.array([[-2,-2,0],[2,2,0],[-2,2,0],[2,-2,0]],float)
        with self.assertRaisesRegex(ValueError,'intersections'):checked_sweep(profile,path,caps=False)
    def test_straight_sweep(self):
        t=np.linspace(0,2*np.pi,12,endpoint=False);p=.1*np.c_[np.cos(t),np.sin(t)]
        m,r=checked_sweep(p,[[0,0,0],[0,0,1],[0,0,2]],caps=False)
        self.assertFalse(r['nonadjacent_contacts']);self.assertFalse(r['certified'])
