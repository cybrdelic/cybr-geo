import unittest
import numpy as np
from cybrgeo import Solid

class SolidLoftTests(unittest.TestCase):
    def test_prism(self):
        p=np.array([[-1,-1,0],[1,-1,0],[1,1,0],[-1,1,0]],float)
        s=Solid.loft([p,p+[0,0,3]])
        self.assertAlmostEqual(s.volume,12,places=7)
        self.assertEqual(s.solid_count,1)
        self.assertEqual(s.mesh().report()['boundary_edges'],0)
    def test_taper_and_boolean(self):
        p=np.array([[-2,-2,0],[2,-2,0],[2,2,0],[-2,2,0]],float)
        q=p*.5+[0,0,3]
        s=Solid.loft([p,q]);self.assertAlmostEqual(s.volume,28,places=6)
        cut=s.subtract(Solid.cylinder(.3,5).move((0,0,-1)))
        self.assertLess(cut.volume,s.volume)
        self.assertEqual(cut.mesh().report()['boundary_edges'],0)
    def test_thin_ruled_sections(self):
        a=np.linspace(0,2*np.pi,24,endpoint=False)
        rings=[np.c_[np.full(24,x),2*np.cos(a),.12*np.sin(a)+.01*x*x] for x in [0,1,2,3]]
        s=Solid.loft(rings);self.assertGreater(s.volume,0)
        self.assertEqual(s.mesh(tolerance=.02).report()['components'],1)
    def test_positive_uniform_scale(self):
        s=Solid.box((2,3,4));t=s.scale(2)
        self.assertAlmostEqual(s.volume,24)
        self.assertAlmostEqual(t.volume,192)
        self.assertEqual(t.mesh().report()['components'],1)
        for f in [0,-1,float('inf'),float('nan')]:
            with self.assertRaises(ValueError):s.scale(f)
    def test_input_rejection(self):
        p=np.array([[-1,-1,0],[1,-1,0],[1,1,0],[-1,1,0]],float)
        bad=p.copy();bad[1,2]=.3
        for sections in [[p], [p,p], [p,bad], [p,p*np.nan], [np.vstack([p,p[0]]),np.vstack([p,p[0]])+[0,0,2]]]:
            with self.assertRaises(ValueError):Solid.loft(sections)
        with self.assertRaises(ValueError):Solid.loft([p,p+[0,0,2]],ruled='yes')
