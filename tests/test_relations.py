import unittest
import numpy as np
from cybrgeo import Kinematics,rotation

class RelationChecks(unittest.TestCase):
    def make_slider_crank(self):
        a=np.array([.5,np.sqrt(3)/2,0]);b=np.array([.5+np.sqrt(9-.75),0,0])
        rig=Kinematics().fixed('ground').revolute('crank','ground',(0,0,0),limits=(-720,720))
        rig.revolute('rod','crank',a,limits=(-720,720)).prismatic('slider','ground',limits=(-3,3))
        rig.coincident('rod',b,'slider',b)
        return rig,a,b

    def test_generic_closure_matches_slider_crank_equation(self):
        rig,a,b=self.make_slider_crank();previous=None
        for angle in range(0,361,10):
            pose=rig.solve({'crank':angle},guess=previous)
            expected=np.cos(np.deg2rad(angle+60))+np.sqrt(9-np.sin(np.deg2rad(angle+60))**2)
            self.assertAlmostEqual(b[0]+pose.coordinates['slider'],expected,places=6)
            self.assertLessEqual(pose.closure_error,1e-7)
            previous=pose.coordinates

    def test_bad_or_conflicting_drivers_rejected(self):
        rig,_,_=self.make_slider_crank()
        for driven in ({},{'unknown':0},{'crank':float('nan')},{'crank':30,'rod':0,'slider':0}):
            with self.assertRaises(ValueError):rig.solve(driven)

    def test_underconstrained_joint_not_hidden(self):
        rig,_,_=self.make_slider_crank();rig.revolute('free','ground',(5,0,0))
        with self.assertRaisesRegex(ValueError,'underconstrained'):rig.solve({'crank':0})

    def test_parent_transforms_are_composed(self):
        rig=Kinematics().fixed('ground').revolute('turn','ground',(0,0,0)).prismatic('slide','turn')
        pose=rig.solve({'turn':90,'slide':2})
        np.testing.assert_allclose(pose.transforms['slide'][:3,3],[0,2,0],atol=1e-12)

if __name__=='__main__':unittest.main()
