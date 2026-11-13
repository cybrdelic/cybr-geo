import unittest
import numpy as np
from cybrgeo import extrude
class Profiles(unittest.TestCase):
    def test_bore_profile(self):
        t=np.arange(48)*2*np.pi/48
        outline=[[-2,-1],[2,-1],[2,1],[-2,1]]
        holes=[np.c_[.3*np.cos(t)+x,.3*np.sin(t)] for x in [-1,1]]
        mesh=extrude(outline,.2,holes,z=.1)
        self.assertEqual(mesh.report()['boundary_edges'],0)
        self.assertEqual(mesh.report()['components'],1)
        self.assertTrue(set(mesh.vertices[:,2])=={.1,.30000000000000004})
        # Exact polygonal area times height, independent volume check.
        tri=mesh.vertices[mesh.faces]
        volume=np.sum(np.einsum('ij,ij->i',tri[:,0],np.cross(tri[:,1],tri[:,2])))/6
        hole_area=48*.3**2*np.sin(2*np.pi/48)/2
        self.assertAlmostEqual(volume,(8-2*hole_area)*.2,places=10)
    def test_bad_height(self):
        with self.assertRaises(ValueError):extrude([[0,0],[1,0],[0,1]],0)
    def test_concave_profile_with_bore(self):
        t=np.arange(20)*np.pi/10
        hole=np.c_[.2*np.cos(t)+.5,.2*np.sin(t)+2]
        mesh=extrude([[0,0],[3,0],[3,1],[1,1],[1,3],[0,3]],1,[hole])
        self.assertEqual(mesh.report()['boundary_edges'],0)
        tri=mesh.vertices[mesh.faces]
        volume=np.sum(np.einsum('ij,ij->i',tri[:,0],np.cross(tri[:,1],tri[:,2])))/6
        self.assertAlmostEqual(volume,5-20*.2**2*np.sin(np.pi/10)/2,places=10)
    def test_outside_hole_rejected(self):
        with self.assertRaises(ValueError):
            extrude([[0,0],[1,0],[1,1],[0,1]],.2,holes=[[[2,2],[3,2],[3,3],[2,3]]])
