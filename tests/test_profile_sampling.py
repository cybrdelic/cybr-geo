import unittest
import numpy as np
from cybrgeo import extrude, refine_to_size

class ProfileSamplingTests(unittest.TestCase):
    def test_hole_and_dense_cap(self):
        outer=np.array([[-20,-15],[20,-15],[20,15],[-20,15]])
        hole=np.array([[-4,-3],[-4,3],[4,3],[4,-3]])
        mesh=extrude(outer,2,[hole],spacing=3)
        self.assertTrue(mesh.closed)
        centers=mesh.vertices[mesh.faces].mean(1)
        caps=np.isclose(centers[:,2],0)|np.isclose(centers[:,2],2)
        self.assertFalse(np.any((abs(centers[caps,0])<4)&(abs(centers[caps,1])<3)))
        self.assertGreater(len(mesh.vertices),100)
        refined=refine_to_size(mesh,5,max_vertices=5000)
        p=refined.vertices[refined.faces]
        self.assertLessEqual(np.linalg.norm(p-np.roll(p,-1,axis=1),axis=2).max(),5.00000001)

    def test_rejects_invalid_material_regions(self):
        outer=np.array([[-10,-10],[10,-10],[10,10],[-10,10]])
        invalid_holes=[[[9,-2],[12,-2],[12,2],[9,2]], [[20,0],[22,0],[22,2],[20,2]], [[8,-2],[10,-2],[10,2],[8,2]]]
        for hole in invalid_holes:
            with self.assertRaises(ValueError):extrude(outer,2,[hole])
        with self.assertRaises(ValueError):
            extrude(outer,2,[[[-5,-5],[5,-5],[5,5],[-5,5]],[[-1,-1],[1,-1],[1,1],[-1,1]]])
        with self.assertRaises(ValueError):
            extrude([[0,0],[5,5],[0,4],[5,0],[6,-1]],2)

    def test_invalid_sampling(self):
        outer=[[-1,-1],[1,-1],[1,1],[-1,1]]
        for spacing in [0,-1,np.nan,np.inf,.0001]:
            with self.assertRaises(ValueError):extrude(outer,1,spacing=spacing)

if __name__=='__main__':unittest.main()
