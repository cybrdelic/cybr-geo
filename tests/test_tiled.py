import unittest
import numpy as np
from cybrgeo.tiled import TiledHeightfield, height_lod

class TiledTests(unittest.TestCase):
    def test_local_edit_matches_full_regeneration_and_shares(self):
        f=lambda x,y: np.sin(x*.2)+np.cos(y*.3)
        a=TiledHeightfield.sample(f,tiles=(7,5),cells=8)
        b,cost=a.edit((7,7,17,15),lambda x,y,z:z+2)
        expected=a.heights();expected[7:16,7:18]+=2
        np.testing.assert_array_equal(b.heights(),expected)
        self.assertTrue(np.shares_memory(a.tile(6,4),b.tile(6,4)))
        self.assertFalse(np.shares_memory(a.tile(1,1),b.tile(1,1)))
        self.assertEqual(cost.evaluated_samples,99)
        self.assertLess(cost.nodes,35)
        self.assertEqual(b.bounds[0][2],expected.min())
        self.assertEqual(b.bounds[1][2],expected.max())
        self.assertFalse(np.array_equal(a.heights(),b.heights()))

    def test_seams_edges_corners_and_single_sample(self):
        a=TiledHeightfield.sample(lambda x,y:x+y,tiles=(3,2),cells=4)
        for rect in [(4,4,4,4),(12,8,12,8),(0,0,0,0),(0,0,12,8)]:
            b,cost=a.edit(rect,lambda x,y,z:z+10)
            h=a.heights();x0,y0,x1,y1=rect;h[y0:y1+1,x0:x1+1]+=10
            np.testing.assert_array_equal(b.heights(),h)
            for ty in range(2):
                for tx in range(2):
                    np.testing.assert_array_equal(b.tile(tx,ty)[:,-1],b.tile(tx+1,ty)[:,0])
            for tx in range(3):
                np.testing.assert_array_equal(b.tile(tx,0)[-1],b.tile(tx,1)[0])

    def test_rejected_edits_keep_snapshot(self):
        a=TiledHeightfield.sample(lambda x,y:x*0,tiles=(2,2),cells=4)
        with self.assertRaises(ValueError):a.edit((0,0,2,2),lambda x,y,z:z+np.nan)
        with self.assertRaises(ValueError):a.edit((0,0,2,2),lambda x,y,z:1)
        with self.assertRaises(ValueError):a.edit((-1,0,2,2),lambda x,y,z:z)
        self.assertEqual(a.bounds[1][2],0)
        with self.assertRaises(ValueError):a.tile(0,0)[0,0]=2

    def test_exact_pl_error_and_budget(self):
        y,x=np.indices((17,17));h=2*x+3*y
        self.assertEqual(height_lod(h,4,0).max_vertical_error,0)
        h=h.astype(float);h[5,6]+=7
        lod=height_lod(h,4,6.9)
        self.assertEqual(lod.max_vertical_error,7)
        self.assertFalse(lod.within_budget)
        self.assertTrue(height_lod(h,4,7).within_budget)
        self.assertEqual(height_lod(h,1,0).max_vertical_error,0)
        with self.assertRaises(ValueError):height_lod(h,3,1)

    def test_full_mesh_matches_samples(self):
        a=TiledHeightfield.sample(lambda x,y:np.sin(x),tiles=(2,2),cells=4)
        m=a.mesh()
        np.testing.assert_array_equal(m.vertices[:,2],a.heights().ravel())
        self.assertEqual(len(m.faces),128)

    def test_local_cost_grows_with_tree_depth_not_total_surface(self):
        a=TiledHeightfield.sample(lambda x,y:x+y,tiles=(4,4),cells=8)
        b=TiledHeightfield.sample(lambda x,y:x+y,tiles=(32,32),cells=8)
        _,small=a.edit((2,2,4,4),lambda x,y,z:z+1)
        _,large=b.edit((2,2,4,4),lambda x,y,z:z+1)
        self.assertEqual(small.copied_height_bytes,large.copied_height_bytes)
        self.assertEqual(small.evaluated_samples,large.evaluated_samples)
        self.assertEqual(small.tiles,1)
        self.assertEqual(large.nodes-small.nodes,6)
