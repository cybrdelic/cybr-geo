import unittest
import numpy as np
from cybrgeo import CurveNetwork,sparse_surface
class Sparse(unittest.TestCase):
 def model(self):return CurveNetwork([[0,0,0],[0,0,1],[.5,0,1.5],[-.5,0,1.5]],np.array([[0,1],[1,2],[1,3]]),[.15,.12,.08,.08])
 def test_closed_join_across_blocks(self):
  m=sparse_surface(self.model(),.06,6);r=m.report()
  self.assertEqual(r['components'],1);self.assertEqual(r['boundary_edges'],0)
  self.assertEqual(r['nonmanifold_edges'],0);self.assertIn('source_edge',m.attributes)
 def test_no_radius_inflation(self):
  n=self.model();r=n.radii.copy();sparse_surface(n,.07,6);np.testing.assert_array_equal(n.radii,r)
 def test_invalid(self):
  for s in [0,-1,float('nan'),.2]:
   with self.assertRaises(ValueError):sparse_surface(self.model(),s)
if __name__=='__main__':unittest.main()
