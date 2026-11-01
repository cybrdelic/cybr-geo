import unittest,numpy as np
from mechanism_lab.optical_coordinates import recover_affine,coordinates
class Tests(unittest.TestCase):
 def test_recovers_source_and_transports_new_vertices(self):
  rng=np.random.default_rng(17);v=rng.normal(size=(30,3));ix=np.arange(30);m=np.array([[138.8888889,0],[0,0],[0,312.5],[0,0]])
  uv=np.c_[v,np.ones(len(v))]@m
  r=recover_affine(v,ix,uv,uv_name='PrintUV',source_part_id=274,source_mesh_sha256='test',units='m')
  new=rng.normal(size=(50,3));np.testing.assert_allclose(coordinates(new,r),np.c_[new,np.ones(50)]@m,atol=1e-10)
 def test_nonaffine_or_wrong_units_rejected(self):
  rng=np.random.default_rng(17);v=rng.normal(size=(30,3));uv=rng.random((30,2))
  for units in ('m','mm'):
   with self.assertRaises(ValueError):recover_affine(v,np.arange(30),uv,uv_name='PrintUV',source_part_id=1,source_mesh_sha256='test',units=units)
if __name__=='__main__':unittest.main()
