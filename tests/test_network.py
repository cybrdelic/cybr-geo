import unittest
from unittest.mock import patch
import numpy as np
from cybrgeo import CurveNetwork,parametric_surface,Instances
from cybrgeo.network import NetworkField

class NetworkTests(unittest.TestCase):
 def make(self):return CurveNetwork([[0,0,0],[0,0,1],[-.5,0,1.6],[.5,0,1.6]],[[0,1],[1,2],[1,3]],[.18,.14,.10,.10])
 def test_general_network(self):
  n=self.make();m=n.surface(.07);self.assertEqual(m.report()['components'],1);self.assertEqual(m.report()['boundary_edges'],0);self.assertIn('source_edge',m.attributes)
 def test_edits_change_geometry_source(self):
  n=self.make();p=n.nodes.copy();p[3,0]+=1;edited=n.edit(nodes=p)
  q=np.array([[1.5,0,1.6]]);self.assertLess(edited.evaluator()(q)[0],0);self.assertGreater(n.evaluator()(q)[0],0)
  self.assertEqual(n.nodes[3,0],.5)
 def test_rebuild_after_edit(self):
  n=self.make();a=n.surface(.08);b=n.edit(radii=n.radii*1.2).surface(.08)
  self.assertGreater(np.ptp(b.vertices[:,0]),np.ptp(a.vertices[:,0]));self.assertEqual(b.report()["components"],1)
 def test_field_called(self):
  import cybrgeo.network as mod
  original=mod.field
  with patch.object(mod,'field',wraps=original) as actual:
   self.make().surface(.08);actual.assert_called_once()
 def test_no_silent_radius_fix(self):
  with self.assertRaises(ValueError):self.make().surface(.2)
 def test_pruning_matches_all_edges(self):
  rng=np.random.default_rng(3);p=np.cumsum(rng.normal(size=(42,3)),axis=0);n=CurveNetwork(p,np.c_[np.arange(41),np.arange(1,42)],rng.uniform(.05,.5,42))
  a=n.evaluator();b=n.evaluator();b.index=None;q=rng.normal(size=(300,3))*4
  np.testing.assert_allclose(a(q),b(q),atol=1e-12)
 def test_map_and_instances(self):
  s=parametric_surface(lambda u,v:np.stack((u,v,u*u),axis=-1),np.linspace(0,1,4),np.linspace(-1,1,3))
  t=np.array([np.eye(4),np.eye(4)]);t[1,0,3]=3;m=Instances(s,t).realize()
  self.assertEqual(m.report()['components'],2);self.assertIn('uv',m.attributes)
 def test_invalid(self):
  with self.assertRaises(ValueError):CurveNetwork([[0,0,0],[0,0,0]],[[0,1]],[.1,.1])

if __name__=='__main__':unittest.main()
