import unittest
import numpy as np
from cybrgeo import BezierPatch,Seam,SurfaceNetwork,fit_network,trim_patch

def patch(x=0,y=0):
 return BezierPatch(np.array([[[x+i/3,y+j/3,.1*(x+i/3)**2+.05*(y+j/3)**2] for j in range(4)] for i in range(4)]))
class Tests(unittest.TestCase):
 def test_global_two_edges(self):
  p={'a':patch(),'b':patch(1.1),'c':patch(0,1.1)}
  seams=[Seam('a','u1','b','u0',2),Seam('a','v1','c','v0',2)]
  n=fit_network(p,seams,fixed=['a'],fairness=.01)
  self.assertLess(n.report()['max_control_constraint_residual'],1e-8)
  np.testing.assert_allclose(n.patches['a'].control,p['a'].control,atol=1e-8)
  for t in np.linspace(0,1,19):
   for d in range(3):np.testing.assert_allclose(n.patches['a'].point(1,t,d,0),n.patches['b'].point(0,t,d,0),atol=1e-8)
 def test_reverse_and_axis(self):
  b=BezierPatch(patch(1).control.transpose(1,0,2)[::-1])
  n=fit_network({'a':patch(),'b':b},[Seam('a','u1','b','v0',2,True)],fixed=['a'])
  for t in np.linspace(0,1,11):np.testing.assert_allclose(n.patches['a'].point(1,t),n.patches['b'].point(1-t,0),atol=1e-8)
 def test_contract_rejects_and_transaction(self):
  n=fit_network({'a':patch(),'b':patch(1)},[Seam('a','u1','b','u0')])
  before=n.patches['b'].control.copy()
  with self.assertRaises(ValueError):n.with_patch('b',patch(2))
  np.testing.assert_array_equal(n.patches['b'].control,before)
  new=n.with_patch('b',patch(2),refit=True,fixed=['a']);self.assertLess(new.report()['max_control_constraint_residual'],1e-8)
 def test_fixed_conflict(self):
  with self.assertRaises(ValueError):fit_network({'a':patch(),'b':patch(2)},[Seam('a','u1','b','u0')],fixed=['a','b'])
 def test_singular_reject(self):
  p=BezierPatch(np.zeros((4,4,3)))
  with self.assertRaises(ValueError):SurfaceNetwork({'a':p,'b':p},[Seam('a','u1','b','u0')])
 def test_polygon_hole(self):
  m=trim_patch(patch(),[[0,0],[1,0],[1,.65],[.7,1],[0,1]],[[[.3,.3],[.6,.3],[.6,.6],[.3,.6]]],spacing=.08)
  uv=m.attributes['uv'].values;centers=uv[m.faces].mean(1)
  self.assertFalse(np.any(np.all((centers>.3)&(centers<.6),axis=1)))
  for point,(u,v) in zip(m.vertices,uv):np.testing.assert_allclose(point,patch().point(u,v),atol=1e-12)
  self.assertEqual(m.report()['components'],1)
 def test_trim_invalid(self):
  with self.assertRaises(ValueError):trim_patch(patch(),[[0,0],[1,1],[1,0],[0,1]])
  with self.assertRaises(ValueError):trim_patch(patch(),[[0,0],[2,0],[1,1]])
 def test_immutability(self):
  n=SurfaceNetwork({'a':patch()})
  with self.assertRaises(TypeError):n.patches['b']=patch()
  with self.assertRaises(ValueError):n.patches['a'].control[0,0]=0
 def test_scale(self):
  n=fit_network({'a':patch(),'b':patch(1)},[Seam('a','u1','b','u0',2,scale=2)])
  for t in np.linspace(0,1,5):
   np.testing.assert_allclose(n.patches['a'].point(1,t,1,0),2*n.patches['b'].point(0,t,1,0),atol=1e-8)
 def test_loop(self):
  p={'a':patch(),'b':patch(1.1),'c':patch(0,1.1),'d':patch(1.1,1.1)}
  s=[Seam('a','u1','b','u0'),Seam('a','v1','c','v0'),Seam('b','v1','d','v0'),Seam('c','u1','d','u0')]
  n=fit_network(p,s,fairness=.01);self.assertLess(n.report()['max_control_constraint_residual'],1e-8)
if __name__=='__main__':unittest.main()

class RegressionTests(unittest.TestCase):
 def test_linear_patch_derivatives(self):
  from cybrgeo import bezier
  np.testing.assert_allclose(bezier([[1,2,3]],[0,.5,1]),[[1,2,3]]*3)
  p=BezierPatch(np.array([[[0,0,0],[0,1,0]],[[1,0,0],[1,1,0]]],float))
  np.testing.assert_allclose(p.point(.5,.5,1,0),[1,0,0])
  np.testing.assert_allclose(p.point(.5,.5,0,1),[0,1,0])
  np.testing.assert_allclose(p.point(.5,.5,1,1),[0,0,0])
  n=fit_network({'a':p,'b':BezierPatch(p.control+[1,0,0])},[Seam('a','u1','b','u0')])
  self.assertLess(n.report()['max_control_constraint_residual'],1e-8)
 def test_boundary_is_sampled(self):
  p=patch();m=trim_patch(p,[[0,0],[1,0],[1,1],[0,1]],spacing=.1)
  uv=m.attributes['uv'].values
  for ax in (0,1):
   for side in (0,1):
    q=uv[np.isclose(uv[:,ax],side),1-ax]
    self.assertGreaterEqual(len(q),11)
    self.assertLessEqual(np.diff(np.sort(q)).max(),.10000001)

class TessellationTests(unittest.TestCase):
 def test_welded_boundary(self):
  from cybrgeo import tessellate_network
  n=fit_network({'a':patch(),'b':patch(1.1)},[Seam('a','u1','b','u0',2)])
  m=tessellate_network(n,11)
  self.assertEqual(len(m.vertices),2*11*11-11)
  self.assertEqual(m.report()['components'],1)
  self.assertEqual(m.report()['boundary_edges'],6*10)
  self.assertEqual(m.attributes['normal'].domain,'corner')
  np.testing.assert_allclose(np.linalg.norm(m.attributes['normal'].values,axis=1),1,atol=1e-12)
 def test_swapped_reversed_winding(self):
  from cybrgeo import tessellate_network
  b=BezierPatch(patch(1).control.transpose(1,0,2)[::-1])
  n=fit_network({'a':patch(),'b':b},[Seam('a','u1','b','v0',2,True)])
  m=tessellate_network(n,9);self.assertEqual(m.report()['winding_errors'],0)
  raw=np.cross(m.vertices[m.faces[:,1]]-m.vertices[m.faces[:,0]],m.vertices[m.faces[:,2]]-m.vertices[m.faces[:,0]])
  normal=m.attributes['normal'].values.reshape(-1,3,3).mean(1)
  self.assertTrue(np.all((raw*normal).sum(1)>0))
 def test_trim_analytic_normals(self):
  m=trim_patch(patch(),[[0,0],[1,0],[1,1],[0,1]],spacing=.15)
  p=patch()
  for uv,n in zip(m.attributes['uv'].values,m.attributes['normal'].values):
   ref=np.cross(p.point(*uv,du=1),p.point(*uv,dv=1));ref/=np.linalg.norm(ref)
   np.testing.assert_allclose(n,ref,atol=1e-12)
 def test_four_way(self):
  from cybrgeo import tessellate_network
  p={'a':patch(),'b':patch(1.1),'c':patch(0,1.1),'d':patch(1.1,1.1)}
  s=[Seam('a','u1','b','u0'),Seam('a','v1','c','v0'),Seam('b','v1','d','v0'),Seam('c','u1','d','u0')]
  m=tessellate_network(fit_network(p,s),8)
  self.assertEqual(len(m.vertices),15*15)
  self.assertEqual(m.report()['components'],1)

class TrimStitchTests(unittest.TestCase):
 def test_trimmed_stitch(self):
  from cybrgeo import stitch_network_meshes
  net=fit_network({'a':patch(),'b':patch(1)},[Seam('a','u1','b','u0')])
  square=[[0,0],[1,0],[1,1],[0,1]]
  meshes={n:trim_patch(p,square,holes=[[[.3,.3],[.6,.3],[.6,.6],[.3,.6]]],spacing=.1) for n,p in net.patches.items()}
  m=stitch_network_meshes(net,meshes);self.assertEqual(m.report()['components'],1)
  self.assertEqual(m.attributes['uv'].domain,'corner')
 def test_mismatched_sampling_rejected(self):
  from cybrgeo import stitch_network_meshes
  net=fit_network({'a':patch(),'b':patch(1)},[Seam('a','u1','b','u0')]);square=[[0,0],[1,0],[1,1],[0,1]]
  meshes={n:trim_patch(p,square,spacing=.1 if n=='a' else .2) for n,p in net.patches.items()}
  with self.assertRaisesRegex(ValueError,'tessellation'):stitch_network_meshes(net,meshes)

class ExportTests(unittest.TestCase):
 def test_corner_normals_export(self):
  from cybrgeo import tessellate_network,export_obj
  import tempfile
  from pathlib import Path
  net=fit_network({'a':patch(),'b':patch(1)},[Seam('a','u1','b','u0')]);m=tessellate_network(net,4)
  with tempfile.TemporaryDirectory() as d:
   p=Path(d)/'x.obj';export_obj(m,p);lines=p.read_text().splitlines()
  self.assertEqual(sum(s.startswith('vn ') for s in lines),len(m.faces)*3)
  face=next(s for s in lines if s.startswith('f '));self.assertEqual(face.split()[1].count('/'),2)
