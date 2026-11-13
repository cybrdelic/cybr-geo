import unittest
import numpy as np
from cybrgeo.curves import *

class CurvesTests(unittest.TestCase):
    def test_bezier(self):
        p=np.array([[0,0,0],[1,2,0],[3,2,0],[4,0,0]])
        np.testing.assert_allclose(bezier(p,[0,1]),p[[0,-1]])
        np.testing.assert_allclose(bezier(p,.5),[2,1.5,0])
    def test_sampling(self):
        p=sample_curve(lambda t: np.array([np.cos(t*np.pi),np.sin(t*np.pi),0]),80)
        length=np.linalg.norm(np.diff(p,axis=0),axis=1)
        self.assertLess(np.ptp(length),1e-5)
        self.assertLess(np.max(abs(np.linalg.norm(p,axis=1)-1)),1e-4)
    def test_frames(self):
        u=np.linspace(0,5,100);p=np.c_[np.cos(u),np.sin(u),u]
        t,n,b=transport_frames(p,twist=np.pi)
        np.testing.assert_allclose(np.linalg.norm(n,axis=1),1)
        np.testing.assert_allclose(np.sum(t*n,axis=1),0,atol=1e-14)
        np.testing.assert_allclose(np.cross(t,n),b)
        self.assertGreater(np.min(np.sum(n[:-1]*n[1:],axis=1)),.98)
    def test_twist(self):
        p=np.c_[np.zeros(10),np.zeros(10),np.linspace(0,1,10)]
        _,n,_=transport_frames(p,initial_normal=[1,0,0],twist=np.pi/2)
        np.testing.assert_allclose(n[-1],[0,1,0],atol=1e-14)
    def test_sweep(self):
        a=np.linspace(0,2*np.pi,20,endpoint=False); q=np.c_[np.cos(a),np.sin(a)]
        mesh=sweep(q,[[0,0,0],[0,0,2],[0,0,4]],scale=[1,.7,.5])
        self.assertEqual(mesh.report()['boundary_edges'],0)
        self.assertEqual(len(mesh.vertices),60)
    def test_matched_loft(self):
        a=np.linspace(0,2*np.pi,12,endpoint=False); b=np.linspace(0,2*np.pi,23,endpoint=False)
        m=matched_loft([np.c_[np.cos(a),np.sin(a),a*0],np.c_[np.cos(b),np.sin(b),b*0+2][::-1]],samples=48)
        self.assertEqual(m.report()['boundary_edges'],0)
    def test_coons(self):
        bottom=lambda u: [u,0,0]
        top=lambda u: [u,1,np.sin(u*np.pi)]
        left=lambda v:[0,v,0]
        right=lambda v:[1,v,0]
        m=coons_patch(bottom,top,left,right,nu=20,nv=15)
        v=m.vertices.reshape(20,15,3)
        np.testing.assert_allclose(v[:,-1],[top(x) for x in np.linspace(0,1,20)],atol=1e-15)
        np.testing.assert_allclose(v[:,0],[bottom(x) for x in np.linspace(0,1,20)])
    def test_shell(self):
        a=np.linspace(0,2*np.pi,24,endpoint=False);q=np.c_[np.cos(a),np.sin(a)]
        m=sweep_shell(q,q*.8,[[0,0,0],[0,0,2]])
        self.assertEqual(m.report()['boundary_edges'],0)
        self.assertEqual(m.report()['winding_errors'],0)
        # Positive outward signed volume, annular prism volume.
        v=m.vertices;f=m.faces
        volume=np.sum(np.einsum('ij,ij->i',v[f[:,0]],np.cross(v[f[:,1]],v[f[:,2]])))/6
        self.assertGreater(volume,0)
        self.assertAlmostEqual(volume,24*np.sin(2*np.pi/24)/2*(1-.8**2)*2)
    def test_rejections(self):
        with self.assertRaises(ValueError): transport_frames([[0,0,0],[1,0,0],[0,0,0]])
        with self.assertRaises(ValueError): resample_polyline([[0,0],[0,0]],8)
        with self.assertRaises(ValueError): coons_patch(lambda t:[t,0,0],lambda t:[t,1,1],lambda t:[0,t,0],lambda t:[1,t,0])
        with self.assertRaises(ValueError): sample_curve(lambda t:[t,t*t,0],tolerance=1e-15,max_depth=1)

if __name__=='__main__': unittest.main()
