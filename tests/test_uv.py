import unittest
import numpy as np
from cybrgeo.model import Mesh,Attribute
from cybrgeo.uv import unwrap,pack,bake

def sheet():
    return Mesh([[0,0,0],[1,0,0],[1,1,0],[0,1,0]],np.array([[0,1,2],[0,2,3]]),{'color':Attribute(np.array([[0,0,0],[1,0,0],[1,1,0],[0,1,0]]))},False)

class UVTests(unittest.TestCase):
    def test_unwrap_pack_bake(self):
        m=sheet();cs=unwrap(m);self.assertEqual(len(cs),1);p=pack(m,cs);im,mask=bake(p,'color',64)
        self.assertGreater(mask.sum(),1000);self.assertTrue(np.isfinite(im).all());self.assertTrue((im[mask]>=0).all());self.assertTrue((im[mask]<=1).all())
    def test_explicit_seam(self):
        m=sheet();cs=unwrap(m,[(0,2)]);self.assertEqual(len(cs),2);p=pack(m,cs)
        uv=p.attributes['uv'].values.reshape(-1,3,2);self.assertGreater(uv[1,:,0].min(),uv[0,:,0].max());self.assertFalse(np.allclose(uv[0,0],uv[1,0]))
        bake(p,'color',64)
    def test_bad_seam(self):
        with self.assertRaises(ValueError):unwrap(sheet(),[(0,17)])
    def test_closed_rejected(self):
        m=Mesh([[0,0,0],[1,0,0],[0,1,0],[0,0,1]],np.array([[0,2,1],[0,1,3],[1,2,3],[2,0,3]]))
        with self.assertRaisesRegex(ValueError,'disk'):unwrap(m)
    def test_padding_and_coverage(self):
        m=sheet();cs=unwrap(m,[(0,2)]);p=pack(m,cs,.05);uv=p.attributes['uv'].values
        self.assertGreaterEqual(uv.min(),.05-1e-10);self.assertLessEqual(uv.max(),.95+1e-10)
        with self.assertRaises(ValueError):pack(m,cs[:1])
    def test_overlap_rejected(self):
        m=sheet();uv=np.tile([[.1,.1],[.9,.1],[.1,.9]],(2,1));attrs=dict(m.attributes);attrs['uv']=Attribute(uv,'corner');m=Mesh(m.vertices,m.faces,attrs,False)
        with self.assertRaisesRegex(ValueError,'Overlapping'):bake(m,'color',32)

    def test_nonseparating_cylinder_cut(self):
        n=12
        v=np.array([[np.cos(i*2*np.pi/n),np.sin(i*2*np.pi/n),z] for z in (0,.5,1) for i in range(n)])
        f=np.array([face for z in range(2) for i in range(n) for face in ((z*n+i,z*n+(i+1)%n,(z+1)*n+(i+1)%n),(z*n+i,(z+1)*n+(i+1)%n,(z+1)*n+i))])
        m=Mesh(v,f,closed=False)
        with self.assertRaises(ValueError):unwrap(m)
        c=unwrap(m,[(0,n),(n,2*n)])
        self.assertEqual(len(c),1)
        self.assertEqual(len(c[0].vertices),len(v)+3)
        p=pack(m,c)
        self.assertEqual(len(p.vertices),len(v))
        self.assertEqual(p.attributes['uv'].domain,'corner')
