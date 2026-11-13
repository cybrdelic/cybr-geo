import unittest
import numpy as np
from cybrgeo.curves import transport_frames, sweep, sweep_shell
from cybrgeo.deform import rotation


def carry(v,a,b):
    axis=np.cross(a,b);s=np.linalg.norm(axis);c=a@b
    if s<1e-12:return v
    axis/=s
    return v*c+np.cross(axis,v)*s+axis*(axis@v)*(1-c)


class ClosedSweepTests(unittest.TestCase):
    def test_torus_closed_topology_volume_and_attribute(self):
        u=np.linspace(0,2*np.pi,80,endpoint=False)
        a=np.linspace(0,2*np.pi,16,endpoint=False)
        path=np.c_[3*np.cos(u),3*np.sin(u),0*u]
        q=.25*np.c_[np.cos(a),np.sin(a)]
        m=sweep(q,path,closed=True)
        self.assertEqual(len(m.vertices),80*16)
        self.assertEqual(len(m.faces),2*80*16)
        self.assertEqual(m.report()['boundary_edges'],0)
        self.assertEqual(m.report()['winding_errors'],0)
        self.assertEqual(m.report()['components'],1)
        f=m.faces;v=m.vertices
        volume=np.sum(np.einsum('ij,ij->i',v[f[:,0]],np.cross(v[f[:,1]],v[f[:,2]])))/6
        self.assertAlmostEqual(volume,(16/2*np.sin(2*np.pi/16)*.25**2)*(80*3*np.sin(2*np.pi/80)),places=12)
        self.assertEqual(len(m.attributes['section_coordinate'].values),len(v))
        self.assertTrue(np.any(np.all(np.isin(m.faces, np.r_[np.arange(16),np.arange(79*16,80*16)]),axis=1)))

    def test_holonomy_distributed_without_seam_spike(self):
        # A nonplanar loop whose raw open-path frame does not close.
        u=np.linspace(0,2*np.pi,101,endpoint=False)
        r=3+.45*np.cos(3*u)
        p=np.c_[r*np.cos(u),r*np.sin(u),.7*np.sin(2*u)+.2*np.cos(3*u)]
        t,n,b=transport_frames(p,closed=True)
        np.testing.assert_allclose(np.sum(t*n,axis=1),0,atol=1e-14)
        np.testing.assert_allclose(np.linalg.norm(n,axis=1),1,atol=1e-14)
        np.testing.assert_allclose(np.cross(t,n),b,atol=1e-14)
        changes=[]
        for i in range(len(p)):
            j=(i+1)%len(p);a=carry(n[i],t[i],t[j])
            angle=np.arctan2(t[j]@np.cross(a,n[j]),a@n[j])
            changes.append(angle/np.linalg.norm(p[j]-p[i]))
        self.assertLess(np.ptp(changes),1e-12)

    def test_whole_turn_and_rejections(self):
        u=np.linspace(0,2*np.pi,50,endpoint=False);p=np.c_[3*np.cos(u),3*np.sin(u),0*u]
        q=np.array([[-.1,-.2],[.1,-.2],[.1,.2],[-.1,.2]])
        self.assertTrue(sweep(q,p,twist=2*np.pi,closed=True).closed)
        with self.assertRaisesRegex(ValueError,'whole number'):sweep(q,p,twist=.5,closed=True)
        with self.assertRaisesRegex(ValueError,'scalar'):sweep(q,p,twist=np.zeros(len(p)),closed=True)
        with self.assertRaisesRegex(ValueError,'caps'):sweep(q,p,caps=True,closed=True)
        with self.assertRaises(ValueError):sweep(q,np.vstack((p,p[0])),closed=True)
        with self.assertRaises(ValueError):transport_frames(p[:2],closed=True)

    def test_shell_rejects_outside_crossing_or_touching_profiles(self):
        outer=np.array([[-2,-2],[2,-2],[2,2],[-2,2]],float)
        path=[[0,0,0],[0,0,2]]
        for inner in (outer*2,outer*.5+[2,0],outer.copy()):
            with self.assertRaises(ValueError):sweep_shell(outer,inner,path)
        self.assertTrue(sweep_shell(outer,outer*.5,path).closed)
        with self.assertRaisesRegex(ValueError,'orientation'):sweep_shell(outer,(outer*.5)[::-1],path)

    def test_crossing_profile_rejected_even_without_caps(self):
        q=np.array([[0,0],[3,2],[0,3],[2,0]],float)
        for caps in (False,True):
            with self.assertRaises(ValueError):sweep(q,[[0,0,0],[0,0,4]],caps=caps)
        u=np.linspace(0,2*np.pi,16,endpoint=False)
        with self.assertRaises(ValueError):sweep(q,np.c_[10*np.cos(u),10*np.sin(u),u*0],closed=True)

    def test_rotation_does_not_mutate_caller_axis(self):
        a=np.array([0.,0.,4.]);before=a.copy()
        rotation(a,30)
        np.testing.assert_array_equal(a,before)
        a.flags.writeable=False
        rotation(a,30)

if __name__=='__main__':unittest.main()
