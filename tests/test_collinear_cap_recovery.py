import unittest
import numpy as np
from cybrgeo import extrude,BezierPatch,trim_patch


def rounded_ring():
    points=[]
    lo=[.24,.10];hi=[.80,.90];r=.09
    for cx,cy,start in [(hi[0]-r,hi[1]-r,0),(lo[0]+r,hi[1]-r,90),(lo[0]+r,lo[1]+r,180),(hi[0]-r,lo[1]+r,270)]:
        for a in np.linspace(start,start+90,9,endpoint=False):
            points.append([cx+r*np.cos(np.radians(a)),cy+r*np.sin(np.radians(a))])
    dense=[]
    for i,a in enumerate(points):
        a=np.array(a);b=np.array(points[(i+1)%len(points)])
        for t in np.linspace(0,1,max(1,int(np.ceil(np.linalg.norm(b-a)/.025))),endpoint=False):
            dense.append(a*(1-t)+b*t)
    return np.array(dense)


class CollinearCapRecovery(unittest.TestCase):
    def test_authored_collinear_samples_and_area(self):
        ring=rounded_ring()
        for spacing in [.04,.02,None]:
            m=extrude(ring,.1,spacing=spacing)
            self.assertEqual(m.report()['boundary_edges'],0)
            self.assertEqual(m.report()['nonmanifold_edges'],0)
            cap=m.faces[np.all(m.vertices[m.faces,2]==0,axis=1)]
            p=m.vertices[cap,:2]
            area=np.abs((p[:,1,0]-p[:,0,0])*(p[:,2,1]-p[:,0,1])-(p[:,1,1]-p[:,0,1])*(p[:,2,0]-p[:,0,0])).sum()/2
            expected=abs(np.sum(ring[:,0]*np.roll(ring[:,1],-1)-ring[:,1]*np.roll(ring[:,0],-1)))/2
            self.assertAlmostEqual(area,expected,places=12)
            for point in ring:self.assertLess(np.min(np.linalg.norm(m.vertices[:,:2]-point,axis=1)),1e-12)

    def test_curved_trim_and_same_ring_as_hole(self):
        ring=rounded_ring();p=BezierPatch(np.array([[[i/3,j/3,.2*(i/3)**2] for j in range(4)] for i in range(4)]))
        for outline,holes in [(ring,()),(np.array([[0,0],[1,0],[1,1],[0,1]]),(ring,))]:
            mesh=trim_patch(p,outline,holes,spacing=.04)
            self.assertEqual(mesh.report()['degenerate_faces'],0)
            self.assertEqual(mesh.report()['nonmanifold_edges'],0)

    def test_invalid_crossing_still_rejected(self):
        with self.assertRaises(ValueError):extrude([[0,0],[1,1],[1,0],[0,1]],.1)
