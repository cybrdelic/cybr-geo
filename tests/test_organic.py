import unittest
import numpy as np
from cybrgeo import Mesh, Attribute, heightfield
from cybrgeo.organic import crease_subdivide, local_displace, lattice_deform, Multires, transfer_attributes

class OrganicTests(unittest.TestCase):
    def plane(self,n=4):
        x=np.linspace(0,1,n); return heightfield(x,x,np.zeros((n,n)))

    def test_local_exact(self):
        m=self.plane();edit=local_displace(m,[5],[0,0,.1])
        np.testing.assert_array_equal(edit.mesh.vertices[np.arange(16)!=5],m.vertices[np.arange(16)!=5])
        np.testing.assert_array_equal(edit.faces,np.flatnonzero(np.any(m.faces==5,axis=1)))
        np.testing.assert_array_equal(edit.mesh.faces,m.faces)
        np.testing.assert_array_equal(m.vertices[:,2],0)

    def test_lattice_affine(self):
        m=self.plane();o=np.zeros((3,3,3,3))
        for i in range(3): o[i,:,:,2]=i*.1
        result=lattice_deform(m,[[0,0,-1],[1,1,1]],o)
        np.testing.assert_allclose(result.vertices[:,2],m.vertices[:,0]*.2,atol=1e-15)
        np.testing.assert_array_equal(result.faces,m.faces)
        np.testing.assert_array_equal(result.attributes['uv'].values,m.attributes['uv'].values)

    def test_multires_retains_detail(self):
        p=Multires(self.plane(),2);d=np.zeros_like(p.mesh().vertices);d[7,2]=.05
        detailed=p.edit(2,d); moved=detailed.edit(0,[0,0,.3])
        np.testing.assert_allclose(moved.mesh().vertices,detailed.mesh().vertices+[0,0,.3],atol=1e-14)
        np.testing.assert_allclose(p.mesh().vertices[:,2],0)

    def test_hard_edges_propagate(self):
        v=np.array([[0,0,0],[1,0,0],[0,1,0],[0,0,1]],float)
        f=np.array([[0,2,1],[0,1,3],[1,2,3],[2,0,3]])
        m=Mesh(v,f);edges=np.unique(np.sort(np.concatenate([f[:,[0,1]],f[:,[1,2]],f[:,[2,0]]]),axis=1),axis=0)
        s=crease_subdivide(m,2,edges)
        np.testing.assert_array_equal(s.vertices[:4],v)
        self.assertEqual(len(s.faces),64)
        self.assertEqual(s.report()['boundary_edges'],0)

    def test_transfer_affine_attribute(self):
        m=self.plane(); m=Mesh(m.vertices,m.faces,{'value':Attribute(m.vertices[:,0]+2*m.vertices[:,1])},False)
        target=crease_subdivide(m)
        out=transfer_attributes(m,target,max_distance=.001)
        np.testing.assert_allclose(out.attributes['value'].values,out.vertices[:,0]+2*out.vertices[:,1],atol=1e-14)
        far=local_displace(target,np.arange(len(target.vertices)),[0,0,2]).mesh
        with self.assertRaises(ValueError): transfer_attributes(m,far,max_distance=.01)

    def test_invalid_inputs(self):
        m=self.plane()
        with self.assertRaises(ValueError): crease_subdivide(m,creases=[(0,15)])
        with self.assertRaises(ValueError): local_displace(m,[.5],[0,0,1])
        with self.assertRaises(ValueError): lattice_deform(m,[[0,0,0],[0,1,1]],np.zeros((2,2,2,3)))
