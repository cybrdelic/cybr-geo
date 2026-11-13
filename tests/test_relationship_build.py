import unittest
import numpy as np
from cybrgeo import Graph, Feature, Mesh, Attribute, extrude, BuildError, Solid


def ring(r,n=16):
    a=np.linspace(0,2*np.pi,n,endpoint=False)
    return np.c_[r*np.cos(a),r*np.sin(a)]


def label(mesh,name,mask):
    return Mesh(mesh.vertices,mesh.faces,{name:Attribute(mask.astype(float),interpolation='nearest')})


def host(radius,offset):
    m=extrude([[-8,-8],[8,-8],[8,8],[-8,8]],3,holes=[ring(radius)+offset])
    p=m.vertices
    return label(m,'bore',np.isclose(p[:,2],0)&np.isclose(np.linalg.norm(p[:,:2]-offset,axis=1),radius))


def shaft(frame,gap):
    m=extrude(ring(frame.fitted_radius(gap)),5)
    return label(m,'base',np.isclose(m.vertices[:,2],0))


class RelationshipBuildTests(unittest.TestCase):
    def graph(self):
        g=Graph();g.input('radius',3.);g.input('offset',np.array([0.,0.]));g.input('gap',.2)
        g.part('housing',host,'radius','offset',assume_valid_solid=True)
        g.fit('shaft',shaft,'housing',target=Feature('bore'),source=Feature('base'),
              dependencies=('gap',),assume_valid_solid=True,clearance=.1)
        return g

    def test_fit_reconstructs_size_and_placement_from_actual_host(self):
        g=self.graph();a=g.build(require_checked=True)
        b=g.edit(radius=4.,offset=np.array([1.,2.]))
        p=b.outputs['shaft'].vertices
        np.testing.assert_allclose((p.min(axis=0)+p.max(axis=0))[:2]/2,[1,2],atol=1e-10)
        self.assertAlmostEqual(np.max(np.linalg.norm(p[:,:2]-[1,2],axis=1)),3.8)
        self.assertEqual(b.report['relationships']['shaft']['target'],'bore')
        self.assertEqual(set(b.report['evaluated']),{'housing','shaft'})
        self.assertEqual(g.build().report['evaluated'],[])

    def test_lost_reference_fails_instead_of_guessing(self):
        g=self.graph();a=g.build()
        g.replace('housing',lambda r,o:Mesh(host(r,o).vertices,host(r,o).faces))
        with self.assertRaisesRegex(BuildError,'Feature label lost'):g.build()
        self.assertIs(g.last,a)

    def test_invalid_fit_edit_restores_parameter(self):
        g=self.graph();a=g.build()
        with self.assertRaises(BuildError):g.edit(gap=8.)
        self.assertEqual(g._inputs['gap'][1],.2);self.assertIs(g.last,a)

    def test_generic_replacement_removes_fit_claim(self):
        g=self.graph();old=g.build().outputs['shaft']
        g.replace('shaft',lambda *args:old)
        self.assertNotIn('shaft',g.build().report['relationships'])

    def test_through_cut_tracks_host_depth_in_dependency_graph(self):
        square=[[-1,-1],[1,-1],[1,1],[-1,1]]
        g=Graph();g.input('height',2.)
        g.op('host',lambda h:Solid.box((10,10,h)).move((0,0,5)),'height')
        g.op('cut',lambda h:h.cut_through(square),'host')
        a=g.build('cut').outputs['cut'];self.assertAlmostEqual(a.volume,192.,places=6)
        b=g.edit(height=9.).outputs['cut'];self.assertAlmostEqual(b.volume,864.,places=6)
        bb=b.shape.BoundingBox();self.assertAlmostEqual(bb.zmin,.5);self.assertAlmostEqual(bb.zmax,9.5)
        shifted=[(x+3,y) for x,y in square]
        batch=Solid.box((10,10,2)).cut_through(square,shifted)
        self.assertAlmostEqual(batch.volume,184.,places=6)
        with self.assertRaises(ValueError):b.cut_through([(20,20),(21,20),(21,21),(20,21)])

if __name__=='__main__':unittest.main()
