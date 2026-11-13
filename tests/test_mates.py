import numpy as np
import unittest
from cybrgeo import Feature, Frame, Graph, BuildError, mate, Mesh, Attribute
from examples.bound_cartridge import build_graph, housing

class MateChecks(unittest.TestCase):
    def test_geometry_rebuild_and_fit(self):
        g=build_graph(); a=g.build(['seat','shaft','placed-shaft'])
        self.assertAlmostEqual(a.outputs['seat'].radius, 9)
        g.set('bore-radius',11).set('bore-offset',np.array([4.,2.]))
        b=g.build(['seat','shaft','placed-shaft'])
        np.testing.assert_allclose(b.outputs['seat'].origin,[4,2,8],atol=1e-10)
        self.assertAlmostEqual(b.outputs['seat'].radius, 11)
        v=b.outputs['placed-shaft'].vertices
        self.assertAlmostEqual(np.max(np.linalg.norm(v[:,:2]-[4,2],axis=1)), 10.8)
        assert set(b.report['evaluated']) == {'housing','seat','shaft','placed-shaft'}
        assert len(g.build(['placed-shaft']).report['evaluated']) == 0
    
    def test_rotation_does_not_rebuild_geometry(self):
        g=build_graph();g.build('placed-shaft');g.set('rotation',42)
        assert g.build('placed-shaft').report['evaluated']==['placed-shaft']
    
    def test_missing_label_rolls_back(self):
        g=build_graph();old=g.build('placed-shaft')
        g.replace('housing',lambda r,o: Mesh(housing(r,o).vertices,housing(r,o).faces))
        with self.assertRaisesRegex(BuildError,'Feature label lost'): g.build('placed-shaft')
        assert g.last is old
    
    def test_reject_bad_rim_and_ambiguous_orientation(self):
        m=housing(9,np.array([0.,0.]));v=m.vertices.copy()
        v[np.where(m.attributes['bearing-seat'].values>0)[0][0],0]+=.5
        with self.assertRaisesRegex(ValueError,'not circular'): Feature('bearing-seat').resolve(Mesh(v,m.faces,m.attributes))
        with self.assertRaisesRegex(ValueError,'ambiguous'): Feature('bearing-seat',normal_hint=(1,0,0)).resolve(m)
        with self.assertRaisesRegex(ValueError,'lost'): Feature('wrong').resolve(m)
    
    def test_mate_preserves_arbitrary_orientation(self):
        from cybrgeo import rotation
        m=housing(9,np.array([0.,0.]));f=Feature('bearing-seat').resolve(m)
        target=Frame(rotation((1,1,0),38)@f.matrix,9)
        T=mate(f,target,angle=23,slide=7)
        np.testing.assert_allclose((T@np.r_[f.origin,1])[:3],target.origin+7*target.axis,atol=1e-10)
        np.testing.assert_allclose(T[:3,:3]@f.axis,target.axis,atol=1e-10)
        self.assertAlmostEqual(np.linalg.det(T[:3,:3]), 1)
    
    def test_clearance_and_planar_frame(self):
        m=housing(9,np.array([0.,0.]));f=Feature('bearing-seat').resolve(m)
        with self.assertRaises(ValueError):f.fitted_radius(-1)
        with self.assertRaises(ValueError):f.fitted_radius(9)
        assert Feature('bearing-seat',kind='plane').resolve(m).radius is None
