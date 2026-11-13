import unittest
import numpy as np
from cybrgeo import Graph, BuildError, heightfield, sculpt, region, inspect, Mesh
from cybrgeo.model import _topology

class GraphTests(unittest.TestCase):
    def test_selective_regional_rebuild(self):
        g=Graph();g.input('left.height',1.);g.input('right.height',2.)
        make=lambda h:heightfield([0,1],[0,1],np.ones((2,2))*h)
        g.op('left',make,'left.height');g.op('right',make,'right.height')
        g.op('views',lambda a,b:(a,b),'left','right')
        original=g.build('views');g.set('left.height',3.)
        edited=g.build('views')
        self.assertEqual(edited.report['evaluated'],['left','views'])
        self.assertEqual(edited.report['reused'],['right'])
        self.assertIs(original.outputs['views'][1],edited.outputs['views'][1])
        full=g.build('views',force=True)
        for a,b in zip(full.outputs['views'],edited.outputs['views']):
            np.testing.assert_array_equal(a.vertices,b.vertices)
            np.testing.assert_array_equal(a.faces,b.faces)
    def test_noop_and_array_input(self):
        g=Graph();data=np.arange(5.);g.input('x',data);data[:]=99
        g.op('sum',lambda x:float(x.sum()),'x')
        self.assertEqual(g.build('sum').outputs['sum'],10.)
        g.set('x',np.arange(5.));r=g.build('sum')
        self.assertEqual(r.report['evaluated'],[])
    def test_atomic_failure_preserves_snapshot(self):
        g=Graph();g.input('x',2);g.op('a',lambda x:x*2,'x')
        def check(x):
            if x<0:raise ValueError('bad geometry')
            return x
        g.op('b',check,'a');old=g.build('b')
        cache=dict(g._cache);g.set('x',-1)
        with self.assertRaises(BuildError) as caught:g.build('b')
        self.assertEqual(caught.exception.node,'b');self.assertIs(g.last,old)
        self.assertEqual(g._cache,cache)
        g.set('x',3);self.assertEqual(g.build('b').outputs['b'],6)
    def test_replacement_invalidates_downstream(self):
        g=Graph();g.input('x',1);g.op('a',lambda x:x+1,'x');g.op('b',lambda x:x*2,'a')
        g.build('b');g.replace('a',lambda x:x+2)
        self.assertEqual(g.build('b').outputs['b'],6)
    def test_region_diagnostic(self):
        m=heightfield([0,1],[0,1],np.zeros((2,2)))
        m=region(m,'east',lambda p:p[:,0]);r=inspect(m,region='east')
        self.assertEqual(r['vertices'],2);self.assertEqual(r['bounds'][0][0],1.)
        with self.assertRaises(KeyError):inspect(m,region='missing')
    def test_topology_cache_does_not_hide_collapse(self):
        m=heightfield([0,1],[0,1],np.zeros((2,2)));m.report();before=_topology.cache_info()
        with self.assertRaises(ValueError):sculpt(m,-m.vertices)
        self.assertGreater(_topology.cache_info().hits,before.hits)
    def test_topology_report_not_mutable_cache(self):
        m=heightfield([0,1],[0,1],np.zeros((2,2)));r=m.report();r['components']=500
        self.assertEqual(m.report()['components'],1)
    def test_unrequested_ops_not_executed(self):
        g=Graph();g.input('x',1);g.op('ok',lambda x:x,'x');g.op('bad',lambda x:1/0,'x')
        self.assertEqual(g.build('ok').outputs['ok'],1)

if __name__=='__main__':unittest.main()
