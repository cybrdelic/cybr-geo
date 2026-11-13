import unittest
import numpy as np
from cybrgeo import Graph, BuildError, ConstructionError, heightfield
from test_spatial import cube


def assembly():
    g=Graph();g.input('x',4.)
    g.op('housing',lambda:cube(size=2))
    g.op('part',lambda x:cube((x,0,0),size=1),'x')
    g.body('housing',assume_valid_solid=True)
    g.body('part',assume_valid_solid=True)
    return g


class ConstructionTests(unittest.TestCase):
    def test_separate_default_and_automatic_rejection(self):
        g=assembly();good=g.build();old_cache=dict(g._cache)
        old_queries=g._construction._queries;old_checks=g._construction._checks
        g.set('x',0.)
        with self.assertRaises(BuildError) as caught:g.build()
        err=caught.exception
        self.assertEqual(err.node,'construction')
        self.assertIsInstance(err.cause,ConstructionError)
        a,b,result=err.cause.conflicts[0]
        self.assertEqual((a,b),('housing','part'))
        self.assertEqual(result.state,'contained_overlap')
        self.assertIsNotNone(result.inside_point)
        self.assertFalse(err.report['committed']);self.assertIs(g.last,good)
        self.assertEqual(g._cache,old_cache)
        self.assertIs(g._construction._queries,old_queries)
        self.assertIs(g._construction._checks,old_checks)
        # Failed input remains a draft; a corrected edit can be retried.
        self.assertEqual(g._inputs['x'][1],0.)
        g.set('x',5.)
        self.assertTrue(g.build().report['committed'])

    def test_partial_build_cannot_bypass_guard(self):
        g=assembly();g.build();g.set('x',0.)
        with self.assertRaises(BuildError):g.build('housing')
        with self.assertRaises(BuildError):g.build([])

    def test_explicit_allowance_keeps_evidence_and_revoke(self):
        g=assembly();g.set('x',0.)
        g.allow_overlap('housing','part',reason='embedded insert')
        r=g.build().report['construction']['results'][0][2]
        self.assertEqual(r.allowed_reason,'embedded insert')
        self.assertEqual(r.state,'contained_overlap');self.assertFalse(r.violation)
        g.separate('part','housing')
        with self.assertRaises(BuildError):g.build()

    def test_clearance_and_touching(self):
        g=assembly();g.separate('housing','part',clearance=3.)
        with self.assertRaises(BuildError) as e:g.build()
        r=e.exception.cause.conflicts[0][2]
        self.assertEqual(r.state,'separated');self.assertEqual(r.surface_gap,2.5)
        g.separate('housing','part',clearance=0.)
        g.set('x',1.5)
        with self.assertRaises(BuildError):g.build()

    def test_cache_selectively_rechecks_and_policy_invalidates(self):
        g=assembly();g.op('far',lambda:cube((10,0,0)))
        g.body('far',assume_valid_solid=True)
        self.assertEqual(len(g.build().report['construction']['checked']),3)
        self.assertEqual(len(g.build().report['construction']['reused']),3)
        g.set('x',5.)
        r=g.build().report['construction']
        self.assertEqual(r['reused'],[('far','housing')]);self.assertEqual(len(r['checked']),2)
        g.separate('housing','far',clearance=1.)
        self.assertEqual(g.build().report['construction']['checked'],[('far','housing')])
        self.assertEqual(len(g.build(force=True).report['construction']['checked']),3)

    def test_intermediates_are_not_bodies(self):
        g=assembly();g.op('cutter',lambda:cube(size=5))
        self.assertTrue(g.build().report['committed'])
        self.assertNotIn('cutter',g.last.report['construction']['bodies'])
        g.body('cutter',assume_valid_solid=True)
        with self.assertRaises(BuildError):g.build()

    def test_invalid_and_open_inputs_fail_closed(self):
        g=assembly()
        with self.assertRaises(ValueError):g.body('part')
        with self.assertRaises(KeyError):g.body('absent',assume_valid_solid=True)
        with self.assertRaises(ValueError):g.allow_overlap('housing','part',reason='')
        with self.assertRaises(ValueError):g.allow_overlap('part','part',reason='self')
        with self.assertRaises(ValueError):g.separate('housing','absent')
        for value in (-1,float('nan'),float('inf')):
            with self.assertRaises(ValueError):g.separate('housing','part',clearance=value)
        g.replace('part',lambda x:heightfield([0,1],[0,1],np.zeros((2,2))))
        with self.assertRaises(BuildError):g.build()

    def test_body_clearance_and_replacement(self):
        g=assembly();g.body('part',assume_valid_solid=True,clearance=3.)
        with self.assertRaises(BuildError):g.build()
        g.body('part',assume_valid_solid=True,clearance=0.)
        g.build();g.replace('part',lambda x:cube())
        with self.assertRaises(BuildError):g.build()

class ConstructionInterfaceTests(unittest.TestCase):
    def model(self):
        g=Graph();g.input('x',4.)
        g.op('cutter',lambda:cube(size=10))
        g.part('fixed',lambda:cube(),assume_valid_solid=True)
        g.part('moving',lambda x:cube((x,0,0)),'x',assume_valid_solid=True)
        return g

    def test_part_default_targets_and_coverage(self):
        g=self.model();r=g.build(require_checked=True)
        self.assertEqual(list(r.outputs),['fixed','moving'])
        self.assertNotIn('cutter',r.report['evaluated'])
        self.assertEqual(r.report['coverage']['status'],'pair_constraints_passed')
        self.assertFalse(r.report['coverage']['self_intersections_checked'])
        self.assertEqual(r.report['coverage']['solid_validity'],'caller_asserted')

    def test_transactional_edit(self):
        g=self.model();original=g.build();inputs=g._inputs.copy();revisions=g._revision.copy()
        with self.assertRaises(BuildError):g.edit(x=0.)
        self.assertIs(g.last,original);self.assertEqual(g._inputs,inputs)
        self.assertEqual(g._revision,revisions)
        r=g.edit(x=5.);self.assertTrue(r.report['committed'])
        self.assertEqual(g._inputs['x'][1],5.)
        with self.assertRaises(KeyError):g.edit(x=6.,missing=2.)
        self.assertEqual(g._inputs['x'][1],5.)

    def test_nested_unchecked_outputs(self):
        g=self.model();g.op('bundle',lambda a,b:dict(checked=a,hidden=[b]),'fixed','cutter')
        r=g.build('bundle')
        self.assertEqual(r.report['coverage']['unchecked_mesh_outputs'],["bundle['hidden'][0]"])
        self.assertEqual(r.report['coverage']['status'],'unchecked_outputs')
        with self.assertRaises(BuildError):g.build('bundle',require_checked=True)

    def test_unrecognized_geometry_container_not_certified(self):
        g=self.model()
        class Wrapper: pass
        def make(mesh):
            w=Wrapper();w.mesh=mesh;return w
        g.op('wrapped',make,'fixed')
        with self.assertRaises(BuildError):g.build('wrapped',require_checked=True)
        self.assertEqual(g.build('wrapped').report['coverage']['unknown_outputs'],['wrapped'])

    def test_single_part_and_all_exempt_not_certified(self):
        g=Graph();g.part('one',lambda:cube(),assume_valid_solid=True)
        self.assertEqual(g.build().report['coverage']['status'],'registered_only')
        with self.assertRaises(BuildError):g.build(require_checked=True)
        g=self.model();g.allow_overlap('fixed','moving',reason='intentional')
        self.assertEqual(g.build().report['coverage']['status'],'all_pairs_exempt')
        with self.assertRaises(BuildError):g.build(require_checked=True)

    def test_bad_part_policy_does_not_add_operation(self):
        g=Graph()
        with self.assertRaises(ValueError):g.part('bad',lambda:cube())
        self.assertNotIn('bad',g._ops)
        with self.assertRaises(ValueError):g.part('bad',lambda:cube(),assume_valid_solid=True,clearance=-1)
        self.assertNotIn('bad',g._ops)

    def test_unrelated_checked_pair_cannot_certify_exempt_output(self):
        g=self.model();g.part('third',lambda:cube((10,0,0)),assume_valid_solid=True)
        g.allow_overlap('fixed','moving',reason='joint')
        g.allow_overlap('fixed','third',reason='joint')
        with self.assertRaises(BuildError):g.build('fixed',require_checked=True)
        r=g.build('fixed')
        self.assertEqual(r.report['coverage']['status'],'unconstrained_outputs')

    def test_raw_array_payload_is_unknown(self):
        g=self.model()
        g.op('raw',lambda m:dict(mesh=m,raw=(m.vertices,m.faces)),'fixed')
        with self.assertRaises(BuildError):g.build('raw',require_checked=True)
        self.assertEqual(len(g.build('raw').report['coverage']['unknown_outputs']),2)

    def test_aabb_separation_reports_lower_bound_not_witness(self):
        g=self.model();r=g.build().report['construction']['results'][0][2]
        self.assertEqual(r.gap_kind,'aabb_lower_bound');self.assertIsNone(r.witness)
        self.assertGreater(r.surface_gap,0)
        g.set('x',1.)
        with self.assertRaises(BuildError) as e:g.build()
        r=e.exception.cause.conflicts[0][2]
        self.assertEqual(r.gap_kind,'exact_surface_distance');self.assertIsNotNone(r.witness)

if __name__=='__main__':unittest.main()
