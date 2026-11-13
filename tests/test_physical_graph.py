import unittest
import numpy as np
from cybrgeo import Graph, BuildError, Mesh, Solid, Feature
from test_spatial import cube
from test_relationship_build import host, shaft

class PhysicalGraphTests(unittest.TestCase):
    def model(self):
        g=Graph(physical=True);g.input('x',4.)
        g.op('cutter',lambda:cube(size=10))
        g.part('fixed',lambda:cube(),assume_valid_solid=True)
        g.part('moving',lambda x:cube((x,0,0)),'x',assume_valid_solid=True)
        return g
    def test_required_protection_not_bypassed_by_flag_or_partial_output(self):
        g=self.model();good=g.build();g.set('x',0.)
        for targets in (None,'fixed',[]):
            with self.assertRaises(BuildError):g.build(targets,require_checked=False)
        self.assertIs(g.last,good)
    def test_unregistered_cutter_or_bundle_cannot_be_finished_output(self):
        g=self.model();good=g.build()
        for name in ('cutter',):
            with self.assertRaises(BuildError):g.build(name)
        g.op('hidden',lambda a,b:[a,b],'fixed','cutter')
        with self.assertRaises(BuildError):g.build('hidden',require_checked=False)
        g.op('raw',lambda a:dict(mesh=a,v=a.vertices,f=a.faces),'fixed')
        with self.assertRaises(BuildError):g.build('raw')
        self.assertIs(g.last,good)
    def test_pair_waiver_and_late_registration_not_available(self):
        g=self.model()
        with self.assertRaisesRegex(ValueError,'blanket'):g.allow_overlap('fixed','moving',reason='sewn')
        with self.assertRaisesRegex(ValueError,'part'):g.body('cutter',assume_valid_solid=True)
    def test_single_final_union_is_allowed_but_hidden_components_rejected(self):
        g=Graph(physical=True)
        g.op('a',lambda:Solid.box((2,2,2)))
        g.op('b',lambda:Solid.box((2,2,2)).move((1,0,0)))
        g.part('joined',lambda a,b:a.union(b).mesh(),'a','b',assume_valid_solid=True)
        self.assertEqual(g.build().report['coverage']['status'],'registered_only')
        a,b=cube(),cube((5,0,0));m=Mesh(np.vstack([a.vertices,b.vertices]),np.vstack([a.faces,b.faces+len(a.vertices)]),connected=False)
        g.replace('joined',lambda a,b:m)
        with self.assertRaisesRegex(BuildError,'connected'):g.build()
    def test_fit_rebuilds_by_relationship_and_failure_restores_inputs(self):
        g=Graph(physical=True);g.input('radius',3.);g.input('offset',np.zeros(2));g.input('gap',.2)
        g.part('housing',host,'radius','offset',assume_valid_solid=True)
        g.fit('insert',shaft,'housing',target=Feature('bore'),source=Feature('base'),dependencies=('gap',),assume_valid_solid=True,clearance=.1)
        g.build();good=g.edit(radius=4.,offset=np.array([1.,2.]))
        f=Feature('base').resolve(good.outputs['insert'])
        self.assertAlmostEqual(f.radius,3.8);np.testing.assert_allclose(f.origin[:2],[1,2],atol=1e-10)
        with self.assertRaises(BuildError):g.edit(gap=0.)
        self.assertIs(g.last,good);self.assertEqual(g._inputs['gap'][1],.2)
        with self.assertRaisesRegex(ValueError,'unbound'):g.replace('insert',lambda *args:good.outputs['insert'])
        self.assertIn('insert',g._fits)
    def test_accepted_container_outputs_cannot_be_mutated(self):
        g=Graph(physical=True);g.part('part',lambda:cube(),assume_valid_solid=True)
        g.op('bundle',lambda m:dict(parts=[m]),'part')
        accepted=g.build('bundle')
        bundle=accepted.outputs['bundle']
        with self.assertRaises(AttributeError):bundle['parts'].append(cube())
        with self.assertRaises(TypeError):bundle['other']=cube()
        self.assertIs(bundle['parts'][0],g._cache['part'][1])
        self.assertEqual(len(g.build('bundle').outputs['bundle']['parts']),1)

    def test_duplicate_mesh_output_aliases_rejected(self):
        g=Graph(physical=True);g.part('part',lambda:cube(),assume_valid_solid=True)
        accepted=g.build()
        g.op('bundle',lambda m:[m,m],'part')
        with self.assertRaisesRegex(BuildError,'repeats'):g.build('bundle')
        g.op('alias',lambda m:m,'part')
        with self.assertRaisesRegex(BuildError,'repeats'):g.build(['part','alias'])
        self.assertIs(g.last,accepted)

    def test_touching_is_not_waived(self):
        g=self.model();g.set('x',2.)
        with self.assertRaises(BuildError):g.build()
    def test_no_final_declarations_rejected(self):
        g=Graph(physical=True);g.op('mesh',lambda:cube())
        with self.assertRaises(BuildError):g.build()
