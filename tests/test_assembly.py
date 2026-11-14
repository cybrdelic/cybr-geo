import json
import unittest
import tempfile
from pathlib import Path
from unittest.mock import patch
from cybrgeo import Assembly, AssemblyError, Solid


def box(x=0, size=2):
    return Solid.box((size, size, size)).move((x, 0, 0))


class AssemblyTests(unittest.TestCase):
    def test_separated_scope_and_transaction(self):
        assembly = Assembly({'a': box(), 'b': box(3)})
        accepted = assembly.build()
        assert accepted.report['pair_count'] == 1
        assert not accepted.report['continuous_motion_checked']
        with self.assertRaises(AssemblyError):
            assembly.edit(b=box(1))
        assert assembly.last is accepted
        assert assembly.build().report['pairs'][0]['state'] == 'separated'


    def test_containment_overlap_rejected_even_with_contact(self):
        shift, size = 0, 1
        with self.assertRaises(AssemblyError) as failure:
            Assembly({'a':box(), 'b':box(shift,size)}, contacts=[('a','b')]).build()
        assert failure.exception.report['pairs'][0]['state'] == 'penetration'


    def test_touch_requires_intent_and_no_gap_waiver(self):
        with self.assertRaises(AssemblyError):
            Assembly({'a':box(), 'b':box(2)}).build()
        accepted = Assembly({'a':box(), 'b':box(2)}, contacts=[('a','b')]).build()
        assert accepted.report['pairs'][0]['state'] == 'contact'
        with self.assertRaises(AssemblyError):
            Assembly({'a':box(), 'b':box(3)}, contacts=[('a','b')]).build()


    def test_clearance(self):
        with self.assertRaises(AssemblyError):
            Assembly({'a':box(), 'b':box(2.1)}, clearance=.2).build()


    def test_mesh_validity_unresolved_not_assumed(self):
        with self.assertRaises(AssemblyError):
            Assembly({'a':box().mesh()}).build()


    def test_all_part_support_pairs_and_cavity(self):
        # Housing cavity is legitimate; strut intrusion must not escape visibility.
        housing = Solid.cylinder(5, 3).subtract(Solid.cylinder(4, 5).move((0,0,-1)))
        shaft = Solid.cylinder(1, 3)
        a = Assembly({'housing':housing, 'shaft':shaft, 'support':box(8)})
        assert a.build().report['pair_count'] == 3
        with self.assertRaises(AssemblyError) as failure:
            a.edit(support=box(4.5))
        assert any('housing / support' in e for e in failure.exception.report['errors'])


    def test_unknown_contacts_and_compounds(self):
        with self.assertRaises(ValueError):
            Assembly({'a':box()}, contacts=[('a','missing')])
        with self.assertRaises(AssemblyError):
            Assembly({'a':box().union(box(4))}).build()


    def test_export_manifest_and_preservation(self):
        tmp_path = Path(tempfile.mkdtemp())
        built=Assembly({'a':box(), 'b':box(3)}).build()
        path=built.export(tmp_path/'result')
        manifest=json.loads(path.read_text())
        assert manifest['participants']==['a','b']
        assert len(list(path.parent.glob('*.obj')))==2
        with self.assertRaises(FileExistsError):
            built.export(path.parent)


    def test_crossing_overlap_and_tiny_penetration(self):
        for shift in (1., 1.999999):
            with self.assertRaises(AssemblyError):
                Assembly({'a':box(), 'b':box(shift)}, contacts=[('a','b')]).build()


    def test_backend_query_failure_is_unresolved(self):
        a,b=box(),box(2)
        with patch.object(type(a.shape), 'intersect', side_effect=RuntimeError('failed')):
            with self.assertRaises(AssemblyError) as failure:
                Assembly({'a':a,'b':b},contacts=[('a','b')]).build()
        assert failure.exception.report['pairs'][0]['state']=='unresolved'


    def test_self_interference_backend_failure_rejects(self):
        from OCP.BOPAlgo import BOPAlgo_ArgumentAnalyzer
        with patch('OCP.BOPAlgo.BOPAlgo_ArgumentAnalyzer', side_effect=RuntimeError('failed')):
            with self.assertRaises(AssemblyError):
                Assembly({'a':box()}).build()


    def test_mesh_self_intersection_escape_closed(self):
        import numpy as np
        from test_spatial import cube
        from cybrgeo import Mesh, Graph, BuildError
        original=cube()
        vertices=original.vertices.copy(); vertices[6]=[0,0,-2]
        folded=Mesh(vertices, original.faces)
        graph=Graph(physical=True)
        graph.part('folded',lambda:folded,assume_valid_solid=True)
        with self.assertRaises(BuildError):
            graph.build()



    def test_cache_reuses_unchanged_and_invalidates_only_incident_pairs(self):
        a=Assembly({'a':box(), 'b':box(3), 'c':box(6)})
        first=a.build()
        second=a.build()
        assert len(second.report['reused_bodies'])==3
        assert len(second.report['reused_pairs'])==3
        edited=a.edit(c=box(7))
        assert edited.report['evaluated_bodies']==['c']
        assert edited.report['reused_pairs']==[['a','b']]
        assert len(edited.report['evaluated_pairs'])==2
        with self.assertRaises(AssemblyError):
            a.edit(c=box(3))
        assert a.last is edited
        assert all(p['state']=='separated' for p in a.build().report['pairs'])

    def test_cache_detects_in_place_shape_replacement_and_export_mutation(self):
        a=Assembly({'a':box(), 'b':box(3)})
        good=a.build()
        # Identity unchanged, but wrapped CAD content changed. Do not trust id().
        a._parts['b'].shape.wrapped=box().shape.wrapped
        with self.assertRaises(AssemblyError):
            a.build()
        assert a.last is good
        good.parts['b'].shape.wrapped=box().shape.wrapped
        folder=Path(tempfile.mkdtemp())/'must-not-exist'
        with self.assertRaises(AssemblyError):
            good.export(folder)
        assert not folder.exists()

    def test_report_cannot_poison_cache(self):
        a=Assembly({'a':box(), 'b':box(3)})
        good=a.build()
        good.report['pairs'][0]['state']='forged'
        assert a.build().report['pairs'][0]['state']=='separated'

    def test_pair_cache_respects_policy(self):
        a=Assembly({'a':box(), 'b':box(3)})
        a.build()
        a._clearance=2.
        with self.assertRaises(AssemblyError):
            a.build()
