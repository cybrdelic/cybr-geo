"""Cross-module checks; do not confuse bounded guarantees with full workflows."""
import builtins
import unittest
from unittest.mock import patch
import numpy as np
from cybrgeo import Mesh, Attribute, BezierPatch, SurfaceAnchor, Correspondence, refine, simplify, Solid
from cybrgeo.uv import unwrap, pack, bake
from cybrgeo.tiled import TiledHeightfield
from cybrgeo.interference import SolidQuery
from cybrgeo.mates import Feature
from examples.bound_cartridge import build_graph

class IntegrationTests(unittest.TestCase):
    def test_patch_uv_refinement_keeps_corner_data(self):
        control=np.array([[[u,v,.1*u*v] for v in (0,1)] for u in (0,1)])
        m=BezierPatch(control).mesh(4,4)
        attrs={'color':Attribute(m.vertices.copy())}
        m=Mesh(m.vertices,m.faces,attrs,closed=False)
        atlas=pack(m,unwrap(m))
        refined=refine(atlas,.2).mesh
        self.assertEqual(refined.attributes['uv'].domain,'corner')
        im,mask=bake(refined,'color',32)
        self.assertTrue(np.isfinite(im).all()); self.assertGreater(mask.sum(),100)
        # Current LOD must reject rather than silently destroy seams.
        with self.assertRaises(ValueError): simplify(atlas,8,.1)

    def test_tiled_mesh_reference_lineage(self):
        terrain=TiledHeightfield.sample(lambda x,y: .1*x*y,tiles=(2,2),cells=4)
        before=terrain.mesh(); anchor=SurfaceAnchor.on(before,0,[.2,.3,.5])
        changed,cost=terrain.edit((0,0,2,2),lambda x,y,z:z+1)
        after=changed.mesh()
        resolution=Correspondence.deformation(before,after).follow(anchor)
        self.assertEqual(resolution.confidence,'exact-lineage')
        self.assertAlmostEqual(resolution.anchor.point(after)[2]-anchor.point(before)[2],1)

    def test_bound_mates_meet_solid_clearance(self):
        graph=build_graph(); result=graph.build(['housing','placed-shaft'])
        a=SolidQuery(result.outputs['housing'],assume_valid_solid=True)
        b=SolidQuery(result.outputs['placed-shaft'],assume_valid_solid=True)
        self.assertEqual(a.interference(b).state,'separated')
        graph.set('bore-radius',11).set('bore-offset',np.array([4.,2.]))
        result=graph.build(['housing','placed-shaft'])
        self.assertFalse(SolidQuery(result.outputs['housing'],assume_valid_solid=True).interference(
            SolidQuery(result.outputs['placed-shaft'],assume_valid_solid=True)).violation)

    def test_cad_conversion_does_not_invent_feature_identity(self):
        mesh=Solid.box((2,2,2)).mesh()
        with self.assertRaisesRegex(ValueError,'lost'): Feature('bearing-seat').resolve(mesh)
        small=Solid.box((1,1,1)).mesh()
        self.assertEqual(SolidQuery(mesh,assume_valid_solid=True).interference(
            SolidQuery(small,assume_valid_solid=True)).state,'contained_overlap')

    def test_optional_cad_dependency_error_is_clear(self):
        original=builtins.__import__
        def no_cad(name,*args,**kwargs):
            if name=='cadquery': raise ImportError('dependency deliberately absent')
            return original(name,*args,**kwargs)
        with patch('builtins.__import__',side_effect=no_cad):
            with self.assertRaisesRegex(ImportError,'optional cadquery'): Solid.box((1,1,1))
            # Non-CAD modules remain usable without loading the kernel.
            terrain=TiledHeightfield.sample(lambda x,y:x*0,tiles=(1,1),cells=2)
            self.assertEqual(len(terrain.mesh().faces),8)
