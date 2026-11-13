import unittest, warnings
import numpy as np
from cybrgeo import Solid

class CadPoleTests(unittest.TestCase):
    def lens(self):
        import cadquery as cq
        R,r,z=85.,20.,11.
        sag=R-np.sqrt(R*R-r*r)
        a=Solid(cq.Solid.makeSphere(R,(0,0,z+1.2+sag-R),angleDegrees1=-90))
        b=Solid(cq.Solid.makeSphere(R,(0,0,z-1.2-sag+R),angleDegrees1=-90))
        return a.intersect(b).intersect(Solid.cylinder(r,20).move((0,0,z-10)))

    def test_exact_zero_poles_are_opt_in(self):
        shape=self.lens()
        with self.assertRaises(ValueError):shape.mesh(.035,.12)
        with warnings.catch_warnings(record=True) as notices:
            warnings.simplefilter('always')
            mesh=shape.mesh(.035,.12,canonicalize_degenerate=True)
        self.assertTrue(any('exactly zero-area' in str(w.message) for w in notices))
        report=mesh.report()
        for key in ['boundary_edges','nonmanifold_edges','winding_errors','nonmanifold_vertices','degenerate_faces']:
            self.assertEqual(report[key],0)
        self.assertTrue(mesh.closed)
        triangles=mesh.vertices[mesh.faces]
        volume=abs(np.einsum('ij,ij->i',triangles[:,0],np.cross(triangles[:,1],triangles[:,2])).sum()/6)
        self.assertLess(abs(volume-shape.volume)/shape.volume,.02)
        np.testing.assert_allclose(mesh.vertices.min(0)[:2],[-20,-20],atol=.04)
        np.testing.assert_allclose(mesh.vertices.max(0)[:2],[20,20],atol=.04)

    def test_finite_area_weld_collapse_is_still_rejected(self):
        shape=Solid.box((.001,1,1))
        with self.assertRaises(ValueError):shape.mesh(weld_tolerance=.01,canonicalize_degenerate=True)

    def test_normal_mesh_unchanged_and_flag_validation(self):
        shape=Solid.box((1,2,3))
        a=shape.mesh();b=shape.mesh(canonicalize_degenerate=True)
        np.testing.assert_array_equal(a.vertices,b.vertices)
        np.testing.assert_array_equal(a.faces,b.faces)
        with self.assertRaises(ValueError):shape.mesh(canonicalize_degenerate='yes')

if __name__=='__main__':unittest.main()
