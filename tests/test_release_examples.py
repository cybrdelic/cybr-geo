import unittest
from examples import protected_cartridge, periodic_tube, follow_surface

class ReleaseExamples(unittest.TestCase):
    def test_protected_cartridge(self):
        self.assertTrue(protected_cartridge.build().report['physical_assembly'])
    def test_periodic_tube(self):
        self.assertEqual(periodic_tube.build().report()['components'], 1)
    def test_surface_correspondence(self):
        mesh, frame = follow_surface.build()
        self.assertGreater(len(mesh.faces), 72)
