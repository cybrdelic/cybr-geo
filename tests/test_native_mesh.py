"""Interchange tests; no mechanical or material-performance claims."""
import hashlib
import struct
import tempfile
import unittest
from pathlib import Path

import numpy as np

from cybrgeo.native_mesh import native_chunks, native_geometry_digest


class NativeMeshTests(unittest.TestCase):
    def setUp(self):
        self.mesh = dict(positions=np.eye(3, dtype='f4'),
                         normals=np.ones((3, 3), dtype='f4'),
                         offsets=np.array([[0, 3]]), body_indices=np.array([7]),
                         material_indices=np.array([2]))

    def test_known_wire_bytes_and_digest(self):
        # Independent literal wire record, not another call to the serializer.
        expected = (struct.pack('<6I', 0x41525850, 1, 0, 7, 2, 1)
                    + struct.pack('<18f', 1, 0, 0, 0, 1, 0, 0, 0, 1, *([1] * 9)))
        self.assertEqual(b''.join(native_chunks(self.mesh)), expected)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'mesh.npz'
            np.savez(path, **self.mesh)
            self.assertEqual(native_geometry_digest(path),
                             dict(sha256=hashlib.sha256(expected).hexdigest(),
                                  bytes=len(expected), parts=1, triangles=1))

    def test_rejects_incomplete_or_out_of_bounds_triangle(self):
        for offset in ([0, 0], [0, 2], [-1, 3], [1, 3]):
            with self.subTest(offset=offset):
                self.mesh['offsets'] = np.array([offset])
                with self.assertRaisesRegex(ValueError, 'triangle bounds'):
                    list(native_chunks(self.mesh))

    def test_rejects_nonfinite_geometry(self):
        for field in ('positions', 'normals'):
            for value in (np.nan, np.inf):
                with self.subTest(field=field, value=value):
                    original = self.mesh[field][0, 0]
                    self.mesh[field][0, 0] = value
                    with self.assertRaisesRegex(ValueError, 'nonfinite'):
                        list(native_chunks(self.mesh))
                    self.mesh[field][0, 0] = original


if __name__ == '__main__':
    unittest.main()
