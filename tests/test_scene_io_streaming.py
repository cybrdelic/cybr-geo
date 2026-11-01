"""Canonical scene I/O preserves large environment meshes without duplicate buffers."""
import hashlib
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import numpy as np
from cybrgeo import Assembly, Material, Part
from cybrgeo.core import _stream_sha256

class SceneIOStreaming(unittest.TestCase):
    def test_stream_digest_and_archive_roundtrip_without_read_bytes(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder)
            p=Part('triangle',np.array([[0.,0,0],[1,0,0],[0,1,0]]),np.array([[0,1,2]]),np.tile([0.,0,1.],(3,1)))
            scene=Assembly('scene',[p],[Material('stone',(.3,.2,.1),0,.8)])
            with patch.object(Path,'read_bytes',side_effect=AssertionError('whole-file allocation')):
                scene.save(root)
                restored=Assembly.load(root/'scene.json')
            for channel in ('vertices','faces','normals'):
                np.testing.assert_array_equal(getattr(restored.parts[0],channel),getattr(p,channel))
            archive=root/'meshes.npz'
            self.assertEqual(_stream_sha256(archive,7),hashlib.sha256(archive.read_bytes()).hexdigest())
            # NumPy may expose an owned buffer through a view; no archive-backed
            # mapping survives close, and mutation cannot affect a second load.
            values = restored.parts[0].vertices
            owner = values
            while isinstance(owner.base, np.ndarray): owner = owner.base
            self.assertTrue(owner.flags.owndata)
            values[0, 0] = 9
            self.assertEqual(Assembly.load(root / "scene.json").parts[0].vertices[0, 0], 0)
            with archive.open('ab') as stream: stream.write(b'corruption')
            with self.assertRaisesRegex(ValueError,'SHA256'):Assembly.load(root/'scene.json')

if __name__=='__main__':unittest.main()
