"""Lossless ARXP triangle-stream interchange, not mechanical qualification.

Coordinates, normals, part order, body IDs and material IDs stay source-owned.
The format has no unit conversion, remeshing, motion or appearance policy.
"""
import hashlib
import struct

import numpy as np


def native_chunks(source):
    """Yield the canonical little-endian stream from an opened triangle NPZ."""
    offsets = source['offsets']
    positions, normals = source['positions'], source['normals']
    yield struct.pack('<II', 0x41525850, len(offsets))
    for pid, (start, length) in enumerate(offsets):
        start, length = int(start), int(length)
        if length <= 0 or length % 3 or start < 0 or start + length > len(positions):
            raise ValueError('Invalid native component triangle bounds')
        triangles = length // 3
        yield struct.pack('<IIII', pid, int(source['body_indices'][pid]),
                          int(source['material_indices'][pid]), triangles)
        rows = np.empty((triangles, 18), dtype='<f4')
        rows[:, :9] = positions[start:start + length].reshape(triangles, 9)
        rows[:, 9:] = normals[start:start + length].reshape(triangles, 9)
        if not np.isfinite(rows).all():
            raise ValueError('Native geometry contains nonfinite coordinates or normals')
        yield rows.tobytes(order='C')


def native_geometry_digest(path):
    """Hash the same stream used by writers, without retaining a second mesh."""
    checksum = hashlib.sha256()
    byte_count = 0
    with np.load(path, allow_pickle=False) as source:
        for block in native_chunks(source):
            checksum.update(block)
            byte_count += len(block)
        return dict(sha256=checksum.hexdigest(), bytes=byte_count,
                    parts=len(source['offsets']),
                    triangles=sum(int(length) // 3 for _, length in source['offsets']))
