#!/usr/bin/env python3
"""Reconstruct the exact native collision cache from the bundled full mesh.

The default output is a temporary sibling followed by an atomic rename after
the SHA-256 matches geometry.json. --verify-only computes the same byte stream
without retaining a duplicate binary. The full source NPZ is never changed.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from cybrgeo.native_mesh import native_chunks

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--geometry', type=Path, required=True)
    parser.add_argument('--provenance', type=Path, required=True)
    parser.add_argument('--output', type=Path, default=Path('P6B-recovered-native.bin'))
    parser.add_argument('--verify-only', action='store_true')
    args = parser.parse_args()
    expected = json.loads(args.provenance.read_text())['output_native_sha256']
    checksum = hashlib.sha256()
    byte_count = 0
    temporary = args.output.with_name(args.output.name + '.tmp')
    stream = None
    try:
        if not args.verify_only:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            stream = temporary.open('wb')
        with np.load(args.geometry, allow_pickle=False) as source:
            for chunk in native_chunks(source):
                checksum.update(chunk)
                byte_count += len(chunk)
                if stream is not None:
                    stream.write(chunk)
        if stream is not None:
            stream.close()
            stream = None
        actual = checksum.hexdigest()
        if actual != expected:
            raise ValueError(f'Native hash mismatch: expected {expected}, obtained {actual}')
        if not args.verify_only:
            temporary.replace(args.output)
        print(json.dumps({'native_sha256': actual, 'expected_sha256': expected,
                          'sha256_matches': True, 'bytes': byte_count,
                          'output': None if args.verify_only else str(args.output)}, indent=2))
    finally:
        if stream is not None:
            stream.close()
        if temporary.exists():
            temporary.unlink()


if __name__ == '__main__':
    main()
