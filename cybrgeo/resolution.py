"""Explicitly achieved mesh edge-size limits, without changing the source shape.

This is linear edge refinement, not isotropic remeshing or surface fitting.
Existing vertices, boundaries, and supported attributes follow `refine`.
"""
import numpy as np
from .adaptive import refine


def refine_to_size(mesh, max_edge, *, max_passes=12, max_vertices=1_000_000):
    """Return a mesh whose every edge meets max_edge, or fail explicitly.

    `refine` is a single lineage-preserving step; this convenience operation
    repeats it until the requested size is actually attained. It returns a
    Mesh, not a composed reference correspondence. No smoothing or repair is
    performed, and this edge bound is not a surface-error guarantee.
    """
    if not np.isfinite(max_edge) or max_edge <= 0:
        raise ValueError('max_edge must be finite and positive')
    if isinstance(max_passes, bool) or not isinstance(max_passes, int) or max_passes < 0:
        raise ValueError('max_passes must be a nonnegative integer')
    if isinstance(max_vertices, bool) or not isinstance(max_vertices, int) or max_vertices < 3:
        raise ValueError('max_vertices must be an integer >= 3')
    if len(mesh.vertices) > max_vertices:
        raise ValueError('Input mesh exceeds max_vertices')
    current = mesh
    for step in range(max_passes + 1):
        triangles = current.vertices[current.faces]
        lengths = np.linalg.norm(triangles - np.roll(triangles, -1, axis=1), axis=2)
        if lengths.size == 0 or lengths.max() <= max_edge * (1 + 1e-12):
            return current
        if step == max_passes:
            raise ValueError('Edge-size bound not achieved within max_passes')
        # Conservative upper bound: at most one midpoint per unique edge.
        edges = np.sort(np.concatenate((current.faces[:,[0,1]], current.faces[:,[1,2]], current.faces[:,[2,0]])), axis=1)
        edges = np.unique(edges,axis=0)
        long_count = np.count_nonzero(np.linalg.norm(current.vertices[edges[:,0]]-current.vertices[edges[:,1]],axis=1)>max_edge)
        if len(current.vertices) + long_count > max_vertices:
            raise ValueError('Edge-size refinement would exceed max_vertices')
        current = refine(current, max_edge).mesh
