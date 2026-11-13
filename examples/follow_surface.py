"""Carry an explicit surface reference through a topology refinement."""
import numpy as np
from cybrgeo import parametric_surface, SurfaceAnchor, Feature, Correspondence, refine


def build():
    grid = np.linspace(-1, 1, 7)
    surface = parametric_surface(lambda u,v: np.stack([u,v,.3*u*u+.2*v*v], axis=-1), grid, grid)
    feature = Feature.at(SurfaceAnchor.on(surface, 12, [.2,.3,.5]))
    edited = refine(surface, .2)
    carried = feature.follow(Correspondence.refinement(surface, edited))
    np.testing.assert_allclose(feature.resolve(surface).origin, carried.resolve(edited.mesh).origin)
    return edited.mesh, carried.resolve(edited.mesh)

if __name__ == '__main__':
    mesh, frame = build()
    print('Preserved point:', frame.origin, 'Triangles:', len(mesh.faces))
