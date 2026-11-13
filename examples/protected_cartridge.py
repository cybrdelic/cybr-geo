"""Relationship-first insert: changing the bore rebuilds the mating geometry."""
import numpy as np
from cybrgeo import Graph, Feature, Mesh, Attribute, extrude, BuildError


def circle(radius):
    t = np.linspace(0, 2*np.pi, 32, endpoint=False)
    return radius * np.c_[np.cos(t), np.sin(t)]


def mark(mesh, name, mask):
    return Mesh(mesh.vertices, mesh.faces,
                {name: Attribute(mask.astype(float), interpolation='nearest')})


def housing(radius, center):
    mesh = extrude([[-8,-8],[8,-8],[8,8],[-8,8]], 3,
                   holes=[circle(radius) + center])
    p = mesh.vertices
    return mark(mesh, 'seat', np.isclose(p[:,2], 0) &
                np.isclose(np.linalg.norm(p[:,:2]-center, axis=1), radius))


def insert(seat, gap):
    mesh = extrude(circle(seat.fitted_radius(gap)), 5)
    return mark(mesh, 'base', np.isclose(mesh.vertices[:,2], 0))


def build():
    graph = Graph(physical=True)
    graph.input('radius', 3.)
    graph.input('center', np.zeros(2))
    graph.input('gap', .25)
    graph.part('housing', housing, 'radius', 'center', assume_valid_solid=True)
    graph.fit('insert', insert, 'housing', target=Feature('seat'),
              source=Feature('base'), dependencies=('gap',),
              assume_valid_solid=True, clearance=.1)
    graph.build()
    accepted = graph.edit(radius=4., center=np.array([1., 2.]))
    try:
        graph.edit(gap=0.)
    except BuildError:
        assert graph.last is accepted
    else:
        raise AssertionError('zero-clearance edit must be rejected')
    return accepted

if __name__ == '__main__':
    result = build()
    print(result.report['coverage'])
    print('Bore and insert rebuilt together; invalid edit rolled back.')
