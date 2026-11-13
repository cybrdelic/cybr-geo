"""Small executable acceptance proof, not a finished visual showcase.
Run: PYTHONPATH=. python examples/construction_constraints.py
Requires the optional scipy profiles dependency.
"""
from cybrgeo import Graph, BuildError, extrude


def square(center, width):
    x,y=center; r=width/2
    return [(x-r,y-r),(x+r,y-r),(x+r,y+r),(x-r,y+r)]


g = Graph()
g.input('insert_x', 4.)
g.part('block', lambda: extrude(square((0,0),4), 4), assume_valid_solid=True)
g.part('insert', lambda x: extrude(square((x,0),1), 1, z=1), 'insert_x',
       assume_valid_solid=True, clearance=.25)
accepted = g.build()
print('Accepted separated bodies')

try:
    g.edit(insert_x=0.)  # Build and parameter edit both roll back on rejection.
except BuildError as error:
    assert g.last is accepted
    for a,b,result in error.cause.conflicts:
        print('Rejected', a, b, result.state, 'inside point:', result.inside_point)
else:
    raise AssertionError('Containment was incorrectly accepted')

g.allow_overlap('block', 'insert', reason='intentional embedded insert')
accepted = g.edit(insert_x=0.)
print('Accepted explicit exception:', accepted.report['construction']['results'][0][2].allowed_reason)
g.separate('block', 'insert', clearance=.25)  # Revokes the exception.
g.set('insert_x', 4.)
assert g.build().report['committed']
print('Rebuilt safely after correction')
