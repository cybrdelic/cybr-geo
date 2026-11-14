"""Overlapping AABBs, disjoint nested hollow solids: no broadphase shortcut."""
from time import perf_counter
import json
from cybrgeo import Assembly, Solid


def ring(radius, height=10):
    return Solid.cylinder(radius, height).subtract(
        Solid.cylinder(radius-.4, height+2).move((0,0,-1)))

parts={f'ring_{i}':ring(2+i) for i in range(12)}
assembly=Assembly(parts)
results={}
for label,fn in [('cold',assembly.build),('unchanged',assembly.build),
                 ('single_edit',lambda:assembly.edit(ring_11=ring(13,11)))]:
    start=perf_counter();result=fn();elapsed=perf_counter()-start
    results[label]=dict(seconds=elapsed, checked_pairs=result.report['pair_count'],
        evaluated_pairs=len(result.report['evaluated_pairs']),
        reused_pairs=len(result.report['reused_pairs']))
print(json.dumps(results,indent=2))
