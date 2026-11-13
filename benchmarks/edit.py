"""Reproducible CPU benchmark; no rendering or agent-token speed claims."""
import json,statistics,time
from pathlib import Path
import numpy as np
from cybrgeo import Graph,heightfield,sculpt,region,inspect
from cybrgeo.model import _topology

def tile(height):
    x=np.linspace(0,1,55);y=np.linspace(0,1,55);xx,yy=np.meshgrid(x,y,indexing='ij')
    return heightfield(x,y,.05*np.sin(8*xx)*np.cos(6*yy)+height*np.exp(-20*((xx-.5)**2+(yy-.5)**2)))

g=Graph()
for i in range(9):
    g.input(f'patch{i}.height',.1)
    g.op(f'patch{i}',tile,f'patch{i}.height')
targets=[f'patch{i}' for i in range(9)]
t=time.perf_counter();first=g.build(targets);first_seconds=time.perf_counter()-t
incremental=[];full=[];noop=[];diagnostics=[]
for n in range(7):
    g.set('patch4.height',.11+n*.01)
    t=time.perf_counter();edit=g.build(targets);incremental.append(time.perf_counter()-t)
    t=time.perf_counter();ref=g.build(targets,force=True);full.append(time.perf_counter()-t)
    for key in targets:
        np.testing.assert_array_equal(edit.outputs[key].vertices,ref.outputs[key].vertices)
        np.testing.assert_array_equal(edit.outputs[key].faces,ref.outputs[key].faces)
    t=time.perf_counter();g.build(targets);noop.append(time.perf_counter()-t)
    t=time.perf_counter();[inspect(m) for m in edit.outputs.values()];diagnostics.append(time.perf_counter()-t)
report=dict(workload='9 independent 55x55 terrain patches; one height input edit',
            vertices=9*55*55,triangles=9*54*54*2,repetitions=7,
            initial_build_seconds=first_seconds,incremental_seconds_median=statistics.median(incremental),
            full_rebuild_seconds_median=statistics.median(full),noop_seconds_median=statistics.median(noop),
            inspect_seconds_median=statistics.median(diagnostics),
            speedup_full_over_incremental=statistics.median(full)/statistics.median(incremental),
            evaluated=edit.report['evaluated'],reused=edit.report['reused'],
            full_and_incremental_geometry='bitwise equal',
            scope='CPU construction only; authored independent patch dependencies, not automatic arbitrary-mesh partitioning')
Path('benchmark.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
