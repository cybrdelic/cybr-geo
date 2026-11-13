import sys,time,json,statistics
sys.path.insert(0,'geo_large_edits_work')
import numpy as np
from cybrgeo.tiled import TiledHeightfield
f=lambda x,y: 10*np.sin(x*.011)*np.cos(y*.009)+3*np.sin(x*.08+y*.03)
a=TiledHeightfield.sample(f,tiles=(64,64),cells=32)
t=[]
for _ in range(15):
    start=time.perf_counter();b,cost=a.edit((997,997,1013,1013),lambda x,y,z:z+2);t.append(time.perf_counter()-start)
full=[]
for _ in range(3):
    start=time.perf_counter();c=TiledHeightfield.sample(lambda x,y:f(x,y)+2*((x>=997)&(x<=1013)&(y>=997)&(y<=1013)),tiles=(64,64),cells=32);full.append(time.perf_counter()-start)
np.testing.assert_array_equal(b.heights(),c.heights())
r={'unique_samples':2049**2,'tiles_total':4096,'local_edit_median_seconds':statistics.median(t),'full_regeneration_median_seconds':statistics.median(full),'dirty_tiles':cost.tiles,'new_tree_nodes':cost.nodes,'height_bytes_copied':cost.copied_height_bytes,'unique_samples_evaluated':cost.evaluated_samples,'scope':'CPU tiled sample update only; not meshing, rendering, export, arbitrary mesh edits or all memory allocations','equality':'exact array equality to full regeneration'}
print(json.dumps(r,indent=2));open('geo_large_edits_demo/benchmark.json','w').write(json.dumps(r,indent=2))
