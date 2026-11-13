"""Run under either original or new package via PYTHONPATH."""
import time,json,statistics
import numpy as np
from cybrgeo import heightfield,sculpt
x=np.linspace(0,1,100);xx,yy=np.meshgrid(x,x,indexing='ij')
m=heightfield(x,x,.1*np.sin(xx*7)*np.cos(yy*4))
# deformation doesn't change topology; collapse checks must still execute.
d=np.zeros_like(m.vertices);d[:,2]=.0001*np.sin(m.vertices[:,0]*10)
times=[]
for _ in range(7):
 t=time.perf_counter();r=sculpt(m,d);times.append(time.perf_counter()-t)
print(json.dumps(dict(vertices=len(m.vertices),triangles=len(m.faces),median_seconds=statistics.median(times),repeats=7)))
