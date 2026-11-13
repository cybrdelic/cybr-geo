import numpy as np
from cybrgeo.tiled import TiledHeightfield, height_lod
terrain = lambda x, y: 12*np.sin(x/35)*np.cos(y/49) + 4*np.sin((x+y)/17)
site = TiledHeightfield.sample(terrain, tiles=(8,8), cells=32, spacing=1.)
channel = lambda x: 120 + 13*np.sin(x/29)
site, cost = site.edit((0,80,256,160), lambda x,y,z: z-10*np.exp(-((y-channel(x))/5)**2))
bench = lambda x, y: np.exp(-(((x-164)/24)**8 + ((y-169)/19)**8))
site, cost = site.edit((125,138,204,200), lambda x,y,z: z*(1-bench(x,y))+8*bench(x,y))
ridge = lambda x, y: 7*np.exp(-((x-172)/21)**2-((y-207)/8)**2)
site, cost = site.edit((115,183,230,232), lambda x,y,z: z+ridge(x,y))
reference = site.heights()
coarse = height_lod(reference, stride=16, budget=1.0)
fine = height_lod(reference, stride=2, budget=1.0)
mesh = site.mesh()
