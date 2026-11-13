import numpy as np
import cybrgeo as geo

scene = []
guides = []
x = np.linspace(-3,3,128)
y = np.linspace(-3,3,128)
xx,yy = np.meshgrid(x,y,indexing='ij')
height = 0.18+1.4*np.exp(-((xx+0.9)**2+(yy-0.4)**2)/2.0)
height += 1.0*np.exp(-((xx-1.25)**2+(yy+0.8)**2)/0.8)
height += 0.17*np.sin(4*xx+yy)*np.sin(3*yy)*np.exp(-(xx*xx+yy*yy)/9)
rng = np.random.default_rng(12)
for frequency,amplitude in [(1.5,0.25),(3.0,0.14),(6.0,0.07),(12.0,0.035)]:
    phase = rng.uniform(0,2*np.pi,4)
    ridges = 1-np.abs(np.sin(frequency*xx+phase[0])*np.cos(frequency*yy+phase[1]))
    height += amplitude*ridges*np.exp(-(xx*xx+yy*yy)/12)
terrain = geo.heightfield(x,y,height)
scene = [('initial relief',terrain,(164,143,107))]
channel = np.exp(-((xx-0.5*np.sin(1.5*yy))/0.70)**2)
terrain = geo.region(terrain,'river corridor',channel.ravel())
displacement = np.zeros_like(terrain.vertices)
displacement[:,2] = -0.35
terrain = geo.sculpt(terrain,displacement,region='river corridor')
scene = [('carved valley',terrain,(164,143,107))]
height = terrain.vertices[:,2].reshape(len(x),len(y)).copy()
for step in range(4):
    neighbor_mean = (height[:-2,1:-1]+height[2:,1:-1]+height[1:-1,:-2]+height[1:-1,2:])*0.25
    height[1:-1,1:-1] += 0.35*(neighbor_mean-height[1:-1,1:-1])
    terrain = geo.heightfield(x,y,height)
    scene = [('diffusion-relaxed valley',terrain,(164,143,107))]
AZIMUTH = -45
ELEVATION = 38
