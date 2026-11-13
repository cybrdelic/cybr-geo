import numpy as np
import cybrgeo as geo

scene = []
guides = []
u = np.linspace(-1,1,64)
v = np.linspace(0,1,48)
def surface(u,v):
    x = 2.8*u*(0.02+0.98*np.sin(np.pi*v)**0.7)
    y = 3.6*v-1.8+0.6*np.abs(u)**2*np.sin(2*np.pi*v)
    z = 0.35*np.cos(np.pi*u/2)**2*np.sin(np.pi*v)
    z += 0.024*np.cos(18*np.pi*u)*np.abs(u)**2*np.sin(np.pi*v)
    return np.stack([x,y,z],axis=-1)
fin = geo.parametric_surface(surface,u,v)
scene = [('rest fin surface',fin,(92,146,153))]
x = fin.vertices[:,0]
left = np.clip(-x/2.8,0,1)
right = np.clip(x/2.8,0,1)
weights = np.c_[1-left-right,left,right]
for phase in np.linspace(0,2*np.pi,20):
    bend = 32*np.sin(phase)
    transforms = np.array([np.eye(4),geo.rotation([0,1,0],bend),geo.rotation([0,1,0],-bend)])
    posed = geo.skin(fin,weights,transforms)
    scene = [('weighted fin deformation',posed,(92,146,153))]
AZIMUTH = -70
ELEVATION = 30
