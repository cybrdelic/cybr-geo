import numpy as np
import cybrgeo as geo

scene = []
guides = []
stakes = 16
rows = 12
z = np.linspace(0,2.4,100)
a = np.linspace(0,2*np.pi,8,endpoint=False)
for k in range(stakes):
    theta = k*2*np.pi/stakes
    radius = 0.95+0.17*np.sin(np.pi*z/2.4)
    radius += 0.028*np.cos(np.pi*(z/2.4*(rows-1)+k))
    center = np.c_[radius*np.cos(theta),radius*np.sin(theta),z]
    radial = np.array([np.cos(theta),np.sin(theta),0])
    tangent = np.array([-np.sin(theta),np.cos(theta),0])
    rings = center[:,None,:]+0.028*np.cos(a)[None,:,None]*radial+0.042*np.sin(a)[None,:,None]*tangent
    stake = geo.loft(rings)
    scene.append(('longitudinal wicker',stake,(160,121,72)))
t = np.linspace(0,2*np.pi,160,endpoint=False)
for j,h in enumerate(np.linspace(0,2.4,rows)):
    radius = 0.95+0.17*np.sin(np.pi*h/2.4)-0.028*np.cos(stakes*t/2+j*np.pi)
    center = np.c_[radius*np.cos(t),radius*np.sin(t),np.full(len(t),h)]
    radial = np.c_[np.cos(t),np.sin(t),np.zeros(len(t))]
    sections = center[:,None,:]+0.028*np.cos(a)[None,:,None]*radial[:,None,:]+0.042*np.sin(a)[None,:,None]*np.array([0,0,1])
    sections = np.concatenate([sections,sections[:1]])
    strip = geo.loft(sections,caps=False)
    hoop = geo.Mesh(strip.vertices[:-len(a)],strip.faces%(len(strip.vertices)-len(a)))
    scene.append(('circumferential wicker',hoop,(191,151,96)))
AZIMUTH = -50
ELEVATION = 28
