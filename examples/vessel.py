import numpy as np

from cybrgeo import field, loft, region, sculpt, subdivide, skin, rotation

theta = np.linspace(0, 2*np.pi, 96, endpoint=False)
guides = [dict(vertices=np.c_[np.cos(theta),np.sin(theta),np.zeros(96)], faces=[], points=list(range(96)), color=[66,155,190])]

z = np.array([0,0.12,0.4,0.9,1.5,2.1,2.7,3.1,3.4])
guides += [dict(vertices=np.c_[np.zeros(9),np.zeros(9),z], faces=[], points=list(range(9)),color=[220,150,70])]

radii = np.array([0.8,0.9,1.15,1.35,1.25,0.95,0.65,0.7,0.85])
guides = [dict(vertices=np.c_[radii,np.zeros(9),z], faces=[], lines=np.c_[np.arange(8),np.arange(1,9)],points=list(range(9)), color=[220,150,70])]

sections = np.array([np.c_[r*np.cos(theta),r*np.sin(theta),np.full(96,h)] for h,r in zip(z,radii)])

guides = [dict(vertices=section,faces=[],lines=np.c_[np.arange(96),np.roll(np.arange(96),-1)],color=[66,155,190]) for section in sections]
model = loft(sections, caps=True)
guides = []

inner = np.array([np.c_[(r-0.11)*np.cos(theta),(r-0.11)*np.sin(theta),np.full(96,h)] for h,r in zip(z[1:][::-1],radii[1:][::-1])])

shell = np.concatenate([sections,inner],axis=0)

model = loft(shell, caps=True)

angle = np.arctan2(shell[:,:,1],shell[:,:,0]); height = shell[:,:,2]

scale = 1 + 0.075*np.cos(12*angle - 2.3*height)

fluted = shell.copy(); fluted[:,:,:2] *= scale[:,:,None]

model = loft(fluted, caps=True)

model = subdivide(model, levels=2)