"""Executable curved-shell example: every authored line is included in video/source."""
import numpy as np
from cybrgeo.model import Mesh, Attribute
from cybrgeo.uv import unwrap, pack, bake
u,v=np.meshgrid(np.linspace(-1,1,17),np.linspace(-1,1,13))
vertices=np.c_[u.ravel(),v.ravel(),(.25*np.cos(u*2)+.2*np.cos(v*3)+.15*u*v).ravel()]
faces=np.array([(j*17+i,j*17+i+1,(j+1)*17+i+1) if k==0 else (j*17+i,(j+1)*17+i+1,(j+1)*17+i) for j in range(12) for i in range(16) for k in range(2)])
color=np.c_[(u.ravel()+1)/2,(v.ravel()+1)/2,np.full(u.size,.65)]
shell=Mesh(vertices,faces,{'color':Attribute(color)},closed=False)
seams=[(j*17+8,(j+1)*17+8) for j in range(12)]
charts=unwrap(shell,seams)
atlas=pack(shell,charts,padding=.04)
texture,coverage=bake(atlas,'color',256)
np.savez('media/uv_demo.npz',vertices=vertices,faces=faces,uv=atlas.attributes['uv'].values,texture=texture,coverage=coverage)
