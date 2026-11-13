import numpy as np
from cybrgeo import Mesh, Editable, loft, mesh_edges
angle = np.linspace(0, 2*np.pi, 24, endpoint=False)
radius = 1.8 + .16*np.cos(6*angle)
profile = np.c_[radius*np.cos(angle), radius*np.sin(angle)]
lower = np.c_[profile, np.zeros(24)]
upper = np.c_[profile*.82, np.full(24, 2.3)]
seed = loft([lower, upper])
caps = seed.faces[48:]
patches = Mesh(seed.vertices, caps, closed=False, connected=False)
model = Editable(patches)
loops = model.boundaries()
model = model.bridge(*loops)
top = np.where(np.all(model.mesh.vertices[model.mesh.faces][...,2] == 2.3, axis=1))[0]
reference = model.ref('face', int(top[0]))
model = model.inset_faces(top, .16)
model = model.extrude_faces(top, [0, 0, .35])
model = model.inset_faces(top, .22)
model = model.extrude_faces(top, [0, 0, .65])
model = model.inset_faces(top, .38)
model = model.extrude_faces(top, [0, 0, .45])
edge = mesh_edges(model.mesh.faces)[0]
model = model.split_edge(*edge)
related_faces = model.descendants(reference)
result = model.mesh
