import numpy as np
from cybrgeo import BezierPatch, join_patch, join_metrics, stitch_patches, offset_surface
u = np.linspace(0, 1, 5)
v = np.linspace(-1, 1, 5)
control = np.array([[[5*x, 2.2*y*(1-.25*x), 1.3*(1-y*y)+.45*np.sin(3*x)] for y in v] for x in u])
front = BezierPatch(control)
rear_control = control + [5, 0, -.6]
rear = BezierPatch(rear_control)
before = join_metrics(front, rear)
rear = join_patch(front, rear, order=2)
after = join_metrics(front, rear)
front = front.trim((.12, 1), (.08, .92))
rear = rear.trim((0, .85), (.08, .92))
body = stitch_patches(front, rear, nu=28, nv=24)
outer, safety = offset_surface(body, .12)
