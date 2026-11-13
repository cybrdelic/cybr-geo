import numpy as np
from cybrgeo import bezier, sample_curve, sweep_shell
scene = []
angle = np.linspace(0, 2*np.pi, 48, endpoint=False)
outer = .24*np.column_stack((np.cos(angle), np.sin(angle)))
inner = .205*np.column_stack((np.cos(angle), np.sin(angle)))
flange = .42*np.sign(outer)*abs(outer/.24)**.4
for runner in range(4):
    x = (runner-1.5)*1.2
    controls = np.array([[x,0,2.7], [x,-1.5,2.7], [x*.6,-2.5,.3], [x*.3,-3.,-.6]])
    path = sample_curve(lambda t: bezier(controls,t), count=70)
    scale = np.linspace(1., .9, len(path))
    tube = sweep_shell(outer, inner, path, scale=scale)
    scene.append(tube)
    inlet = np.array([[x,.08,2.7], [x,-.08,2.7]])
    mount = sweep_shell(flange, inner, inlet)
    scene.append(mount)
    for index in (13, 32, 51):
        segment = path[index:index+3]
        collar = sweep_shell(outer*1.075, outer*.99, segment, scale=scale[index:index+3])
        scene.append(collar)
