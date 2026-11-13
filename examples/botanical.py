import numpy as np
from cybrgeo import loft, Mesh
from cybrgeo.curves import bezier, sweep, transport_frames
parts = []
M = np.eye(3)
O = np.zeros(3)
t = np.linspace(0, 1, 65)
a = np.linspace(0, 2*np.pi, 24, endpoint=False)
circle = np.c_[np.cos(a), np.sin(a)]
def tube(path, radius, color):
    mesh = sweep(circle, path, scale=radius)
    parts.append((Mesh(mesh.vertices@M.T+O,mesh.faces), color))
def blade(path, width, thickness, color, across):
    tangent, normal, side = transport_frames(path, initial_normal=across)
    rings = path[:,None,:] + width[:,None,None]*np.cos(a)[None,:,None]*normal[:,None,:] + thickness*np.sin(a)[None,:,None]*side[:,None,:]
    mesh = loft(rings)
    parts.append((Mesh(mesh.vertices@M.T+O,mesh.faces), color))
stem = bezier([[0,0,-3.7],[-.3,.1,-1.5],[.3,0,1.5],[0,0,2.1]], t)
tube(stem, .055+.035*(1-t), [57,104,62])
for height, angle, length in [(-2.9,.2,1.6),(-2.3,3.3,1.8),(-1.65,1.9,1.5),(-1.,4.6,1.45),(-.4,.4,1.25),(.3,3.5,1.3),(.7,5.5,1.1)]:
    radial = np.array([np.cos(angle),np.sin(angle),0])
    origin = stem[np.argmin(abs(stem[:,2]-height))]
    leaf = origin + (length*t)[:,None]*radial + np.c_[0*t,0*t,.9*np.sin(t*2.1)]
    width = .005 + .21*np.sin(np.pi*t)**.9
    blade(leaf, width, .009, [61,117,69], [-radial[1],radial[0],0])
    tube(leaf+[0,0,.012], .006+.009*(1-t), [111,151,80])
branch = bezier([[.1,0,-.1],[.6,.2,.3],[1.15,.2,1.4],[1.05,.2,1.8]], t)
tube(branch, .025+.02*(1-t), [57,104,62])
bud = branch[-1] + np.c_[.15*np.sin(np.pi*t),0*t,.9*t]
bud_radius = .012+.19*np.sin(np.pi*t)**.8
tube(bud, bud_radius, [146,166,104])
for angle in np.arange(6)*np.pi/3:
    radial = np.array([np.cos(angle),np.sin(angle),0])
    seam = bud + bud_radius[:,None]*radial
    tube(seam, .004, [83,120,67])
for origin, size, lean in [([0,0,0],1.,0),([1.65,-.65,.9],.72,.52),([-1.6,.4,.25],.8,-.55)]:
    O = np.array(origin)
    c, s = np.cos(lean), np.sin(lean)
    M = size*np.array([[c,0,s],[0,1,0],[-s,0,c]])
    if lean != 0:
        attachment = np.array([0,0,-.7])
        base = np.array([0,0,2.05])@M.T+O
        stem_path = bezier([attachment,attachment+[0,0,.6],base+[0,0,-.7],base],t)
        saved_M, saved_O = M.copy(), O.copy()
        M, O = np.eye(3), np.zeros(3)
        tube(stem_path,.028+.025*(1-t),[57,104,62])
        M, O = saved_M, saved_O
    for index in range(6):
        angle = index*np.pi/3 + .045*np.sin(index*3)
        radial = np.array([np.cos(angle),np.sin(angle),0])
        reach = 2.15 if index%2 else 1.95
        petal = np.array([0,0,2.05]) + (reach*t)[:,None]*radial + np.c_[0*t,0*t,1.05*np.sin(np.pi*t)-(.30+.14*np.sin(index*2))*t]
        width = .018 + (.44 if index%2 else .51)*np.sin(np.pi*t)**.85*(1+.075*np.sin(9*np.pi*t+index))
        blade(petal, width, .017, [236,219,195] if index%2 else [244,229,207], [-radial[1],radial[0],0])
        for fraction in [-.65,0,.65]:
            tangent, normal, side = transport_frames(petal, initial_normal=[-radial[1],radial[0],0])
            vein = petal + fraction*width[:,None]*normal + .019*side
            tube(vein, .0035, [188,160,123])
    for angle in np.arange(6)*np.pi/3+.25:
        end = np.array([.53*np.cos(angle),.53*np.sin(angle),3.25])
        filament = bezier([[0,0,2.10],end*[.35,.35,.82],end+[0,0,-.3],end], t)
        tube(filament, .014+.008*(1-t), [159,166,98])
        anther = end + np.c_[.22*(t-.5)*np.cos(angle),.22*(t-.5)*np.sin(angle),.035*np.sin(np.pi*t)]
        tube(anther, .012+.052*np.sin(np.pi*t), [163,92,33])
    style = bezier([[0,0,2.1],[.02,0,2.6],[-.06,.04,3.1],[-.10,.08,3.45]],t)
    tube(style, .025, [141,156,99])
    for angle in np.arange(3)*2*np.pi/3:
        stigma = style[-1] + np.c_[.06*t*np.cos(angle),.06*t*np.sin(angle),.04*t]
        tube(stigma, .035-.017*t, [146,127,70])
