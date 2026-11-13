import numpy as np
from scipy.ndimage import gaussian_filter, map_coordinates
from cybrgeo import Mesh, heightfield, loft
rng = np.random.default_rng(41)
x = np.linspace(-26, 20, 145)
y = np.linspace(-20, 20, 129)
X, Y = np.meshgrid(x, y, indexing='ij')
noise = rng.normal(size=X.shape)
rough = 3*gaussian_filter(noise, 1.1) + 20*gaussian_filter(noise, 6) + 38*gaussian_filter(noise, 16)
valley = 1.4 + .062*Y**2 + .003*(X+7)**2
ridges = 2.8*abs(gaussian_filter(noise, 4))*np.sqrt(abs(Y))
H = valley + ridges + rough
for iteration in range(6):
    mean = gaussian_filter(H, .7)
    H = H + .22*np.clip(mean-H, -.6, .6)
H = np.where((X>1)&(X<19)&(abs(Y)<5.3), np.minimum(H,1.75), H)
H = H + .22*np.tanh(3*np.sin(2.4*H+.08*X))
terrain = heightfield(x, y, H)
scene = [('terrain', terrain, (157,148,121))]
water = heightfield(x, y, np.full_like(H, 11.0))
wet = (gaussian_filter(H, 3) < 10.5) & (X < 1.5-.025*Y**2-1.5)
faces = water.faces[np.all(wet.ravel()[water.faces], axis=1)]
used, remap = np.unique(faces, return_inverse=True)
water = Mesh(water.vertices[used], remap.reshape(-1,3), closed=False)
scene = scene + [('reservoir', water, (46,119,128))]
stations = np.linspace(-12, 12, 97)
curve = 1.5 - .025*stations**2
foot = map_coordinates(H, [(curve-x[0])/(x[1]-x[0]), (stations-y[0])/(y[1]-y[0])], order=1)
base = np.minimum(foot-.4, 11.5)
sections = np.array([[[c-1.6,s,b],[c+1.6,s,b],[c+.45,s,12],[c-.45,s,12]] for c,s,b in zip(curve,stations,base)])
dam = loft(sections)
scene = scene + [('dam', dam, (202,194,170))]
crest = sections.copy()
crest[:,:,2] = np.array([12,12,12.3,12.3])
crest[:,:,0] = curve[:,None] + np.array([-.65,.65,.65,-.65])
scene = scene + [('crest', loft(crest), (221,215,194))]
rails = []
for side in [-.62, .62]:
    strip = np.array([[[c+side-.045,s,12.3],[c+side+.045,s,12.3],[c+side+.045,s,12.8],[c+side-.045,s,12.8]] for c,s in zip(curve,stations)])
    rails = rails + [('parapet', loft(strip), (183,177,154))]
scene = scene + rails
chutes = []
for offset in [-3.1, 0, 3.1]:
    t = np.linspace(0,1,37)
    route = np.c_[1.9+10*t, np.full_like(t,offset), 11.7-9.4*(3*t*t-2*t*t*t)]
    trough = np.array([[[a,b-1.15,c],[a,b+1.15,c],[a,b+1.15,c+.22],[a,b-1.15,c+.22]] for a,b,c in route])
    chutes = chutes + [('spillway', loft(trough), (196,190,168))]
    for edge in [-1.1,1.1]:
        wall = trough.copy()
        wall[:,:,1] = offset+edge+np.array([-.08,.08,.08,-.08])
        wall[:,:,2] = route[:,2,None]+np.array([.22,.22,.9,.9])
        chutes = chutes + [('channel wall', loft(wall), (211,204,179))]
scene = scene + chutes
basin = np.array([[[11,-5,1.8],[18,-5,1.8],[18,5,1.8],[11,5,1.8]],[[11,-5,2.3],[18,-5,2.3],[18,5,2.3],[11,5,2.3]]])
scene = scene + [('stilling floor', loft(basin), (187,181,161))]
blocks = []
for a in [13,15,17]:
    for b in [-4,-2,0,2,4]:
        corners = np.array([[a-.32,b-.45,2.3],[a+.32,b-.45,2.3],[a+.32,b+.45,2.3],[a-.32,b+.45,2.3]])
        blocks = blocks + [('baffle', loft(np.array([corners,corners+[0,0,.85]])), (213,202,175))]
scene = scene + blocks
