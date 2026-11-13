"""Sectional centrifugal pump, millimetres. Explanatory CAD, not a rated pump."""
import numpy as np
import cybrgeo as geo
scene=[]
def add(name,mesh,color):
    scene.append((name,mesh,color))
def circle(r,n=64):
    t=np.linspace(0,2*np.pi,n,endpoint=False)
    return np.c_[r*np.cos(t),r*np.sin(t)]
def ring(ri,ro,z,h):
    return geo.extrude(circle(ro),h,holes=[circle(ri)],z=z)
def move(mesh,x,y,z):
    T=np.eye(4)
    T[:3,3]=[x,y,z]
    return geo.transform(mesh,T)
def arc(ri,ro,a,b,z,h):
    t=np.linspace(a,b,72)
    p=np.r_[np.c_[ro*np.cos(t),ro*np.sin(t)],np.c_[ri*np.cos(t[::-1]),ri*np.sin(t[::-1])]]
    return geo.extrude(p,h,z=z)
steel=[153,169,178]
bronze=[199,137,62]
blue=[55,102,130]
dark=[61,72,80]
# Plate has machined shaft and fastener openings, not painted disks.
angles=np.linspace(0,2*np.pi,8,endpoint=False)
holes=[circle(7.2)]
for a in angles:
    holes.append(circle(2.2,20)+[57*np.cos(a),57*np.sin(a)])
add('rear flange',geo.extrude(circle(63),5,holes=holes,z=-5),blue)
add('shaft',geo.extrude(circle(6),83,z=-62),steel)
add('impeller hub',ring(6.05,13,-.5,17),bronze)
add('impeller rear shroud',ring(12.9,39,1,2),bronze)
# Seven curved blades are explicit swept planar boundaries.
u=np.linspace(0,1,48)
r=14+24*u
bend=.72*u**1.35
for a in np.linspace(0,2*np.pi,7,endpoint=False):
    center=np.c_[r*np.cos(a+bend),r*np.sin(a+bend)]
    tangent=np.gradient(center,axis=0)
    normal=np.c_[-tangent[:,1],tangent[:,0]]
    normal/=np.linalg.norm(normal,axis=1)[:,None]
    profile=np.r_[center+.8*normal,(center-.8*normal)[::-1]]
    add('back-swept blade',geo.extrude(profile,12,z=3),bronze)
# Partial front shroud is a declared section, exposing real passages.
add('sectioned front shroud',arc(17,39,0,np.pi,15,2),bronze)
# Spiral volute wall increases its collection area toward the discharge.
t=np.linspace(.08,2*np.pi-.08,180)
r=43+8*t/(2*np.pi)
wall=np.r_[np.c_[(r+5)*np.cos(t),(r+5)*np.sin(t)],np.c_[r[::-1]*np.cos(t[::-1]),r[::-1]*np.sin(t[::-1])]]
casing=geo.Solid.extrude(wall,19)
nozzle=geo.Solid.cylinder(10,34).rotate((0,0,0),(1,0,0),-90).move((48,0,9.5))
bore=geo.Solid.cylinder(7,40).rotate((0,0,0),(1,0,0),-90).move((48,-1,9.5))
add('volute with connected discharge',casing.union(nozzle).subtract(bore).mesh(),blue)
cover=geo.Solid.cylinder(62,4).move((0,0,19)).subtract(geo.Solid.cylinder(18,6).move((0,0,18)))
for a in angles:
    cover=cover.subtract(geo.Solid.cylinder(2.2,6).move((57*np.cos(a),57*np.sin(a),18)))
cover=cover.intersect(geo.Solid.box((130,70,10)).move((0,35,21)))
add('drilled sectioned cover',cover.mesh(),blue)
add('sectioned inlet neck',arc(18,22,0,np.pi,23,21),blue)
add('sectioned inlet flange',arc(18,29,0,np.pi,44,4),blue)
# Tangential discharge: concentric polygonal tube, rotated to point along +Y.
pipe=ring(7,10,0,34)
T=geo.rotation((1,0,0),-90)
T[:3,3]=[48,0,9.5]
# The discharge neck was fused and bored with the volute above.
flange=ring(7,17,31,4)
add('discharge flange',geo.transform(flange,T),blue)
# Bearing carrier connects both cartridges behind the wet housing.
add('sectioned bearing carrier',arc(14,18,0,np.pi,-55,50),blue)
for z in [-52,-16]:
    add('outer bearing race',ring(10.2,14,z,7),steel)
    add('inner bearing race',ring(6.05,7.9,z,7),steel)
    for a in np.linspace(0,2*np.pi,12,endpoint=False):
        lat=np.linspace(-np.pi/2+.01,np.pi/2-.01,10)
        lon=np.linspace(0,2*np.pi,14,endpoint=False)
        sections=np.stack([1.1*np.cos(lat)[:,None]*np.cos(lon),1.1*np.cos(lat)[:,None]*np.sin(lon),np.broadcast_to(1.1*np.sin(lat)[:,None],(10,14))],axis=-1)
        add('rolling element',move(geo.loft(sections,caps=True),9.05*np.cos(a),9.05*np.sin(a),z+3.5),steel)
add('seal stationary face',ring(6.2,12,-7,2),dark)
add('seal rotary face',ring(6.05,11,-2.5,2),dark)
add('shaft retaining collar',ring(6.05,9,-55,2),steel)
# Fasteners pass through real flange holes; top front half is sectioned away.
for a in angles:
    x,y=57*np.cos(a),57*np.sin(a)
    add('through bolt',move(geo.extrude(circle(1.9,20),34,z=-7),x,y,0),steel)
    add('hex head',move(geo.extrude(circle(3.6,6),3,z=-8),x,y,0),steel)
    if y>=-.01:
        add('hex nut',move(geo.extrude(circle(3.6,6),3,holes=[circle(2,20)],z=23),x,y,0),steel)
# Shaft coupling and keyed drive end, not an animated free-spinning decoration.
add('coupling sleeve',geo.extrude(circle(11),12,holes=[circle(6.05)]+[circle(1.3,16)+[8.5*np.cos(a),8.5*np.sin(a)] for a in np.linspace(0,2*np.pi,4,endpoint=False)],z=-68),dark)
for a in np.linspace(0,2*np.pi,4,endpoint=False):
    add('coupling axial screw',move(geo.extrude(circle(1.2,16),4,z=-69),8.5*np.cos(a),8.5*np.sin(a),0),steel)
