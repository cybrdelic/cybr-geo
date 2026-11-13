import numpy as np
from cybrgeo import Mesh, loft, sweep, extrude
parts = []
stone, iron, copper, wood = (191,174,145), (47,89,85), (119,157,145), (153,106,65)
# Generic solid and swept-section operations; no building/column/roof prefab.
def block(low, high, color):
    x,y,z = low; X,Y,Z = high
    ring = np.array([[x,y],[X,y],[X,Y],[x,Y]])
    sections = [np.c_[ring,np.full(4,h)] for h in (z,Z)]
    parts.append((loft(sections), color))
def member(path, width, color):
    profile = np.array([[-1,-1],[1,-1],[1,1],[-1,1]])*width/2
    parts.append((sweep(profile,np.array(path)),color))
# A continuous podium, four full-width risers, and a recessed central walk.
block((-7.5,-12.5,0),(7.5,12.5,.45),stone)
for i in range(4):
    block((-3.1,-14.5+i*.5,i*.1125),(3.1,-14+i*.5,(i+1)*.1125),stone)
block((-2.3,-12.5,.45),(2.3,12.5,.49),(214,204,181))
# Six bays per side. Window voids are real holes in the wall profile.
t = np.linspace(0,np.pi,33)
window = np.vstack(([-1.45,.95],[1.45,.95],np.c_[1.45*np.cos(t),2.2+1.45*np.sin(t)]))
wall = extrude([[-2,.45],[2,.45],[2,4.15],[-2,4.15]],.32,holes=[window])
for side in (-1,1):
    for y in np.arange(-10,11,4):
        v = np.c_[side*(6.84+wall.vertices[:,2]),y+wall.vertices[:,0],wall.vertices[:,1]]
        parts.append((Mesh(v,wall.faces),stone))
        path = np.c_[np.full(33,side*6.82),y+1.45*np.cos(t),2.2+1.45*np.sin(t)]
        member(path,.065,iron)
        for dy in (-.72,0,.72):
            top = 2.2+np.sqrt(1.45**2-dy**2)
            member([[side*6.82,y+dy,.95],[side*6.82,y+dy,top]],.045,iron)
        member([[side*6.82,y-1.45,2.2],[side*6.82,y+1.45,2.2]],.045,iron)
# Turned column profiles: bases, shaft entasis and capital in one connected loft.
a = np.linspace(0,2*np.pi,25)[:-1]
heights = [.45,.59,.62,.78,.84,1,1.1,4.9,5.04,5.12,5.2,5.32,5.4]
radii = [.36,.36,.29,.29,.22,.22,.17,.14,.2,.2,.31,.31,.36]
for x in (-4,4):
    for y in np.arange(-12,13,4):
        rings = [np.c_[x+r*np.cos(a),y+r*np.sin(a),np.full(len(a),z)] for z,r in zip(heights,radii)]
        parts.append((loft(rings),iron))
    member([[x,-12,5.4],[x,12,5.4]],.24,iron)
# Seven full-span barrel arches; the thinner inner arch makes each a curved truss.
theta = np.linspace(0,np.pi,65)
for y in np.arange(-12,13,4):
    outer = np.c_[4*np.cos(theta),np.full(65,y),5.4+4*np.sin(theta)]
    inner = np.c_[3.7*np.cos(theta),np.full(65,y),5.4+3.7*np.sin(theta)]
    member(outer,.14,iron)
    member(inner,.085,iron)
    for q in np.linspace(0,np.pi,17):
        member([[3.7*np.cos(q),y,5.4+3.7*np.sin(q)],[4*np.cos(q),y,5.4+4*np.sin(q)]],.045,iron)
    for q in np.linspace(0,np.pi,17)[:-1]:
        Q = q+np.pi/16
        member([[3.7*np.cos(q),y,5.4+3.7*np.sin(q)],[4*np.cos(Q),y,5.4+4*np.sin(Q)]],.033,iron)
    for side in (-1,1):
        member([[side*4,y,5.4],[side*6.84,y,4.15]],.13,iron)
        member([[side*4,y,4.85],[side*6.84,y,4.15]],.075,iron)
# Continuous purlins, and diagonal end-bay bracing, meet the rib centerlines.
for q in np.linspace(0,np.pi,13):
    member([[4*np.cos(q),-12,5.4+4*np.sin(q)],[4*np.cos(q),12,5.4+4*np.sin(q)]],.055,copper)
for side in (-1,1):
    for x in np.linspace(4,6.84,5):
        z = 5.4-(x-4)*1.25/2.84
        member([[side*x,-12,z],[side*x,12,z]],.06,copper)
    for y in (-12,8):
        member([[side*4,y,5.4],[side*4,y+4,3.7]],.065,iron)
        member([[side*4,y,3.7],[side*4,y+4,5.4]],.065,iron)
# End screens meet existing columns and leave a clear central entrance.
for y in (-12,12):
    for side in (-1,1):
        member([[side*1.6,y,.49],[side*1.6,y,5.4]],.12,iron)
        for z in (1.2,3.5,5.4):
            member([[side*1.6,y,z],[side*4,y,z]],.065,iron)
        for x in np.arange(1.9,4,.35):
            member([[side*x,y,1.2],[side*x,y,5.4]],.035,iron)
    member([[-1.6,y,3.5],[1.6,y,3.5]],.12,iron)
    for x in np.arange(-1.4,1.5,.35):
        member([[x,y,3.5],[x,y,5.4]],.035,iron)
# Radial fanlights close the end frames without filling their openings.
for y in (-12,12):
    for r in (.8,2.0):
        member(np.c_[r*np.cos(theta),np.full(65,y),5.4+r*np.sin(theta)],.055,copper)
    for q in np.linspace(0,np.pi,13):
        member([[.8*np.cos(q),y,5.4+.8*np.sin(q)],[4*np.cos(q),y,5.4+4*np.sin(q)]],.045,copper)
# Eight fitted timber benches, clear of both columns and central circulation.
for x in (-5.4,5.4):
    for y in (-8,-3,3,8):
        for dx in (-.32,.32):
            for dy in (-.85,.85):
                block((x+dx-.045,y+dy-.045,.45),(x+dx+.045,y+dy+.045,.9),iron)
        for dx in np.linspace(-.4,.4,5):
            block((x+dx-.075,y-1,.9),(x+dx+.075,y+1,.965),wood)
# Stair rails follow the actual riser slope and connect to a level landing rail.
for side in (-1,1):
    for y,z in ((-14.3,.1125),(-13.3,.3375),(-12.5,.45)):
        member([[side*2.95,y,z],[side*2.95,y,z+1]],.045,iron)
    member([[side*2.95,-14.3,1.1125],[side*2.95,-12.5,1.45],[side*2.95,-12,1.45]],.065,iron)
# An open-frame architectural study: glazing deliberately omitted for inspection.
