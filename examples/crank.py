import numpy as np
import cybrgeo as geo

scene = []
guides = []
a = np.linspace(0,2*np.pi,48,endpoint=False)
outline = 0.34*np.c_[np.cos(a),np.sin(a)]
wheel = geo.extrude(outline,height=0.18,holes=[0.065*np.c_[np.cos(a),np.sin(a)]])
def link(start,end,radius,z,height):
    left = np.linspace(np.pi/2,3*np.pi/2,24)
    right = np.linspace(-np.pi/2,np.pi/2,24)
    border = np.concatenate([np.c_[start+radius*np.cos(left),radius*np.sin(left)],np.c_[end+radius*np.cos(right),radius*np.sin(right)]])
    holes = np.array([np.c_[q+0.065*np.cos(a),0.065*np.sin(a)] for q in [start,end]])
    return geo.extrude(border,height=height,holes=holes,z=z)
crank = link(0,0.9,0.13,0.18,0.16)
rod = link(0.9,3.0,0.11,0.36,0.14)
pin = geo.extrude(0.061*np.c_[np.cos(a),np.sin(a)],height=0.40,z=0.14)
pin_at_crank = geo.transform(pin,geo.rotation([0,0,1],0))
crank_pin_matrix = np.eye(4)
crank_pin_matrix[0,3] = 0.9
pin_at_rod = geo.transform(pin,crank_pin_matrix)
slider_pin_matrix = np.eye(4)
slider_pin_matrix[0,3] = 3.0
pin_at_slider = geo.transform(pin,slider_pin_matrix)
piston = geo.extrude(np.array([[2.72,-0.3],[3.28,-0.3],[3.28,0.3],[2.72,0.3]]),height=0.3,z=0.15,holes=[np.c_[3+0.065*np.cos(a),0.065*np.sin(a)]])
rail_a = geo.extrude(np.array([[1.5,-0.43],[4.2,-0.43],[4.2,-0.33],[1.5,-0.33]]),height=0.18,z=0.15)
rail_b = geo.extrude(np.array([[1.5,0.33],[4.2,0.33],[4.2,0.43],[1.5,0.43]]),height=0.18,z=0.15)
scene = [('axis pin',pin_at_crank,(75,89,96)),('crank pin',pin_at_rod,(75,89,96)),('slider pin',pin_at_slider,(75,89,96)),('wheel',wheel,(88,107,119)),('crank',crank,(185,111,65)),('connecting rod',rod,(164,178,184)),('slider',piston,(190,151,78)),('rail',rail_a,(64,79,87)),('rail',rail_b,(64,79,87))]
rig = geo.Kinematics().fixed('ground')
rig.revolute('crank','ground',[0,0,0])
rig.revolute('rod','crank',[0.9,0,0])
rig.prismatic('slider','ground',[1,0,0],limits=(-2,2))
rig.coincident('rod',[3,0,0],'slider',[3,0,0])
guess = {'rod':0,'slider':0}
errors = []
for angle in np.linspace(0,360,37):
    pose = rig.solve({'crank':float(angle)},guess=guess)
    guess = dict(pose.coordinates)
    errors.append(pose.closure_error)
    moving_crank = geo.transform(crank,pose.transforms['crank'])
    moving_rod = geo.transform(rod,pose.transforms['rod'])
    moving_slider = geo.transform(piston,pose.transforms['slider'])
    pin_at_rod_pose = geo.transform(pin_at_rod,pose.transforms['crank'])
    pin_at_slider_pose = geo.transform(pin_at_slider,pose.transforms['slider'])
    scene = [('axis pin',pin_at_crank,(75,89,96)),('crank pin',pin_at_rod_pose,(75,89,96)),('slider pin',pin_at_slider_pose,(75,89,96)),('wheel',wheel,(88,107,119)),('crank',moving_crank,(185,111,65)),('connecting rod',moving_rod,(164,178,184)),('slider',moving_slider,(190,151,78)),('rail',rail_a,(64,79,87)),('rail',rail_b,(64,79,87))]
assert max(errors)<1e-7
AZIMUTH = -90
ELEVATION = 65
