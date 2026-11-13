"""Authored 12-slot/14-pole outrunner inspection model; units mm.
Geometry only. Winding phasing, electromagnetic/thermal performance and fits are not certified.
"""
import numpy as np
import cybrgeo as geo

# Generic construction helpers, not stored motor parts.
def circle(radius, n=72):
    t=np.linspace(0,2*np.pi,n,endpoint=False)
    return np.c_[radius*np.cos(t),radius*np.sin(t)]

def annulus(inner,outer,z,height):
    return geo.extrude(circle(outer),height,holes=[circle(inner)],z=z)

def sector(inner,outer,start,end,z,height):
    t=np.linspace(start,end,12)
    return geo.extrude(np.vstack([np.c_[outer*np.cos(t),outer*np.sin(t)],np.c_[inner*np.cos(t[::-1]),inner*np.sin(t[::-1])]]),height,z=z)

def tube(path,radius,sides=8):
    path=np.asarray(path,float);tangent=np.gradient(path,axis=0);tangent/=np.linalg.norm(tangent,axis=1)[:,None]
    reference=np.tile([0.,0.,1.],(len(path),1));reference[np.abs(tangent[:,2])>.9]=[0,1,0]
    normal=np.cross(tangent,reference);normal/=np.linalg.norm(normal,axis=1)[:,None];binormal=np.cross(tangent,normal)
    t=np.linspace(0,2*np.pi,sides,endpoint=False)
    return geo.loft(path[:,None,:]+radius*(normal[:,None,:]*np.cos(t)[None,:,None]+binormal[:,None,:]*np.sin(t)[None,:,None]),caps=True)

def move(mesh,offset):
    T=np.eye(4);T[:3,3]=offset
    return geo.transform(mesh,T)

scene=[]
records=[]
def add(name,mesh,color,group):
    scene.append((name,mesh,color))
    records.append(dict(name=name,mesh=mesh,color=color,group=group))

def build():
    steel=[132,144,151];aluminium=[181,191,195];dark=[54,68,76];copper=[193,111,49]
    slots=12;poles=14;stator_outer=19.2;magnet_inner=20.1;magnet_outer=21.8;bell_outer=24.5
    # Base includes real screw holes, not drawn black disks.
    holes=[circle(7.2)]+[circle(1.65,24)+[16*np.cos(a),16*np.sin(a)] for a in np.arange(4)*np.pi/2]
    add('mounting base',geo.extrude(circle(22),4,holes=holes),dark,'fixed')
    add('bearing sleeve web',annulus(4.2,8.2,8,13),aluminium,'fixed')
    for z in [4,21]:
        add('bearing seat',annulus(7.2,8.2,z,3),aluminium,'fixed')
    # Shaft rotates inside bearing inner rings; sleeve bore is relieved between races.
    add('shaft',geo.extrude(circle(3,40),36,z=-3),steel,'rotor')
    for z in [6.5,22.5]:
        add('bearing inner race',annulus(3,4.2,z-1.5,3),steel,'rotor')
        add('bearing outer race',annulus(5.8,7.2,z-1.5,3),steel,'fixed')
        for a in np.linspace(0,2*np.pi,10,endpoint=False):
            # Local sphere, not a full annular rolling element.
            lat=np.linspace(-np.pi/2+.02,np.pi/2-.02,9);lon=np.linspace(0,2*np.pi,12,endpoint=False)
            sections=np.stack([.8*np.cos(lat)[:,None]*np.cos(lon),.8*np.cos(lat)[:,None]*np.sin(lon),np.broadcast_to(.8*np.sin(lat)[:,None],(9,12))],axis=-1)
            add('bearing ball',move(geo.loft(sections),[5*np.cos(a),5*np.sin(a),z]),aluminium,'fixed')
    # Each lamination is one connected toothed profile with central bore.
    outline=[]
    for a in np.arange(slots)*2*np.pi/slots:
        for r,d in [(11,-15),(11,-5),(17.6,-5),(17.6,-9),(stator_outer,-9),(stator_outer,9),(17.6,9),(17.6,5),(11,5),(11,15)]:
            angle=a+np.deg2rad(d);outline.append([r*np.cos(angle),r*np.sin(angle)])
    # Omit duplicated neighboring yoke seam vertices.
    outline=np.array(outline);keep=np.linalg.norm(outline-np.roll(outline,1,axis=0),axis=1)>1e-8;outline=outline[keep]
    for z in np.arange(9,21,.5):
        add('stator lamination',geo.extrude(outline,.46,holes=[circle(8.25,48)],z=z),steel,'stator')
    # Concentrated insulated copper loops surround each tooth in radial direction.
    for i,a in enumerate(np.arange(slots)*2*np.pi/slots):
        t=np.linspace(0,22*np.pi,265);radial=11.7+.45*t/(2*np.pi)
        local=np.c_[radial,2.05*np.sign(np.cos(t))*np.abs(np.cos(t))**.4,15+6.85*np.sign(np.sin(t))*np.abs(np.sin(t))**.4]
        R=geo.rotation((0,0,1),np.rad2deg(a))
        wire=geo.transform(tube(local,.18),R)
        add('coil '+str(i),wire,copper,'stator')
    # Magnet segments have a measurable radial air gap from stator tips.
    for i in range(poles):
        a=2*np.pi*i/poles;half=np.pi/poles*.88
        add('magnet '+str(i),sector(magnet_inner,magnet_outer,a-half,a+half,8.5,13),[103,118,126] if i%2 else [154,165,171],'rotor')
    add('rotor bell',annulus(magnet_outer,bell_outer,6.5,19),dark,'rotor')
    add('rotor upper rim',annulus(21.2,bell_outer,25.5,1.5),dark,'rotor')
    add('rotor shaft hub',annulus(3,5,24.5,4),aluminium,'rotor')
    for i in range(6):
        a=i*np.pi/3
        add('vented endplate spoke',sector(4.9,23,a-.12,a+.12,25.5,1.5),dark,'rotor')
    # Shaft retention washer and clip, shown as geometric features, not fit validation.
    add('retention washer',annulus(3,4.4,-1.1,.6),steel,'rotor')
    add('split retention clip',sector(2.9,3.8,.22,2*np.pi-.22,-1.7,.5),dark,'rotor')
    for a in np.arange(4)*np.pi/2:
        p=np.array([16*np.cos(a),16*np.sin(a),0])
        add('base screw head',move(geo.extrude(circle(2.5,6),1.7,z=-1.7),p),steel,'fixed')
    # External phase leads; actual coil termination network is not yet validated.
    for i in range(3):
        path=np.array([[8+i*1.2,-9,8],[8+i*1.2,-12,6],[10+i*1.2,-18,5],[14+i*1.2,-27,5]])
        add('phase lead '+str(i),tube(path,.65,10),[[61,64,68],[165,58,43],[193,155,46]][i],'fixed')
    return records

assembly=build()
