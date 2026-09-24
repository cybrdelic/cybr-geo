"""Original, deterministic human anatomy made from equations and authored landmarks.

There are no imported meshes, photographs, scanned textures, morph targets,
learned weights, or anatomy assets. Units are millimetres, Z up, face toward -Y.
This is an artistic anatomical model, not a measured or medically validated head.
"""
from __future__ import annotations

from dataclasses import dataclass, asdict
import numpy as np
from scipy.interpolate import PchipInterpolator
from scipy.ndimage import gaussian_filter1d
from mechanism_lab.core import Assembly, Material, Part, View


@dataclass(frozen=True)
class FaceParameters:
    seed: int = 271828
    eye_spacing: float = 62.5
    eye_height: float = 30.0
    eye_width: float = 27.2
    eye_opening: float = 8.6
    eye_tilt: float = 0.62
    nose_projection: float = 17.4
    nose_width: float = 1.08
    mouth_width: float = 52.5
    upper_lip_fullness: float = 0.94
    lower_lip_fullness: float = 0.98
    jaw_width: float = 0.99
    skull_width: float = 0.995
    cheek_width: float = 1.025
    chin_width: float = 0.93
    brow_weight: float = 1.0
    skin_relief: float = .0025
    eyelid_closure: float = 0.10


ZMIN, ZMAX = -180., 144.
SKIN, LID, CAVITY, SCLERA, IRIS, PUPIL, CORNEA, HAIR = range(8)


def gaussian(x, z, cx, cz, sx, sz):
    return np.exp(-.5*((x-cx)/sx)**2-.5*((z-cz)/sz)**2)


def smoothstep(a, b, x):
    t=np.clip((x-a)/(b-a),0.,1.)
    return t*t*(3.-2.*t)


def unit(v):
    return v/np.maximum(np.linalg.norm(v,axis=-1,keepdims=True),1e-12)


def smooth_minimum(a,b,width):
    h=np.maximum(width-np.abs(a-b),0)/width
    return np.minimum(a,b)-h*h*width*.25


class Anatomy:
    """Continuous head surface with locally defined orbital and facial forms."""
    def __init__(self, p=FaceParameters()):
        self.p=p
        z=np.array([-180,-166,-145,-125,-103,-89,-80,-73,-62,-49,-30,
                    -10,15,40,65,90,115,133,141,144.])
        rx=np.array([171,140,83,49,43,42,44,50,61,68,72,73,74,75,74,
                     70,59,39,18,0.])
        front=np.array([-50,-44,-28,-18,-21,-33,-50,-63,-65,-66,-66,
                        -65,-66,-64,-63,-58,-41,-18,4,17.])
        back=np.array([85,85,68,59,52,51,52,55,61,71,84,93,95,95,92,
                       83,67,49,32,17.])
        # Smooth the interpolated profiles themselves; isolated control points
        # must not leave rings or discontinuous curvature across the sculpture.
        zz=np.linspace(ZMIN,ZMAX,4097)
        self.profile=[]
        for a in (rx,front,back):
            y=gaussian_filter1d(PchipInterpolator(z,a)(zz),8.,mode='nearest')
            self.profile.append(PchipInterpolator(zz,y))

    def section(self,z):
        rx,front,back=[f(np.clip(z,ZMIN,ZMAX)) for f in self.profile]
        jaw=1+(self.p.jaw_width-1)*np.exp(-((z+55)/34)**2)
        skull=1+(self.p.skull_width-1)*smoothstep(-15,70,z)
        cheek=1+(self.p.cheek_width-1)*np.exp(-((z-2)/31)**2)
        chin=1+(self.p.chin_width-1)*np.exp(-((z+76)/16)**2)
        temple=.972 + .028*(1-np.exp(-((z-48)/23)**2))
        mandible=1+.014*np.exp(-((z+52)/18)**2)
        return rx*jaw*skull*cheek*chin*temple*mandible,front,back

    def mouth(self,x):
        a=self.p.mouth_width/2
        q=np.clip(np.abs(x)/a,0,1)
        line=-39.2-.9*np.exp(-(x/5.4)**2)+.85*q*q
        span=np.maximum(1-q*q,0)
        upper=self.p.upper_lip_fullness*(4.35+2.35*np.exp(-((np.abs(x)-6.2)/4.4)**2)
               -.82*np.exp(-(x/2.4)**2))*span**.70
        lower=self.p.lower_lip_fullness*6.85*span**.78
        return line,upper,lower

    def eye(self,side):
        return np.array([side*self.p.eye_spacing/2,-47.25,
                         self.p.eye_height+(.16 if side<0 else -.10)])

    def eye_opening(self,x,side):
        c=self.eye(side)
        half=self.p.eye_width/2
        q=(x-c[0])*side/half
        shape=np.maximum(1-q*q,0)
        closure=self.p.eyelid_closure
        tilt=self.p.eye_tilt*q
        center=c[2]+tilt-.35*shape
        opening=self.p.eye_opening*(1-closure)
        upper=center+.54*opening*shape**.72
        lower=center-.46*opening*shape**.78
        # The medial and lateral canthi close before the sphere ends, eliminating
        # the black corner wedges present in v10.
        return q,upper,lower

    def deformation(self,x,z):
        """Front-facing displacement in Y; negative displacement projects out."""
        d=np.zeros(np.broadcast_shapes(np.shape(x),np.shape(z)))
        ax=np.abs(x)
        # Zygoma, malar volume, submalar hollow, temporalis and masseter.
        for s in (-1,1):
            d-=3.9*gaussian(x,z,s*43,4,16,15)
            d-=2.5*gaussian(x,z,s*48,-25,14,21)
            d+=2.0*gaussian(x,z,s*51,-12,11,11)
            d+=2.0*gaussian(x,z,s*64,40,9,20)
            c=self.eye(s)
            d+=2.6*gaussian(x,z,c[0],c[2],16.6,9.4)
            d-=4.1*self.p.brow_weight*gaussian(x,z,s*29,47,18.2,6.2)
            d-=1.1*gaussian(x,z,s*34,16,18,4.5)
            # Tear trough, softened into the cheek rather than a painted line.
            trough=16.5+.1*(ax-23.)
            d+=.7*np.exp(-((z-trough)/1.4)**2)*np.exp(-((x-s*26)/13)**2)
        d-=2.0*gaussian(x,z,0,44,9,13)
        # Nasal bridge, dorsum, tip lobule, alar cartilage and columella.
        d-=7.6*gaussian(x,z,.25,15,6.3,19)
        d-=4.6*gaussian(x,z,.35,1.5,8.5,13.5)
        d-=self.p.nose_projection*gaussian(x,z,.25,-7.8,8.0,7.2)
        for s in (-1,1):
            alar_x=s*(10.8*self.p.nose_width)
            d-=7.2*gaussian(x,z,alar_x,-13.6,5.7,5.3)
            # Distinct alar bank, alar-facial groove and nostril sill.
            d+=1.05*gaussian(x,z,s*(16.0*self.p.nose_width),-13.5,2.4,5.0)
            d-=1.15*gaussian(x,z,s*(13.8*self.p.nose_width),-10.8,2.8,3.4)
            d-=1.05*gaussian(x,z,s*(8.2*self.p.nose_width),-17.0,3.0,2.0)
        d-=4.4*gaussian(x,z,.2,-17.9,3.2,3.8)
        d+=1.25*gaussian(x,z,0,-21.0,6.2,2.7)
        # Orbicularis/muzzle, philtrum pillars, chin and mental crease.
        d-=3.8*gaussian(x,z,0,-38,23,15)
        d-=.7*gaussian(x,z,-3.2,-27.4,1.65,5.5)
        d-=.75*gaussian(x,z,3.5,-27.2,1.65,5.5)
        d+=.28*gaussian(x,z,.1,-27.2,1.8,5.5)
        d+=1.8*gaussian(x,z,0,-57,17,3.4)
        d-=3.9*gaussian(x,z,.25,-69,21,9)
        # Actual vermilion volumes are part of the head mesh.
        line,upper,lower=self.mouth(x)
        inside=ax<self.p.mouth_width/2
        t=np.where(z>=line,(z-line)/np.maximum(upper,.001),
                   (line-z)/np.maximum(lower,.001))
        t=np.clip(t,0,1)
        upper_mask=(z>=line)
        fullness=np.where(upper_mask,self.p.upper_lip_fullness,self.p.lower_lip_fullness)
        lip=fullness*(1-t**1.55)*(2.35+3.35*np.sin(np.pi*t))
        lip_region=inside*((z>=line-lower)&(z<=line+upper))
        d-=lip*lip_region
        # A neutral closed mouth is a crease in continuous skin, not a literal
        # black cut through the head mesh.
        crease=np.exp(-((z-line)/.46)**2)*np.maximum(1-(x/(self.p.mouth_width*.485))**8,0)
        d+=.34*crease
        for s in (-1,1):
            d+=1.15*gaussian(x,z,s*(self.p.mouth_width*.485),-39.2,2.5,3.0)
            d+=.28*gaussian(x,z,s*(self.p.mouth_width*.53),-41.2,3.6,5.2)
        # Nasolabial furrow and its gently convex lateral bank.
        fold=14.7+.34*np.clip(-z-17,0,34)
        extent=smoothstep(-52,-42,z)*(1-smoothstep(-18,-11,z))
        d+=.24*np.exp(-((ax-fold)/1.4)**2)*extent
        d-=.4*np.exp(-((ax-fold-3.8)/3.8)**2)*extent
        # Shallow forehead creases: relief, not stripes in the albedo.
        for height,amp in [(67,.09),(78,.075),(89,.05)]:
            curve=height+.0016*x*x+.28*np.sin(.08*x)
            d+=amp*np.exp(-((z-curve)/.42)**2)*np.exp(-(x/47)**6)
        # Laryngeal and sternocleidomastoid contours, clavicles and sternal notch.
        d-=2.7*gaussian(x,z,0,-108,8,6)
        muscle=18.+.34*np.clip(-z-95,0,65)
        neck=np.exp(-((z+123)/27)**4)
        d-=2.7*np.exp(-((ax-muscle)/5.5)**2)*neck
        clavicle=-149.+.11*ax
        d-=2.0*np.exp(-((z-clavicle)/3.6)**2)*np.exp(-((ax-62)/49)**4)
        d+=2.3*gaussian(x,z,0,-146,8,6)
        d-=.45*gaussian(x,z,-37,-7,20,19)
        # Low-frequency deterministic asymmetry prevents a perfectly mirrored
        # mannequin face without reading any external identity data.
        seed_phase=(self.p.seed % 104729)/104729.*(2*np.pi)
        d+=.24*np.sin(seed_phase)*np.tanh(x/28.)*gaussian(x,z,0,-4,66,78)
        d+=.16*np.cos(seed_phase*.73)*np.tanh(x/21.)*gaussian(x,z,0,-52,58,36)
        # Analytic low-amplitude skin relief in world coordinates, no asset input.
        d+=self.p.skin_relief*(np.sin(x*2.9+np.sin(z*1.3))*np.sin(z*3.2)
                             +.45*np.sin(x*7.1+z*5.3))
        return d

    def base_front(self,x,z):
        rx,front,back=self.section(z)
        q=np.clip(x/np.maximum(rx,.01),-.999999,.999999)
        c=np.sqrt(np.maximum(1-q*q,0))
        mid=(front+back)/2
        base=mid-(mid-front)*c*(1+.28*q*q)
        # Deformations fade before the side seam of the parametric skull.
        return base+self.deformation(x,z)*smoothstep(0,.55,c)

    def lid_skin(self,x,z,base):
        """Eyelid relief is part of the continuous head, with no overlay shell."""
        y=np.array(base,copy=True)
        for side in (-1,1):
            c=self.eye(side);lx=(x-c[0])*side;lz=z-c[2]
            sphere=1-(lx/12.55)**2-(lz/11.85)**2
            cap=c[1]-11.92*np.sqrt(np.maximum(sphere,0))
            cap-=1.05*np.exp(-((lx*lx+lz*lz)/(5.0**2))**2)
            target=np.where(sphere>0,smooth_minimum(base,cap-.22,4.2),base)
            ellipse=(lx/17.0)**2+((lz-.85*lx/17.0)/np.where(lz>=0,9.1,8.2))**2
            blend=1-smoothstep(.72,1.,ellipse)
            y+=(target-base)*blend
            q=np.clip(lx/(self.p.eye_width/2),-1,1)
            shape=np.maximum(1-q*q,0)
            _,upper,lower=self.eye_opening(x,side)
            # Real upper/lower lid folds follow the palpebral opening rather
            # than an arbitrary horizontal seam.
            y+=.12*np.exp(-((z-upper-2.3)/.52)**2)*shape*blend
            y-=.10*np.exp(-((z-lower+1.55)/.58)**2)*shape*blend
        return y

    def front(self,x,z):
        return self.lid_skin(x,z,self.base_front(x,z))

    def point(self,theta,z):
        rx,front,back=self.section(z)
        s=np.sin(theta);c=np.cos(theta)
        x=rx*s
        mid=(front+back)/2
        y=np.where(c>=0,mid-(mid-front)*c*(1+.28*s*s),mid-(back-mid)*c)
        y+=self.deformation(x,z)*smoothstep(0,.55,c)
        y+=(self.lid_skin(x,z,y)-y)*smoothstep(.3,.7,c)
        return np.stack(np.broadcast_arrays(x,y,z),axis=-1)

    def uv(self,vertices):
        x,y,z=np.asarray(vertices).T
        rx,f,b=self.section(z)
        a=np.arcsin(np.clip(x/np.maximum(rx,.01),-1,1))
        a=np.where(y>(f+b)/2,np.where(x>=0,np.pi-a,-np.pi-a),a)
        return np.column_stack([.5+a/(2*np.pi),(z-ZMIN)/(ZMAX-ZMIN)])


def surface_part(name,vertices,faces,material=SKIN,uv=None,role=''):
    v=np.ascontiguousarray(vertices,dtype=np.float64).reshape(-1,3)
    f=np.ascontiguousarray(faces,dtype=np.int32).reshape(-1,3)
    area=np.cross(v[f[:,1]]-v[f[:,0]],v[f[:,2]]-v[f[:,0]])
    keep=np.linalg.norm(area,axis=1)>1e-10
    f=f[keep];area=area[keep]
    n=np.zeros_like(v)
    for i in range(3):np.add.at(n,f[:,i],area)
    n=unit(n)
    p=Part(name,v,f,n,material=material,group='anatomy',
           provenance='original-procedural',role=role or name,
           tags=('original-procedural','no-scan','deterministic'))
    p.portrait_uv=np.zeros((len(v),2)) if uv is None else np.asarray(uv,float).reshape(-1,2)
    return p


def grid_faces(rows,cols,reverse=False):
    a=(np.arange(rows-1)[:,None]*cols+np.arange(cols-1)[None,:]).ravel()
    f=np.concatenate([np.stack([a,a+1,a+cols],1),
                      np.stack([a+1,a+cols+1,a+cols],1)])
    return f[:,::-1] if reverse else f


def head_mesh(a,quality):
    rows,cols=(380,601) if quality=='preview' else (800,1201)
    theta=np.linspace(-np.pi,np.pi,cols)
    zz=np.linspace(ZMIN,ZMAX-.01,rows)
    tt,z=np.meshgrid(theta,zz)
    v=a.point(tt,z).reshape(-1,3)
    uv=np.column_stack([((tt+np.pi)/(2*np.pi)).ravel(),((z-ZMIN)/(ZMAX-ZMIN)).ravel()])
    f=grid_faces(rows,cols)
    center=v[f].mean(axis=1);x,y,z=center.T
    remove=np.zeros(len(f),bool)
    for side in (-1,1):
        # Dedicated eyelid surfaces replace the generic head mesh throughout
        # the orbital patch. This avoids a literal aperture cut into an otherwise
        # egg-shaped surface and gives the lids their own curvature/thickness.
        ec=a.eye(side);lx=(x-ec[0])*side;lz=z-ec[2]
        orbit=(lx/19.0)**2+((lz-.55*lx/19.0)/np.where(lz>=0,11.8,9.8))**2
        remove|=(orbit<1.015)&(y<0)
        # Open apertures lead to separately modeled recessed nasal vestibules.
        nx=side*(10.2*a.p.nose_width)+.20;nz=-16.4
        du=x-nx;dz=z-nz
        ur=du+side*.42*dz;vr=dz-side*.10*du
        remove|=((ur/3.05)**2+(vr/1.12)**2<1)&(y<-65.5)
    # Closed lips remain continuous geometry. Mouth depth is represented by
    # the analytic crease in Anatomy.deformation(), not by deleted triangles.
    f=f[~remove]
    p=surface_part('Sculpted_head_neck_and_shoulders',v,f,uv=uv,
                   role='Continuous authored skull, jaw, cheeks, nose, lips, neck and shoulders')
    # Average duplicate meridian normals, preserving the texture seam.
    n=p.normals.reshape(rows,cols,3)
    seam=unit(n[:,0]+n[:,-1]);n[:,0]=seam;n[:,-1]=seam
    return [p]


def ellipsoid(name,center,radii,material,nu=128,nv=80,iris_cut=False):
    theta=np.linspace(-np.pi,np.pi,nu+1)
    phi=np.linspace(.0001,np.pi-.0001,nv+1)
    tt,pp=np.meshgrid(theta,phi)
    q=np.stack([np.sin(pp)*np.sin(tt),-np.sin(pp)*np.cos(tt),np.cos(pp)],-1)
    v=(q*np.array(radii)+center).reshape(-1,3)
    f=grid_faces(nv+1,nu+1,reverse=True)
    if iris_cut:
        mid=v[f].mean(axis=1)-center
        f=f[~((mid[:,0]**2+mid[:,2]**2<5.7**2)&(mid[:,1]<0))]
    uv=np.column_stack([.5+(v[:,0]-center[0])/(2*radii[0]),
                        .5+(v[:,2]-center[2])/(2*radii[2])])
    return surface_part(name,v,f,material,uv)


def disk(name,center,radius,material,yfunc,rmin=0.,nr=32,nt=192):
    r=np.linspace(rmin,radius,nr)
    theta=np.linspace(0,2*np.pi,nt+1)
    rr,tt=np.meshgrid(r,theta,indexing='ij')
    x=rr*np.cos(tt);z=rr*np.sin(tt)
    y=yfunc(rr)
    v=np.stack([x+center[0],y+center[1],z+center[2]],-1).reshape(-1,3)
    uv=np.column_stack([.5+x.ravel()/(2*radius),.5+z.ravel()/(2*radius)])
    return surface_part(name,v,grid_faces(nr,nt+1,reverse=True),material,uv)


def tube_collection(name,curves,radii,material,a=None,sides=5):
    vs=[];fs=[];offset=0
    for curve,radius in zip(curves,radii):
        c=np.asarray(curve,float)
        tangent=unit(np.gradient(c,axis=0))
        ref=np.tile([0.,1.,0.],(len(c),1))
        b=unit(np.cross(tangent,ref));n=unit(np.cross(b,tangent))
        angles=np.linspace(0,2*np.pi,sides,endpoint=False)
        radius=np.broadcast_to(radius,(len(c),))
        v=c[:,None,:]+radius[:,None,None]*(np.cos(angles)[None,:,None]*b[:,None,:]
                                          +np.sin(angles)[None,:,None]*n[:,None,:])
        i=np.arange((len(c)-1)*sides).reshape(-1,sides)
        j=np.roll(i,-1,axis=1)
        f=np.concatenate([np.stack([i,j,i+sides],-1).reshape(-1,3),
                          np.stack([j,j+sides,i+sides],-1).reshape(-1,3)])
        # Close tube tips; even fine lashes have real, finite geometry.
        if sides>2:
            cap0=np.array([[0,k+1,k] for k in range(1,sides-1)])
            cap1=np.array([[(len(c)-1)*sides,(len(c)-1)*sides+k,
                           (len(c)-1)*sides+k+1] for k in range(1,sides-1)])
            f=np.concatenate([f,cap0,cap1])
        vs.append(v.reshape(-1,3));fs.append(f+offset);offset+=v.size//3
    v=np.concatenate(vs)
    return surface_part(name,v,np.concatenate(fs),material,a.uv(v) if a else None)


def eyelid_patch(a,side):
    """Upper/lower organic lid surfaces replacing the generic orbital head mesh.

    Each patch has an outer boundary on the authored facial surface and an inner
    boundary on the ocular surface. Intermediate rows bow outward to model
    orbicularis/pretarsal lid volume. No imported topology is used.
    """
    ec=a.eye(side)
    q=np.linspace(-.995,.995,201)
    shape=np.maximum(1-q*q,0)
    inner_x=ec[0]+side*q*(a.p.eye_width/2)
    _,inner_upper,inner_lower=a.eye_opening(inner_x,side)
    # A fully closed blink needs finite overlap at the lid seam so a ray cannot
    # leak through a zero-width shared boundary. The overlap vanishes rapidly
    # for ordinary open-eye poses.
    overlap=.42*(a.p.eyelid_closure**6)*shape**.72
    inner_upper=inner_upper-overlap
    inner_lower=inner_lower+overlap

    # Outer lid boundaries follow a wider asymmetric orbital ellipse.
    outer_x=ec[0]+side*q*18.9
    outer_upper=ec[2]+.65*a.p.eye_tilt*q+10.7*shape**.56
    outer_lower=ec[2]+.40*a.p.eye_tilt*q-8.8*shape**.60

    parts=[]
    for name,inner_z,outer_z,bulge in [
        ('upper',inner_upper,outer_upper,.88),
        ('lower',inner_lower,outer_lower,.42),
    ]:
        outer_y=a.base_front(outer_x,outer_z)
        dx=inner_x-ec[0];dz=inner_z-ec[2]
        sph=np.maximum(1-(dx/14.15)**2-(dz/12.25)**2,0)
        inner_y=ec[1]-12.22*np.sqrt(sph)-.035

        rows=11
        t=np.linspace(0,1,rows)[:,None]
        ease=t*t*(3-2*t)
        # Outer -> inner boundary with a slight inward convergence in X/Z.
        xx=outer_x[None,:]*(1-ease)+inner_x[None,:]*ease
        zz=outer_z[None,:]*(1-ease)+inner_z[None,:]*ease
        yy=outer_y[None,:]*(1-ease)+inner_y[None,:]*ease
        # Pretarsal/preseptal soft-tissue volume. Negative Y is outward.
        arch=np.sin(np.pi*t)*shape[None,:]**.42
        yy-=bulge*arch
        # Canthi collapse smoothly into their facial attachment.
        canthus=shape[None,:]**.28
        yy=outer_y[None,:]+(yy-outer_y[None,:])*canthus

        v=np.stack([xx,yy,zz],-1).reshape(-1,3)
        f=grid_faces(rows,len(q),reverse=(name=='lower'))
        p=surface_part(
            ('Left' if side<0 else 'Right')+f'_procedural_{name}_eyelid',
            v,f,SKIN,a.uv(v),
            role='Original procedural eyelid surface from orbital boundary to eyeball')
        parts.append(p)
    return parts


def eyelid_bridge(a,side):
    """Finite eyelid thickness wrapping from facial skin to the eyeball.

    The strip follows the actual spherical ocular surface all the way inward.
    Multiple rows avoid the v10/v12 black canthus cavity caused by a short,
    nearly-flat bridge that stopped before reaching the eye.
    """
    c=a.eye(side)
    q=np.linspace(-.992,.992,181)
    x=c[0]+side*q*(a.p.eye_width/2)
    _,upper,lower=a.eye_opening(x,side)
    parts=[]
    for name,z in [('upper',upper),('lower',lower)]:
        outer_y=a.front(x,z)-.018
        dx=x-c[0];dz=z-c[2]
        sphere=np.maximum(1-(dx/14.15)**2-(dz/12.25)**2,0)
        ocular_y=c[1]-12.22*np.sqrt(sphere)
        # Inner lid approaches the ocular surface. The slight vertical inset
        # creates a real posterior lid edge rather than coplanar surfaces.
        inset_z=z+(-.20 if name=='upper' else .20)
        rows=7
        t=np.linspace(0,1,rows)[:,None]
        ease=t*t*(3-2*t)
        xx=np.broadcast_to(x,(rows,len(x)))
        zz=z[None,:]*(1-ease)+inset_z[None,:]*ease
        yy=outer_y[None,:]*(1-ease)+(ocular_y[None,:]-.045)*ease
        # Near the canthi, taper thickness smoothly into the facial attachment.
        canthus=np.maximum(1-q*q,0)[None,:]**.35
        yy=outer_y[None,:]+(yy-outer_y[None,:])*canthus
        v=np.stack([xx,yy,zz],-1).reshape(-1,3)
        f=grid_faces(rows,len(q),reverse=(name=='lower'))
        parts.append(surface_part(
            ('Left' if side<0 else 'Right')+f'_eyelid_{name}_thickness',
            v,f,SKIN,a.uv(v),
            role='Procedural eyelid thickness continuously wrapping to ocular surface'))
    return parts

def closed_blink_seal(a,side):
    """Finite skin seal used only for near-complete eyelid closure."""
    if a.p.eyelid_closure < .94:
        return None
    ec=a.eye(side)
    q=np.linspace(-.985,.985,201)
    shape=np.maximum(1-q*q,0)
    x=ec[0]+side*q*(a.p.eye_width/2)
    _,upper,lower=a.eye_opening(x,side)
    center=(upper+lower)*.5
    half=.34*shape**.65
    dx=x-ec[0];dz=center-ec[2]
    sph=np.maximum(1-(dx/14.15)**2-(dz/12.25)**2,0)
    ocular_y=ec[1]-12.22*np.sqrt(sph)
    # Closed lid skin sits slightly anterior to the cornea.
    y=ocular_y-.52-.18*shape
    v=np.stack([
        np.column_stack([x,y,center-half]),
        np.column_stack([x,y-.03,center+half])
    ],axis=0).reshape(-1,3)
    return surface_part(
        ('Left' if side<0 else 'Right')+'_closed_eyelid_seal',
        v,grid_faces(2,len(q)),SKIN,a.uv(v),
        role='Procedural finite seal for fully closed blink')


def eyelids(a,side):
    """Continuous upper/lower moist margins following the analytic eye opening."""
    c=a.eye(side)
    q=np.linspace(-.995,.995,129)
    x=c[0]+side*q*(a.p.eye_width/2)
    _,upper,lower=a.eye_opening(x,side)
    def ocular_y(xx,zz):
        dx=xx-c[0];dz=zz-c[2]
        sph=np.maximum(1-(dx/14.15)**2-(dz/12.25)**2,0)
        return c[1]-12.22*np.sqrt(sph)-.055
    upper_curve=np.column_stack([x,ocular_y(x,upper),upper])
    lower_curve=np.column_stack([x[::-1],ocular_y(x[::-1],lower[::-1]),lower[::-1]])
    margin=np.concatenate([upper_curve,lower_curve[1:]],axis=0)
    name=('Left' if side<0 else 'Right')+'_eyelids'
    wet=tube_collection(name+'_wet_margin',[upper_curve,lower_curve],
                        [np.full(len(upper_curve),.038),np.full(len(lower_curve),.025)],
                        LID,a,6)
    return wet,margin


def nostril_rim(a,side):
    """Analytic alar rim surrounding the recessed nostril slit."""
    cx=side*(10.2*a.p.nose_width)+.20;cz=-16.4
    theta=np.linspace(0,2*np.pi,161)
    # Outer and inner rotated ellipses form a thin skin annulus.
    rings=[]
    for scale,depth in [(1.07,-.02),(.80,.72)]:
        ur=3.22*scale*np.cos(theta);vr=1.23*scale*np.sin(theta)
        du=ur-side*.42*vr;dz=vr+side*.10*ur
        x=cx+du;z=cz+dz
        y=a.front(x,z)+depth
        rings.append(np.column_stack([x,y,z]))
    v=np.stack(rings,axis=0).reshape(-1,3)
    return surface_part(
        ('Left' if side<0 else 'Right')+'_nostril_alar_rim',
        v,grid_faces(2,len(theta)),SKIN,a.uv(v),
        role='Procedural alar rim around recessed nostril opening')


def nasal_cavity(a,side):
    """Recessed rotated slit matching the alar opening, not a circular black disk."""
    cx=side*(10.2*a.p.nose_width)+.20;cz=-16.4
    theta=np.linspace(0,2*np.pi,129)
    r=np.linspace(0,1,24)[:,None]
    ur=3.18*r*np.cos(theta);vr=1.20*r*np.sin(theta)
    u=ur-side*.42*vr;v=vr+side*.10*ur
    x=cx+u;z=cz+v
    y=a.front(x,z)+2.7+2.0*(1-r*r)
    vertices=np.stack([x,y,z],-1).reshape(-1,3)
    return surface_part(('Left' if side<0 else 'Right')+'_nasal_vestibule',
                        vertices,grid_faces(24,129,reverse=True),CAVITY,a.uv(vertices))


def mouth_interior(a):
    x=np.linspace(-25,25,180);w=np.linspace(-1,1,12)
    xx,ww=np.meshgrid(x,w)
    line,upper,lower=a.mouth(xx)
    z=line+ww*(1.1*np.maximum(1-(xx/25)**2,0)**.6+.04)
    y=a.front(xx,z)+3.4+1.5*(1-ww*ww)
    v=np.stack([xx,y,z],-1).reshape(-1,3)
    return surface_part('Recessed_closed_mouth_cavity',v,grid_faces(12,180),CAVITY,a.uv(v))


def ear(a,side):
    """Polar cartilage shell: helix, scapha, antihelix, concha and lobule."""
    center=np.array([side*73.0,5.5,4.0])
    normal=np.array([side*.85,-.5268,0.]);horizontal=np.array([side*.5268,.85,0.])
    theta=np.linspace(0,2*np.pi,257);r=np.linspace(0,1,100)[:,None]
    t=np.sin(theta);c=np.cos(theta)
    u=12.8*r*c*(1+.14*t)
    v=29.5*r*t
    u+=1.7*(v/30.)
    # Elevation of the outer pinna, with a rolled rim and an inset central bowl.
    h=1.8+2.7*np.exp(-((r-.85)/.073)**2)
    h=np.broadcast_to(h,u.shape).copy()
    h-=3.2*np.exp(-((u+1.3)/6.6)**2-((v+3)/11)**2)
    h-=1.1*np.exp(-((r-.64)/.12)**2)*smoothstep(-18,4,v)
    h+=1.8*np.exp(-((u-3)/2.8)**2-((v-4)/14)**2)
    h+=1.2*np.exp(-((u+2.5-.22*v)/2.1)**2-((v-15)/9)**2)
    h+=2.5*np.exp(-((u+6.0)/2.4)**2-((v+5)/4.5)**2)  # tragus
    h+=1.7*np.exp(-((u-1)/4.)**2-((v+22)/7.0)**2)    # lobule
    h-=2.2*np.exp(-((u+1.8)/2.5)**2-((v+7)/3.8)**2) # canal recess
    h-=2.5*smoothstep(.93,1,r)
    vertices=center+u[...,None]*horizontal+v[...,None]*[0,0,1]+h[...,None]*normal
    vertices=vertices.reshape(-1,3)
    f=grid_faces(100,257,reverse=side>0)
    # A nondegenerate local chart samples a quiet region of the procedural skin
    # atlas. Projecting an ear onto the skull would collapse its outside UVs.
    uv=np.column_stack([.69+u.ravel()/420.,.47+v.ravel()/(ZMAX-ZMIN)])
    p=surface_part(('Left' if side<0 else 'Right')+'_sculpted_pinna',vertices,f,SKIN,uv)
    if np.mean(p.normals@normal)<0:
        p=surface_part(p.name,vertices,f[:,::-1],SKIN,uv)
    return p


def eyebrow_and_lashes(a,side,margin,rng):
    curves=[];radii=[]
    for _ in range(550):
        q=rng.beta(1.35,1.3)
        x=side*(15.4+39*q)
        z=44.0+4.8*np.sin(np.pi*q*.91)-2.7*q+rng.normal(0,1.3)*(1-.5*q)
        length=rng.uniform(3.4,6.0)*(1-.26*q)
        dx=side*(.35+.62*q)*length
        dz=(.96-.9*q)*length
        t=np.linspace(0,1,6)
        xx=x+dx*t;zz=z+dz*t
        yy=a.front(xx,zz)-.08-.24*np.sin(np.pi*t)
        curves.append(np.column_stack([xx,yy,zz]));radii.append(rng.uniform(.022,.043)*(1-.85*t))
    for upper,count in [(True,48),(False,14)]:
        angles=np.linspace(.08,np.pi-.08,count) if upper else np.linspace(np.pi+.12,2*np.pi-.12,count)
        for angle in angles:
            j=int(angle/(2*np.pi)*(len(margin)-1))
            root=margin[j].copy();root[1]-=.09
            t=np.linspace(0,1,6);length=rng.uniform(4.,6.2) if upper else rng.uniform(1.7,3.0)
            xx=root[0]+side*np.cos(angle)*1.5*t
            yy=root[1]-length*(.73*t-.2*t*t)
            zz=root[2]+(1 if upper else -1)*length*.57*t*t
            curves.append(np.column_stack([xx,yy,zz]));radii.append((.020 if upper else .011)*(1-.94*t))
    return tube_collection(('Left' if side<0 else 'Right')+'_individual_brow_hairs_and_lashes',
                           curves,radii,HAIR,a,5)


def stubble(a,rng,quality):
    curves=[];radii=[]
    # Shaved hair is modeled with short tapered shafts, sampled on the skin.
    count=22000 if quality=='final' else 8000
    x=rng.uniform(-68,68,count);z=rng.uniform(-85,-24,count)
    rx,_,_=a.section(z)
    line,up,lo=a.mouth(x)
    lip=(abs(x)<a.p.mouth_width/2+1)&(z<line+up+1)&(z>line-lo-1)
    mask=(abs(x)<rx*.93)&~lip
    density=.55+.45*np.exp(-((z+63)/17)**2)
    mask&=rng.random(count)<density
    selected_x=x[mask];selected_z=z[mask]
    selected_y=a.front(selected_x,selected_z)
    for xx,yy,zz in zip(selected_x,selected_y,selected_z):
        t=np.linspace(0,1,3)
        length=rng.uniform(.08,.30)
        curve=np.column_stack([xx+.12*t,yy-.045-length*t,zz-.18*t])
        curves.append(curve);radii.append(rng.uniform(.015,.024)*(1-.83*t))
    return tube_collection('Individual_shaved_facial_hair_shafts',curves,radii,HAIR,a,4)


def build(parameters=None,quality='final',hair=True):
    p=parameters or FaceParameters();a=Anatomy(p);rng=np.random.default_rng(p.seed)
    parts=head_mesh(a,quality)+[mouth_interior(a)]
    for side in (-1,1):
        prefix='Left' if side<0 else 'Right';c=a.eye(side)
        parts.append(nasal_cavity(a,side));parts.append(nostril_rim(a,side));parts.append(ear(a,side))
        wet,margin=eyelids(a,side)
        parts.extend(eyelid_patch(a,side))
        seal=closed_blink_seal(a,side)
        if seal is not None:parts.append(seal)
        parts.append(wet)
        parts.append(ellipsoid(prefix+'_sclera',c,[14.15,12.25,12.25],SCLERA,iris_cut=True))
        caruncle=c+np.array([-side*12.65,-6.9,-.85])
        parts.append(ellipsoid(prefix+'_lacrimal_caruncle',caruncle,[1.2,.85,.65],LID,nu=48,nv=32))
        parts.append(disk(prefix+'_iris_stroma',c,5.85,IRIS,
                          lambda r:-12.03+.025*r*r,rmin=1.78))
        parts.append(disk(prefix+'_pupil',c,1.82,PUPIL,lambda r:np.full_like(r,-11.82),nr=12))
        # Clear ocular envelope over an independently recessed iris and pupil.
        cornea=ellipsoid(prefix+'_ocular_tear_surface',c,[14.25,12.48,12.38],CORNEA)
        v=cornea.vertices.copy();dx=v[:,0]-c[0];dz=v[:,2]-c[2]
        frontal=v[:,1]<c[1]
        bulge=.72*np.exp(-((dx*dx+dz*dz)/(5.7**2))**2)
        v[frontal,1]-=bulge[frontal]
        parts.append(surface_part(cornea.name,v,cornea.faces,CORNEA,cornea.portrait_uv))
        if hair:parts.append(eyebrow_and_lashes(a,side,margin,rng))
    if hair:parts.append(stubble(a,rng,quality))
    materials=[Material('Procedural skin',(.37,.215,.145),rough=.45,ior=1.4,
                        material_source='Seeded mathematical pigment and pore fields'),
               Material('Moist eyelid margin',(.43,.22,.17),rough=.31),
               Material('Recessed oral and nasal mucosa',(.075,.022,.018),rough=.48),
               Material('Procedural sclera',(.69,.675,.61),rough=.34),
               Material('Procedural hazel iris',(.12,.095,.033),rough=.4),
               Material('Pupil interior',(.001,.001,.001),rough=.7),
               Material('Ocular clear surface',(.97,.99,1.),rough=.018,ior=1.376,opacity=.03),
               Material('Brown keratin fibers',(.026,.013,.007),rough=.52)]
    views={
        'portrait':View(az=-79,el=1.3,target=(0,-18,4),scale=145,focal_length_mm=85,f_stop=16,floor=False),
        'front':View(az=-90,el=.4,target=(0,-18,4),scale=143,focal_length_mm=85,f_stop=16,floor=False),
        'profile':View(az=-34,el=1.0,target=(0,0,-8),scale=164,focal_length_mm=85,f_stop=16,floor=False),
        'detail':View(az=-78,el=0,target=(0,-64,7),scale=80,focal_length_mm=100,f_stop=16,floor=False)}
    metadata={'units':'mm','geometry_source':'Original analytic surfaces and hand-authored landmark parameters',
              'scan_used':False,'photographic_skin_textures':False,'imported_anatomy_mesh':False,
              'image_generation':False,'parameters':asdict(p),'quality':quality,
              'renderer_reference':'CYBR GEO ORBIT v9, 40797a6fa756ce209b5e21dd83f2df1e41dbd5fb',
              'limitations':['Artistic anatomy; not measured or medically validated.',
                             'Skin scattering uses a surface BSDF approximation, not multilayer tissue transport.',
                             'Distinct anatomical surfaces intersect where hidden; this is a render assembly, not a watertight print.'],
              'input_assets':[]}
    return Assembly('procedural_human_face',parts,materials,views=views,metadata=metadata)
