"""CYBR GEO procedural human face v13: semantic cage, no scan, no face assets.

The face is generated from an original procedural skull scaffold, explicit
anthropometric parameters, a semantic control cage, analytic eye/nose/mouth
systems, and deterministic microstructure. There are no imported human meshes,
photographs, morph targets, learned face bases, or anatomy assets.

Units: millimetres. Z up. Face points toward -Y.
"""
from __future__ import annotations

from dataclasses import dataclass, asdict
import math
import numpy as np
from scipy.interpolate import PchipInterpolator, RectBivariateSpline
from mechanism_lab.core import Assembly, Material, Part, View

SKIN, LID, CAVITY, SCLERA, IRIS, PUPIL, CORNEA, HAIR = range(8)
ZMIN, ZMAX = -180.0, 132.0


def smoothstep(a,b,x):
    x=np.asarray(x,float)
    t=np.clip((x-a)/max(b-a,1e-12),0.,1.)
    return t*t*(3-2*t)


def gaussian(x,z,cx,cz,sx,sz):
    x,z=np.broadcast_arrays(np.asarray(x,float),np.asarray(z,float))
    return np.exp(-.5*((x-cx)/sx)**2-.5*((z-cz)/sz)**2)


def unit(v):
    v=np.asarray(v,float)
    return v/np.maximum(np.linalg.norm(v,axis=-1,keepdims=True),1e-12)


def grid_faces(rows,cols,reverse=False):
    a=(np.arange(rows-1)[:,None]*cols+np.arange(cols-1)[None,:]).ravel()
    f=np.concatenate([
        np.stack([a,a+1,a+cols],1),
        np.stack([a+1,a+cols+1,a+cols],1),
    ])
    return f[:,::-1] if reverse else f


def surface_part(name,vertices,faces,material=SKIN,uv=None,role='',tags=()):
    v=np.ascontiguousarray(vertices,dtype=np.float64).reshape(-1,3)
    f=np.ascontiguousarray(faces,dtype=np.int32).reshape(-1,3)
    if len(f)==0:
        raise ValueError(f"{name}: empty face set")
    tri=v[f]
    cross=np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0])
    keep=np.linalg.norm(cross,axis=1)>1e-10
    f=f[keep]
    cross=cross[keep]
    n=np.zeros_like(v)
    for k in range(3):
        np.add.at(n,f[:,k],cross)
    n=unit(n)
    p=Part(
        name,v,f,n,material=material,group='anatomy',
        provenance='original-procedural',
        role=role or name,
        tags=('original-procedural','no-scan','semantic-cage','deterministic')+tuple(tags),
    )
    p.portrait_uv=np.zeros((len(v),2),float) if uv is None else np.asarray(uv,float).reshape(-1,2)
    return p


@dataclass(frozen=True)
class FaceParameters:
    seed:int=271828
    # Anthropometric controls (mm unless noted).
    face_height:float=121.0
    bizygomatic_width:float=139.0
    bigonial_width:float=104.0
    intercanthal_width:float=34.0
    eye_width:float=27.5
    eye_opening:float=7.2
    eye_height:float=30.0
    eye_tilt:float=0.22
    nasal_width:float=38.0
    nasal_projection:float=25.0
    mouth_width:float=52.0
    upper_lip_fullness:float=1.0
    lower_lip_fullness:float=1.0
    chin_height:float=24.0
    gonial_angle:float=124.0
    forehead_width:float=129.0
    skull_width:float=1.0
    brow_weight:float=1.0
    eyelid_closure:float=.10
    asymmetry:float=.22
    skin_relief:float=.0020


class ProceduralSkull:
    """Original cranium/mandible scaffold. Not a scan or fitted human mesh."""
    def __init__(self,p:FaceParameters):
        self.p=p
        z=np.array([-180,-165,-145,-124,-103,-88,-76,-64,-50,-32,-10,15,40,65,90,108,121,129,132.],float)
        # Half-widths combine cranium, neck/shoulder base, zygoma and mandible.
        half=np.array([171,140,82,49,41,40,43,50, p.bigonial_width*.50,
                       62, p.bizygomatic_width*.50, p.bizygomatic_width*.50,
                       p.forehead_width*.50,64,65,61,51,30,0.],float)
        front=np.array([-50,-44,-29,-19,-22,-34,-52,-63,-65,-66,-66,-66,
                        -64,-61,-55,-45,-27,-5,12.],float)
        back=np.array([85,85,68,59,52,51,52,56,63,73,86,94,96,94,88,76,57,36,12.],float)
        self.half=PchipInterpolator(z,half)
        self.front=PchipInterpolator(z,front)
        self.back=PchipInterpolator(z,back)

        # Face-width profile is independent of the cranium latitude structure.
        fz=np.array([-105,-92,-80,-68,-55,-42,-25,-5,15,35,55,72,84,90.],float)
        fw=np.array([20,28,37,47,p.bigonial_width*.50,59,66,
                     p.bizygomatic_width*.50,p.bizygomatic_width*.50,
                     68,p.forehead_width*.50,59,53,49],float)
        self.face_half=PchipInterpolator(fz,fw)

    def section(self,z):
        z=np.clip(np.asarray(z,float),ZMIN,ZMAX)
        half=self.half(z)*self.p.skull_width
        return half,self.front(z),self.back(z)

    def face_half_width(self,z):
        z=np.clip(np.asarray(z,float),-105,90)
        return self.face_half(z)

    def front_shell(self,x,z):
        x,z=np.broadcast_arrays(np.asarray(x,float),np.asarray(z,float))
        rx,f,b=self.section(z)
        q=np.clip(x/np.maximum(rx,.01),-.999999,.999999)
        c=np.sqrt(np.maximum(1-q*q,0))
        mid=(f+b)*.5
        return mid-(mid-f)*c*(1+.24*q*q)

    def point(self,theta,z):
        theta,z=np.broadcast_arrays(np.asarray(theta,float),np.asarray(z,float))
        rx,f,b=self.section(z)
        s=np.sin(theta); c=np.cos(theta)
        x=rx*s
        mid=(f+b)*.5
        y=np.where(c>=0,mid-(mid-f)*c*(1+.24*s*s),mid-(b-mid)*c)
        return np.stack([x,y,z],-1)

    def uv(self,vertices):
        v=np.asarray(vertices,float)
        x,y,z=v.T
        rx,f,b=self.section(z)
        a=np.arcsin(np.clip(x/np.maximum(rx,.01),-1,1))
        a=np.where(y>(f+b)*.5,np.where(x>=0,np.pi-a,-np.pi-a),a)
        return np.column_stack([.5+a/(2*np.pi),(z-ZMIN)/(ZMAX-ZMIN)])


class EyeSystem:
    def __init__(self,p:FaceParameters,skull:ProceduralSkull):
        self.p=p; self.skull=skull

    def center(self,side):
        spacing=self.p.intercanthal_width+self.p.eye_width
        return np.array([side*spacing*.5,-47.2,self.p.eye_height+(0.12 if side<0 else -0.08)],float)

    def opening(self,x,side):
        c=self.center(side)
        half=self.p.eye_width*.5
        q=(np.asarray(x,float)-c[0])*side/half
        shape=np.maximum(1-q*q,0)
        center=c[2]+self.p.eye_tilt*q-.16*shape
        opening=self.p.eye_opening*(1-self.p.eyelid_closure)
        upper=center+.48*opening*shape**.72
        lower=center-.52*opening*shape**.80
        return q,upper,lower

    def globe_front_y(self,x,z,side):
        c=self.center(side)
        dx=np.asarray(x)-c[0]; dz=np.asarray(z)-c[2]
        rx,rz=15.2,12.0
        sph=np.maximum(1-(dx/rx)**2-(dz/rz)**2,0)
        return c[1]-12.55*np.sqrt(sph)

    def lid_offset(self,x,z,base):
        x,z,base=np.broadcast_arrays(np.asarray(x,float),np.asarray(z,float),np.asarray(base,float))
        y=base.copy()
        for side in (-1,1):
            c=self.center(side)
            lx=(x-c[0])*side; lz=z-c[2]
            ellipse=(lx/17.8)**2+(lz/10.7)**2
            blend=1-smoothstep(.60,1.05,ellipse)
            globe=self.globe_front_y(x,z,side)
            # Project periorbital skin toward the physical eyeball while leaving
            # enough thickness for the lid and orbicularis tissue.
            target=np.minimum(base,globe-.62)
            y+=(target-base)*blend*.86
            q,upper,lower=self.opening(x,side)
            shape=np.maximum(1-q*q,0)
            upper_roll=gaussian(x,z,c[0],c[2]+3.2,14.5,3.3)*shape
            lower_roll=gaussian(x,z,c[0],c[2]-3.0,14.0,3.0)*shape
            supra=gaussian(x,z,c[0],c[2]+6.4,15.6,2.0)*shape
            y-=.42*upper_roll*blend
            y-=.20*lower_roll*blend
            y+=.35*supra*blend
        return y

    def aperture_mask(self,x,z,y):
        x,z,y=np.broadcast_arrays(x,z,y)
        m=np.zeros(x.shape,bool)
        if self.p.eyelid_closure>=.995:
            return m
        for side in (-1,1):
            q,upper,lower=self.opening(x,side)
            m|=(np.abs(q)<.94)&(z<upper-.05)&(z>lower+.05)&(y<0)
        return m


class NoseSystem:
    def __init__(self,p:FaceParameters):
        self.p=p

    def offset(self,x,z):
        x,z=np.broadcast_arrays(np.asarray(x,float),np.asarray(z,float))
        d=np.zeros(x.shape,float)
        # Nasal bones / bridge / dorsum.
        bridge=smoothstep(-20,-7,z)*(1-smoothstep(30,48,z))
        bridge=np.exp(-.5*(x/4.4)**2)*bridge
        d-=8.4*bridge
        d-=7.8*gaussian(x,z,0,6,7.8,19)
        d-=self.p.nasal_projection*gaussian(x,z,0,-8.2,8.8,8.7)
        # Lower lateral cartilage and alae.
        ala=self.p.nasal_width*.31
        for s in (-1,1):
            d-=10.2*gaussian(x,z,s*ala,-13.5,6.1,6.2)
            d+=1.2*gaussian(x,z,s*(ala+5.2),-13.5,2.4,5.0)
            d-=1.0*gaussian(x,z,s*(ala-2.4),-17.0,2.7,2.2)
        # Columella and soft-triangle underside.
        d-=6.2*gaussian(x,z,0,-17.8,3.4,4.4)
        d+=1.0*gaussian(x,z,0,-22.0,6.4,2.8)
        return d

    def nostril_center(self,side):
        return side*self.p.nasal_width*.29,-16.7

    def nostril_mask(self,x,z,y):
        x,z,y=np.broadcast_arrays(x,z,y)
        m=np.zeros(x.shape,bool)
        for side in (-1,1):
            cx,cz=self.nostril_center(side)
            du=x-cx; dz=z-cz
            ur=du+side*.38*dz
            vr=dz-side*.10*du
            m|=((ur/4.8)**2+(vr/1.80)**2<1)&(y<-63.5)
        return m


class MouthSystem:
    def __init__(self,p:FaceParameters):
        self.p=p

    def bounds(self,x):
        x=np.asarray(x,float)
        a=self.p.mouth_width*.5
        q=np.clip(np.abs(x)/max(a,1e-6),0,1)
        span=np.maximum(1-q*q,0)
        line=-39.5-.55*np.exp(-(x/5.4)**2)+.10*q*q
        upper=self.p.upper_lip_fullness*(4.6+1.7*np.exp(-((np.abs(x)-6.0)/4.8)**2)
                -.7*np.exp(-(x/2.4)**2))*span**.72
        lower=self.p.lower_lip_fullness*6.5*span**.80
        return line,upper,lower

    def offset(self,x,z):
        x,z=np.broadcast_arrays(np.asarray(x,float),np.asarray(z,float))
        d=np.zeros(x.shape,float)
        line,upper,lower=self.bounds(x)
        a=self.p.mouth_width*.5
        span=np.maximum(1-(x/max(a,1e-6))**2,0)
        inside=np.abs(x)<a
        uu=np.clip((z-line)/np.maximum(upper,.001),0,1)
        ul=np.clip((line-z)/np.maximum(lower,.001),0,1)
        up=inside&(z>=line)&(z<=line+upper)
        lo=inside&(z<line)&(z>=line-lower)
        # Perioral muzzle and actual lip rolls.
        d-=1.70*gaussian(x,z,0,-38,24,15)
        d-=np.where(up,1.08*np.sin(np.pi*uu)**.92*span**.68,0)
        d-=np.where(lo,1.42*np.sin(np.pi*ul)**.94*span**.72,0)
        # Closed contact seam, philtrum and labiomental crease.
        d+=.18*np.exp(-((z-line)/.40)**2)*span**.84*inside
        d-=.55*gaussian(x,z,-3.0,-28,1.7,5.0)
        d-=.55*gaussian(x,z,3.0,-28,1.7,5.0)
        d+=1.2*gaussian(x,z,0,-57,17,3.5)
        d-=3.0*gaussian(x,z,0,-70,20,9)
        return d


class SemanticFaceCage:
    """Original semantic control cage sampled into a dense subdivision surface."""
    def __init__(self,p:FaceParameters,skull:ProceduralSkull,eyes:EyeSystem,nose:NoseSystem,mouth:MouthSystem):
        self.p=p; self.skull=skull; self.eyes=eyes; self.nose=nose; self.mouth=mouth
        self.rows=np.array([-102,-90,-78,-66,-54,-42,-30,-18,-6,8,22,36,50,62,72,82,90.],float)
        self.cols=np.array([-1,-.84,-.68,-.52,-.36,-.20,0,.20,.36,.52,.68,.84,1.],float)

        # Explicit semantic landmark corrections to the cranium shell.
        lm=[]
        def add(x,z,dy,sym=True):
            lm.append((x,z,dy))
            if sym and x: lm.append((-x,z,dy))
        add(0,95,.4,False)
        add(0,66,-.3,False)
        add(23,48,-2.8)
        add(43,42,-1.7)
        add(58,45,.8)
        add(31,31,1.2)       # orbital hollow
        add(47,17,-6.0)      # zygoma
        add(37,5,-5.8)       # malar
        add(54,-8,-2.8)
        add(48,-23,1.7)       # submalar hollow
        add(34,-22,-2.5)     # maxilla
        add(48,-50,-2.2)     # masseter
        add(50,-63,-3.3)     # mandibular body
        add(37,-75,-4.0)
        add(20,-82,-4.8)
        add(0,-78,-5.8,False) # menton/chin pad
        add(0,-93,.2,False)
        self.landmarks=np.asarray(lm,float)
        xy=self.landmarks[:,:2]
        dx=(xy[:,None,0]-xy[None,:,0])/23.
        dz=(xy[:,None,1]-xy[None,:,1])/24.
        K=np.exp(-.5*(dx*dx+dz*dz))
        self.weights=np.linalg.solve(K+np.eye(len(K))*.004,self.landmarks[:,2])

        dy=np.empty((len(self.rows),len(self.cols)),float)
        dx_ctrl=np.zeros_like(dy)
        dz_ctrl=np.zeros_like(dy)
        gonial=(124.-self.p.gonial_angle)/18.
        for i,z in enumerate(self.rows):
            w=float(self.skull.face_half_width(z))
            for j,u in enumerate(self.cols):
                x=u*w
                ax=abs(x);sgn=0. if x==0 else np.sign(x)
                dy[i,j]=self.macro_offset(x,z)
                dx_ctrl[i,j]=sgn*(
                    1.9*gaussian(ax,z,47,12,19,22)
                    -.9*gaussian(ax,z,30,31,17,14)
                    +(1.2+1.4*gonial)*gaussian(ax,z,49,-56,19,18)
                    -2.2*gaussian(ax,z,20,-80,18,13)
                )
                dz_ctrl[i,j]=(
                    -.85*gaussian(ax,z,37,25,22,13)
                    +.55*gaussian(ax,z,48,-55,22,17)
                    -.70*gaussian(ax,z,18,-80,22,12)
                )
        for arr in (dy,dx_ctrl,dz_ctrl):
            arr[:,0]=0;arr[:,-1]=0;arr[0,:]=0;arr[-1,:]=0
        self.control_offsets=dy
        self.control_dx=dx_ctrl
        self.control_dz=dz_ctrl
        self.spline=RectBivariateSpline(self.rows,self.cols,dy,kx=3,ky=3,s=0)
        self.spline_dx=RectBivariateSpline(self.rows,self.cols,dx_ctrl,kx=3,ky=3,s=0)
        self.spline_dz=RectBivariateSpline(self.rows,self.cols,dz_ctrl,kx=3,ky=3,s=0)

    def macro_offset(self,x,z):
        x,z=np.broadcast_arrays(np.asarray(x,float),np.asarray(z,float))
        dx=(x[...,None]-self.landmarks[:,0])/23.
        dz=(z[...,None]-self.landmarks[:,1])/24.
        K=np.exp(-.5*(dx*dx+dz*dz))
        return np.sum(K*self.weights,axis=-1)

    def surface_y(self,x,z):
        x,z=np.broadcast_arrays(np.asarray(x,float),np.asarray(z,float))
        w=self.skull.face_half_width(z)
        u=np.clip(x/np.maximum(w,.01),-1,1)
        macro=self.spline.ev(z,u)
        edge=(1-np.abs(u)**8)
        vertical=smoothstep(-104,-92,z)*(1-smoothstep(101,113,z))
        base=self.skull.front_shell(x,z)+macro*edge*vertical
        y=base+self.nose.offset(x,z)+self.mouth.offset(x,z)
        y=self.eyes.lid_offset(x,z,y)

        # Jaw and chin react directly to anthropometric parameters.
        jaw_delta=(self.p.bigonial_width-108.)/16.
        y-=jaw_delta*gaussian(x,z,48,-56,22,24)*.8
        chin_delta=(self.p.chin_height-24.)/8.
        y-=chin_delta*gaussian(x,z,0,-78,23,17)*.8

        # Deterministic low-frequency asymmetry only; no identity asset.
        phase=(self.p.seed%104729)/104729.*(2*np.pi)
        asym=self.p.asymmetry*(.55*np.sin(phase)*np.tanh(x/28.)*gaussian(x,z,0,-2,65,78)
                              +.35*np.cos(phase*.71)*np.tanh(x/23.)*gaussian(x,z,0,-50,58,38))
        return y+asym

    def vector_offset(self,x,z):
        """Interpolate XYZ cage displacement at arbitrary facial coordinates."""
        x,z=np.broadcast_arrays(np.asarray(x,float),np.asarray(z,float))
        w=self.skull.face_half_width(z)
        u=np.clip(x/np.maximum(w,.01),-1,1)
        return self.spline_dx.ev(z,u),self.spline_dz.ev(z,u)

    def sample_patch(self,quality='preview'):
        rows,cols=(230,241) if quality=='preview' else (400,421)
        base_z=np.linspace(self.rows[0],self.rows[-1],rows)
        uu=np.linspace(-1.015,1.015,cols)
        u,z0=np.meshgrid(uu,base_z)
        vertical_scale=self.p.face_height/121.
        z_ref=self.p.eye_height+(z0-self.p.eye_height)*vertical_scale
        w=self.skull.face_half_width(z_ref)
        dx=self.spline_dx.ev(z0,u)
        dz=self.spline_dz.ev(z0,u)*vertical_scale
        x=u*w+dx
        z=z_ref+dz
        y=self.surface_y(x,z)
        outside=np.maximum(np.abs(u)-1,0)/.015
        if np.any(outside>0):
            shell=self.skull.front_shell(x,z)
            t=np.clip(outside,0,1)
            y=y*(1-t)+shell*t
        top_t=np.clip((z-84.)/6.,0,1)
        side_fade=1-np.clip((np.abs(u)-.92)/.08,0,1)
        y-=.10*np.sin(np.pi*top_t)*side_fade
        v=np.stack([x,y,z],-1).reshape(-1,3)
        uv=self.skull.uv(v)
        f=grid_faces(rows,cols)

        center=v[f].mean(axis=1)
        cx,cy,cz=center.T
        remove=self.eyes.aperture_mask(cx,cz,cy)|self.nose.nostril_mask(cx,cz,cy)
        f=f[~remove]
        return surface_part(
            'Semantic_face_subdivision_surface',v,f,SKIN,uv,
            role='Dense surface subdivided from original semantic anthropometric control cage',
            tags=('semantic-face-surface','subdivision-style'),
        )


class HumanAnatomy:
    def __init__(self,p=FaceParameters()):
        self.p=p
        self.skull=ProceduralSkull(p)
        self.eyes=EyeSystem(p,self.skull)
        self.nose=NoseSystem(p)
        self.mouth=MouthSystem(p)
        self.cage=SemanticFaceCage(p,self.skull,self.eyes,self.nose,self.mouth)

    def surface_y(self,x,z):
        return self.cage.surface_y(x,z)

    def uv(self,v):
        return self.skull.uv(v)


def unified_head_mesh(a:HumanAnatomy,quality='preview'):
    """Single continuous skull/face mesh deformed by the semantic XYZ cage.

    The control cage remains the semantic design representation, but the final
    rendered skin has one topology and one normal field from occiput to chin.
    """
    rows,cols=(420,641) if quality=='preview' else (700,1001)
    theta=np.linspace(-np.pi,np.pi,cols)
    zz=np.linspace(ZMIN,ZMAX-.01,rows)
    tt,z0=np.meshgrid(theta,zz)
    base=a.skull.point(tt,z0)
    x0=base[...,0];y0=base[...,1]
    frontness=np.clip(np.cos(tt),0,1)
    # Cage influence vanishes smoothly before the ears/back of skull and above
    # the superior forehead, preventing any patch seam.
    side_blend=smoothstep(.08,.78,frontness)
    vertical=smoothstep(-110,-95,z0)*(1-smoothstep(84,103,z0))
    blend=side_blend*vertical

    dx,dz=a.cage.vector_offset(x0,z0)
    x=x0+blend*dx
    z=z0+blend*dz
    target_y=a.cage.surface_y(x,z)
    y=y0+blend*(target_y-y0)

    v=np.stack([x,y,z],-1).reshape(-1,3)
    uv=a.skull.uv(v)
    f=grid_faces(rows,cols)
    center=v[f].mean(axis=1)
    cx,cy,cz=center.T
    remove=a.eyes.aperture_mask(cx,cz,cy)|a.nose.nostril_mask(cx,cz,cy)
    f=f[~remove]
    return surface_part(
        'Unified_semantic_cage_human_skin',v,f,SKIN,uv,
        role='One continuous procedural skull/face skin surface deformed by XYZ semantic cage',
        tags=('unified-skin-topology','vector-cage-driven'),
    )

def ellipsoid(name,center,radii,material,nu=128,nv=80,iris_cut=False):
    theta=np.linspace(-np.pi,np.pi,nu+1)
    phi=np.linspace(.0001,np.pi-.0001,nv+1)
    tt,pp=np.meshgrid(theta,phi)
    q=np.stack([np.sin(pp)*np.sin(tt),-np.sin(pp)*np.cos(tt),np.cos(pp)],-1)
    v=(q*np.asarray(radii,float)+np.asarray(center,float)).reshape(-1,3)
    f=grid_faces(nv+1,nu+1,reverse=True)
    if iris_cut:
        mid=v[f].mean(axis=1)-center
        f=f[~((mid[:,0]**2+mid[:,2]**2<5.7**2)&(mid[:,1]<0))]
    uv=np.column_stack([.5+(v[:,0]-center[0])/(2*radii[0]),.5+(v[:,2]-center[2])/(2*radii[2])])
    return surface_part(name,v,f,material,uv)


def disk(name,center,radius,material,y_offset,rmin=0.,nr=30,nt=160):
    r=np.linspace(rmin,radius,nr)
    th=np.linspace(0,2*np.pi,nt+1)
    rr,tt=np.meshgrid(r,th,indexing='ij')
    x=rr*np.cos(tt); z=rr*np.sin(tt)
    y=np.full_like(x,y_offset)
    v=np.stack([x+center[0],y+center[1],z+center[2]],-1).reshape(-1,3)
    uv=np.column_stack([.5+x.ravel()/(2*radius),.5+z.ravel()/(2*radius)])
    return surface_part(name,v,grid_faces(nr,nt+1,reverse=True),material,uv)


def tube_collection(name,curves,radii,material,a:HumanAnatomy|None=None,sides=5):
    vs=[];fs=[];off=0
    for curve,radius in zip(curves,radii):
        c=np.asarray(curve,float)
        tangent=unit(np.gradient(c,axis=0))
        ref=np.tile([0.,1.,0.],(len(c),1))
        b=unit(np.cross(tangent,ref)); n=unit(np.cross(b,tangent))
        ang=np.linspace(0,2*np.pi,sides,endpoint=False)
        rad=np.broadcast_to(radius,(len(c),))
        v=c[:,None,:]+rad[:,None,None]*(np.cos(ang)[None,:,None]*b[:,None,:]+np.sin(ang)[None,:,None]*n[:,None,:])
        base=np.arange((len(c)-1)*sides).reshape(-1,sides)
        nxt=np.roll(base,-1,axis=1)
        f=np.concatenate([np.stack([base,nxt,base+sides],-1).reshape(-1,3),
                          np.stack([nxt,nxt+sides,base+sides],-1).reshape(-1,3)])
        vs.append(v.reshape(-1,3));fs.append(f+off);off+=v.size//3
    v=np.concatenate(vs); f=np.concatenate(fs)
    return surface_part(name,v,f,material,a.uv(v) if a else None)


def eyelid_wet_margin(a:HumanAnatomy,side):
    e=a.eyes
    c=e.center(side)
    q=np.linspace(-.94,.94,150)
    x=c[0]+side*q*(a.p.eye_width*.5)
    _,up,lo=e.opening(x,side)
    yu=e.globe_front_y(x,up,side)-.10
    yl=e.globe_front_y(x,lo,side)-.08
    upper=np.column_stack([x,yu,up])
    lower=np.column_stack([x[::-1],yl[::-1],lo[::-1]])
    return tube_collection(
        ('Left' if side<0 else 'Right')+'_wet_lid_margin',
        [upper,lower],
        [np.full(len(upper),.045),np.full(len(lower),.032)],
        LID,a,6
    ),np.concatenate([upper,lower],0)


def nostril_parts(a:HumanAnatomy,side):
    cx,cz=a.nose.nostril_center(side)
    theta=np.linspace(0,2*np.pi,161)
    rings=[]
    for scale,depth in [(1.10,-.03),(.82,.58)]:
        ur=5.0*scale*np.cos(theta); vr=1.90*scale*np.sin(theta)
        du=ur-side*.38*vr; dz=vr+side*.10*ur
        x=cx+du; z=cz+dz
        y=a.surface_y(x,z)+depth
        rings.append(np.column_stack([x,y,z]))
    v=np.stack(rings,0).reshape(-1,3)
    rim=surface_part(
        ('Left' if side<0 else 'Right')+'_alar_nostril_rim',
        v,grid_faces(2,len(theta)),SKIN,a.uv(v),
        role='Procedural alar rim and nostril sill',
    )

    r=np.linspace(0,1,24)[:,None]
    th=np.linspace(0,2*np.pi,129)
    ur=4.75*r*np.cos(th); vr=1.78*r*np.sin(th)
    du=ur-side*.38*vr; dz=vr+side*.10*ur
    x=cx+du; z=cz+dz
    y=a.surface_y(x,z)+.22+3.8*(1-r*r)
    vv=np.stack([x,y,z],-1).reshape(-1,3)
    cavity=surface_part(
        ('Left' if side<0 else 'Right')+'_nasal_vestibule',
        vv,grid_faces(24,129,reverse=True),CAVITY,a.uv(vv),
        role='Recessed procedural nasal vestibule',
    )
    return rim,cavity


def ear(a:HumanAnatomy,side):
    center=np.array([side*72.5,5.5,3.0])
    normal=np.array([side*.85,-.5268,0.]); horizontal=np.array([side*.5268,.85,0.])
    th=np.linspace(0,2*np.pi,221); r=np.linspace(0,1,84)[:,None]
    s=np.sin(th); c=np.cos(th)
    u=12.8*r*c*(1+.14*s)+1.7*(29*r*s/30)
    z=29*r*s
    h=1.8+2.7*np.exp(-((r-.85)/.073)**2)
    h=np.broadcast_to(h,u.shape).copy()
    h-=3.1*np.exp(-((u+1.3)/6.6)**2-((z+3)/11)**2)
    h+=1.7*np.exp(-((u-3)/2.8)**2-((z-4)/14)**2)
    h+=2.3*np.exp(-((u+6)/2.4)**2-((z+5)/4.5)**2)
    h+=1.5*np.exp(-((u-1)/4.)**2-((z+22)/7.)**2)
    h-=2.2*np.exp(-((u+1.8)/2.5)**2-((z+7)/3.8)**2)
    h-=2.5*smoothstep(.93,1,r)
    v=center+u[...,None]*horizontal+z[...,None]*[0,0,1]+h[...,None]*normal
    v=v.reshape(-1,3)
    f=grid_faces(len(r),len(th),reverse=side>0)
    uv=np.column_stack([.69+u.ravel()/420.,.47+z.ravel()/(ZMAX-ZMIN)])
    p=surface_part(('Left' if side<0 else 'Right')+'_procedural_pinna',v,f,SKIN,uv,role='Procedural pinna cartilage surface')
    if np.mean(p.normals@normal)<0:
        p=surface_part(p.name,v,f[:,::-1],SKIN,uv,role=p.role)
    return p


def brows_lashes(a:HumanAnatomy,side,margin,rng):
    curves=[];radii=[]
    for _ in range(430):
        q=rng.beta(1.35,1.3)
        x=side*(15.5+39*q)
        z=44+4.6*np.sin(np.pi*q*.92)-2.5*q+rng.normal(0,1.1)*(1-.5*q)
        length=rng.uniform(3.2,5.6)*(1-.25*q)
        t=np.linspace(0,1,6)
        xx=x+side*(.35+.58*q)*length*t
        zz=z+(.95-.85*q)*length*t
        yy=a.surface_y(xx,zz)-.08-.20*np.sin(np.pi*t)
        curves.append(np.column_stack([xx,yy,zz]))
        radii.append(rng.uniform(.020,.039)*(1-.85*t))
    for upper,count in [(True,52),(False,20)]:
        arr=margin[:len(margin)//2] if upper else margin[len(margin)//2:]
        idx=np.linspace(2,len(arr)-3,count,dtype=int)
        for j in idx:
            root=arr[j].copy(); t=np.linspace(0,1,6)
            length=rng.uniform(3.5,5.6) if upper else rng.uniform(1.6,2.7)
            xx=root[0]+side*.4*length*t
            yy=root[1]-length*(.70*t-.18*t*t)
            zz=root[2]+(1 if upper else -1)*length*.52*t*t
            curves.append(np.column_stack([xx,yy,zz]))
            radii.append((.020 if upper else .012)*(1-.94*t))
    return tube_collection(
        ('Left' if side<0 else 'Right')+'_procedural_brow_lash_fibers',
        curves,radii,HAIR,a,5
    )


def build(parameters=None,quality='final',hair=True):
    p=parameters or FaceParameters()
    a=HumanAnatomy(p)
    rng=np.random.default_rng(p.seed)

    parts=[unified_head_mesh(a,quality)]
    for side in (-1,1):
        prefix='Left' if side<0 else 'Right'
        parts.extend(nostril_parts(a,side))
        parts.append(ear(a,side))
        c=a.eyes.center(side)
        if p.eyelid_closure<.995:
            parts.append(ellipsoid(prefix+'_sclera_globe',c,[15.2,12.55,12.0],SCLERA))
            parts.append(disk(prefix+'_iris',c,4.65,IRIS,-12.68,rmin=1.48))
            parts.append(disk(prefix+'_pupil',c,1.50,PUPIL,-12.72,nr=12))
            cornea=ellipsoid(prefix+'_corneal_tear_surface',c,[15.3,12.72,12.1],CORNEA,nu=120,nv=76)
            vv=cornea.vertices.copy()
            dx=vv[:,0]-c[0]; dz=vv[:,2]-c[2]
            frontal=vv[:,1]<c[1]
            vv[frontal,1]-=.68*np.exp(-((dx[frontal]**2+dz[frontal]**2)/(5.3**2))**2)
            parts.append(surface_part(cornea.name,vv,cornea.faces,CORNEA,cornea.portrait_uv,role='Procedural cornea and tear film'))
            wet,margin=eyelid_wet_margin(a,side)
            parts.append(wet)
            car=c+np.array([-side*(p.eye_width*.455),-10.8,-.35])
            parts.append(ellipsoid(prefix+'_caruncle',car,[.60,.34,.42],LID,nu=40,nv=28))
            if hair: parts.append(brows_lashes(a,side,margin,rng))

    materials=[
        Material('Procedural multilobe skin',(.37,.215,.145),rough=.46,ior=1.40,material_source='Seeded chromophore and multi-scale numeric fields'),
        Material('Moist lid/caruncle',(.41,.19,.15),rough=.30),
        Material('Nasal/oral mucosa',(.105,.034,.028),rough=.50),
        Material('Procedural sclera',(.68,.65,.59),rough=.31),
        Material('Procedural iris',(.10,.082,.032),rough=.42),
        Material('Pupil',(.001,.001,.001),rough=.72),
        Material('Cornea/tear film',(.98,.99,1.),rough=.018,ior=1.376,opacity=.03),
        Material('Keratin fibers',(.024,.012,.006),rough=.53),
    ]
    views={
        'portrait':View(az=-79,el=1.5,target=(0,-18,2),scale=146,focal_length_mm=85,f_stop=16,floor=False),
        'front':View(az=-90,el=.4,target=(0,-18,2),scale=143,focal_length_mm=85,f_stop=16,floor=False),
        'profile':View(az=-35,el=1.0,target=(0,0,-5),scale=164,focal_length_mm=85,f_stop=16,floor=False),
        'detail':View(az=-80,el=.2,target=(0,-64,5),scale=79,focal_length_mm=100,f_stop=16,floor=False),
    }
    meta={
        'units':'mm',
        'geometry_source':'Single continuous procedural skull/face mesh driven by XYZ semantic anthropometric cage + analytic local anatomy systems',
        'topology':'semantic-control-cage-subdivision',
        'vector_control_cage':True,
        'unified_skin_topology':True,
        'scan_used':False,
        'imported_anatomy_mesh':False,
        'photographic_skin_textures':False,
        'learned_face_model':False,
        'image_generation':False,
        'input_assets':[],
        'parameters':asdict(p),
        'cage_control_vertices':int(len(a.cage.rows)*len(a.cage.cols)),
        'cage_rows':a.cage.rows.tolist(),
        'cage_cols':a.cage.cols.tolist(),
        'semantic_landmark_count':int(len(a.cage.landmarks)),
        'quality':quality,
        'limitations':[
            'Artistic procedural anatomy, not a medical or population-statistical model.',
            'Skin transport is a renderer-compatible multilobe surface approximation rather than volumetric histology.',
            'Hidden overlapping organic surfaces are render geometry, not print-qualified anatomy.',
        ],
    }
    return Assembly('procedural_human_face_v13',parts,materials,views=views,metadata=meta)


if __name__=='__main__':
    a=build(quality='preview',hair=False)
    print(a.name,len(a.parts),sum(len(p.faces) for p in a.parts),a.metadata)
