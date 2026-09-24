"""Deterministic skin/eye material maps synthesized from numeric fields only."""
from __future__ import annotations
import hashlib
import json
from pathlib import Path
import numpy as np
from PIL import Image
from scipy.ndimage import gaussian_filter, gaussian_filter1d, map_coordinates


def srgb(x):
    x=np.clip(x,0,1)
    return np.where(x<=.0031308,x*12.92,1.055*x**(1/2.4)-.055)


def save_rgb(path,linear):
    Image.fromarray(np.uint8(np.clip(srgb(linear)*255+.5,0,255))).save(path)


def soft_noise(rng,size,small):
    a=rng.standard_normal((small,small)).astype(np.float32)
    im=Image.fromarray(a).resize((size,size),Image.Resampling.BICUBIC)
    a=np.array(im,dtype=np.float32)
    return a/np.maximum(a.std(),1e-6)


def skin_maps(out,anatomy,size=4096):
    from procedural_human_face import ZMIN,ZMAX,gaussian,smoothstep
    rng=np.random.default_rng(anatomy.p.seed+10)
    theta=np.linspace(-np.pi,np.pi,size,dtype=np.float32)[None,:]
    z=np.linspace(ZMAX,ZMIN,size,dtype=np.float32)[:,None]
    rx,_,_=anatomy.section(z);rx=rx.astype(np.float32)
    x=rx*np.sin(theta);z=np.broadcast_to(z,x.shape)
    front=smoothstep(0,.6,np.cos(theta)).astype(np.float32)
    macro=soft_noise(rng,size,28)
    meso=soft_noise(rng,size,180)
    fine=soft_noise(rng,size,720)
    pigment=.012*macro+.0055*meso+.0018*fine
    # Chromophore-inspired artistic color mixing, not a measured spectral model.
    color=np.empty((size,size,3),np.float32)
    for c,value in enumerate([.355,.203,.131]):
        color[...,c]=value+pigment*[1.,.64,.43][c]
    red=(.09*gaussian(x,z,0,-7,16,19)
         +.10*gaussian(abs(x),z,43,1,19,15)
         +.05*gaussian(abs(x),z,31,28,20,12))*front
    color[...,0]+=red*.20;color[...,1]-=red*.105;color[...,2]-=red*.05
    # Tiny melanin clusters have varying radii and density, without copied pores.
    freckles=np.maximum(soft_noise(rng,size,500)-2.1,0)
    freckles=gaussian_filter(freckles,.45,mode='wrap')
    freckle_zone=.5+.5*gaussian(abs(x),z,38,5,24,35)*front
    color*=1-.075*freckles[...,None]*freckle_zone[...,None]
    line,upper,lower=anatomy.mouth(x)
    rel=np.where(z>=line,(z-line)/np.maximum(upper,.001),(line-z)/np.maximum(lower,.001))
    lips=(1-smoothstep(.68,1.32,rel))*smoothstep(anatomy.p.mouth_width/2+1.4,
                anatomy.p.mouth_width/2-2.3,abs(x))*front
    lip_color=np.stack([.235+.010*meso,.145+.006*meso,.120+.005*meso],-1)
    color=color*(1-lips[...,None])+lip_color*lips[...,None]
    # Sparse neutral follicle pigmentation complements actual modeled stubble.
    beard=np.exp(-((z+59)/30)**4)*front*(1-lips)
    color[...,0]-=.009*beard;color[...,1]-=.004*beard
    rough=.46+.016*meso+.009*fine
    oil=(gaussian(x,z,0,-8,15,23)+.5*gaussian(x,z,0,66,44,20))*front
    rough-=.060*oil;rough=rough*(1-lips)+(.425+.016*meso)*lips
    # Skin furrows and isolated pore dimples in a calibrated 0.06 mm range.
    impulses=(rng.random((size,size),dtype=np.float32)<.012).astype(np.float32)
    impulses*=rng.uniform(.5,1.5,(size,size)).astype(np.float32)
    inner=gaussian_filter(impulses,.72,mode='wrap')*3.4
    rim=gaussian_filter(impulses,1.45,mode='wrap')*2.4
    pores=-.020*inner+.005*rim
    region=.6+.5*gaussian(abs(x),z,38,1,22,25)*front+.25*oil
    height=pores*region+.0017*fine+.0009*meso
    # Vermilion microfolds run across the lips' vertical extent, interrupting
    # at independent phases rather than a single periodic corrugation.
    lipfold=(np.sin(x*3.2+.6*meso)+.35*np.sin(x*6.4+.3*fine))
    height=height*(1-lips)+(.0032*lipfold+.0012*fine)*lips
    for h,amp in [(67,.007),(78,.005),(89,.004)]:
        curve=h+.0016*x*x+.28*np.sin(.08*x)
        height-=amp*np.exp(-((z-curve)/.24)**2)*np.exp(-(x/47)**6)*front
    height=np.clip(.5+height/.06,0,1)
    # The meridian is one continuous surface; match both texture edges exactly.
    for arr in (color,rough,height):
        seam=(arr[:,0]+arr[:,-1])*.5
        arr[:,0]=seam;arr[:,-1]=seam
    save_rgb(out/'skin_color.png',color)
    Image.fromarray(np.uint8(np.clip(rough,0,1)*255+.5)).save(out/'skin_roughness.png')
    Image.fromarray(np.uint16(height*65535+.5)).save(out/'skin_height.png')


def eye_maps(out,seed,size=1024):
    rng=np.random.default_rng(seed+20)
    q=np.linspace(-1,1,size,dtype=np.float32)
    x,z=np.meshgrid(q,-q);r=np.sqrt(x*x+z*z);theta=np.arctan2(z,x)
    angular=rng.standard_normal(2048).astype(np.float32)
    fibers=np.zeros_like(r)
    for sigma,weight in [(1.3,.3),(4.,.4),(11.,.3)]:
        f=gaussian_filter1d(angular,sigma,mode='wrap');f/=max(f.std(),1e-6)
        coord=(theta/(2*np.pi)+.5)*2048+6*np.sin(r*19+theta*7)
        fibers+=weight*map_coordinates(f,[coord],order=1,mode='wrap')
    radnoise=.4*np.sin(r*43+theta*3)+.3*np.sin(r*87-theta*8)
    amber=np.exp(-((r-.42)/.23)**2)
    iris=np.empty((size,size,3),np.float32)
    iris[...,0]=.075+.115*amber+.018*fibers+.004*radnoise
    iris[...,1]=.091+.027*amber+.018*fibers+.005*radnoise
    iris[...,2]=.033+.005*amber+.009*fibers+.003*radnoise
    crypts=np.maximum(-fibers-.65,0)*np.exp(-((r-.52)/.19)**2)
    iris*=1-.33*np.clip(crypts,0,1)[...,None]
    iris*= (1-.77*np.exp(-((r-.978)/.06)**2))[...,None]
    iris[r>1]=[.009,.014,.006]
    save_rgb(out/'iris_color.png',np.clip(iris,.002,1))
    sclera=np.empty_like(iris);sclera[:]=[.68,.645,.59]
    n=soft_noise(rng,size,30)
    sclera+=n[...,None]*np.array([.009,.008,.006])
    # Curved, branching small vessels concentrated toward the canthi.
    vein=np.zeros_like(r)
    for side in (-1,1):
        for k in range(14):
            start=rng.uniform(-.9,.9);freq=rng.uniform(3,8);phase=rng.uniform(0,6)
            path=start+.08*np.sin(x*freq+phase)+.055*np.sin(x*18+phase)
            d=np.abs(z-path)
            strength=np.clip((side*x-.29)/.55,0,1)**2
            vein+=np.exp(-(d/rng.uniform(.0018,.004))**2)*strength*rng.uniform(.10,.25)
    sclera[...,0]+=vein*.11;sclera[...,1]-=vein*.22;sclera[...,2]-=vein*.19
    save_rgb(out/'sclera_color.png',sclera)


def generate(out,anatomy,size=4096):
    out=Path(out);out.mkdir(parents=True,exist_ok=True)
    skin_maps(out,anatomy,size)
    eye_maps(out,anatomy.p.seed)
    files=['skin_color.png','skin_roughness.png','skin_height.png','iris_color.png','sclera_color.png']
    report={'source':'Deterministic numeric fields only; no input images or meshes',
            'seed':anatomy.p.seed,'skin_resolution':[size,size],'eye_resolution':[1024,1024],
            'encoded_height_normalization_mm':.06,
            'files':{f:hashlib.sha256((out/f).read_bytes()).hexdigest() for f in files}}
    (out/'texture_provenance.json').write_text(json.dumps(report,indent=2)+'\n')
    return report
