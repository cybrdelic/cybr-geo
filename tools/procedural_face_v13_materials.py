"""Deterministic v13 human skin/eye maps from numeric fields only.

No photographs, scans, learned textures, or external face assets are used.
"""
from __future__ import annotations
import hashlib, json
from pathlib import Path
import numpy as np
from PIL import Image
from scipy.ndimage import gaussian_filter, gaussian_filter1d, map_coordinates


def srgb(x):
    x=np.clip(x,0,1)
    return np.where(x<=.0031308,x*12.92,1.055*x**(1/2.4)-.055)


def save_rgb(path,linear):
    Image.fromarray(np.uint8(np.clip(srgb(linear)*255+.5,0,255))).save(path)


def save_gray(path,x,bits=8):
    x=np.clip(x,0,1)
    if bits==16:
        Image.fromarray(np.uint16(x*65535+.5)).save(path)
    else:
        Image.fromarray(np.uint8(x*255+.5)).save(path)


def soft_noise(rng,size,small):
    a=rng.standard_normal((small,small)).astype(np.float32)
    a=np.asarray(Image.fromarray(a).resize((size,size),Image.Resampling.BICUBIC),np.float32)
    return a/np.maximum(a.std(),1e-6)


def atlas_coordinates(anatomy,size):
    from procedural_human_face_v13 import ZMIN,ZMAX
    theta=np.linspace(-np.pi,np.pi,size,dtype=np.float32)[None,:]
    z=np.linspace(ZMAX,ZMIN,size,dtype=np.float32)[:,None]
    rx,_,_=anatomy.skull.section(z)
    x=rx.astype(np.float32)*np.sin(theta)
    zz=np.broadcast_to(z,x.shape)
    front=np.clip(np.cos(theta),0,1).astype(np.float32)
    return theta,x,zz,front


def skin_maps(out,anatomy,size=2048):
    from procedural_human_face_v13 import gaussian,ZMIN,ZMAX
    rng=np.random.default_rng(anatomy.p.seed+101)
    theta,x,z,front=atlas_coordinates(anatomy,size)

    # Multi-scale deterministic fields: broad pigmentation, mesoscopic mottling,
    # fine epidermal variance, and sparse pore impulses.
    macro=soft_noise(rng,size,24)
    meso=soft_noise(rng,size,150)
    fine=soft_noise(rng,size,620)

    # Chromophore-inspired parameter fields. They are artistic numerical
    # approximations, not measured tissue spectra.
    melanin=np.clip(.46+.045*macro+.018*meso,0.22,.72)
    hemoglobin=np.clip(.38+.055*meso+.020*fine,0.15,.68)

    cheek=(gaussian(np.abs(x),z,40,2,22,25)+.5*gaussian(np.abs(x),z,32,28,23,16))*front
    nose=gaussian(x,z,0,-4,16,25)*front
    ear_zone=np.clip((np.abs(x)-58)/18,0,1)*front
    vascular=np.clip(.55*cheek+.35*nose+.18*ear_zone,0,1)
    hemoglobin=np.clip(hemoglobin+.14*vascular,0,1)

    # Simple chromophore mixing in linear RGB.
    base=np.empty((size,size,3),np.float32)
    base[...,0]=.50-.27*melanin+.115*hemoglobin
    base[...,1]=.34-.23*melanin+.030*hemoglobin
    base[...,2]=.245-.18*melanin-.025*hemoglobin
    base+=fine[...,None]*np.array([.0028,.0022,.0017],np.float32)

    # Under-eye and beard-region variation.
    for s in (-1,1):
        c=anatomy.eyes.center(s)
        under=gaussian(x,z,c[0],c[2]-7,18,7)*front
        base[...,0]-=.010*under;base[...,1]-=.013*under;base[...,2]-=.006*under
    beard=np.exp(-((z+62)/29)**4)*front
    base[...,0]-=.007*beard;base[...,1]-=.005*beard;base[...,2]-=.002*beard

    # Sparse melanin clusters / freckles; no copied image content.
    clusters=np.maximum(soft_noise(rng,size,430)-2.25,0)
    clusters=gaussian_filter(clusters,.50,mode='wrap')
    freckle_zone=(.35+.65*cheek)*front
    base*=1-.055*clusters[...,None]*freckle_zone[...,None]

    # Lips follow the actual procedural mouth bounds.
    line,upper,lower=anatomy.mouth.bounds(x)
    rel=np.where(z>=line,(z-line)/np.maximum(upper,.001),(line-z)/np.maximum(lower,.001))
    lips=(rel<1).astype(np.float32)*np.clip(1-rel,0,1)**.35
    lips*=np.clip(1-(np.abs(x)/(anatomy.p.mouth_width*.53))**8,0,1)*front
    lip=np.stack([
        .270+.008*meso,
        .155+.005*meso,
        .132+.004*fine,
    ],-1)
    base=base*(1-lips[...,None])+lip*lips[...,None]

    # Regional roughness. T-zone is smoother/oilier, cheeks a little rougher,
    # lips are moist but not cosmetic-glossy.
    rough=.47+.013*meso+.007*fine
    oil=(gaussian(x,z,0,-5,15,27)+.55*gaussian(x,z,0,62,42,20))*front
    rough-=.055*oil
    rough+=.018*cheek
    rough=rough*(1-lips)+(.43+.012*meso)*lips

    # Multi-scale height. Macro folds are geometric in v13; this map contains
    # pores, furrows, lip microfolds and subtle wrinkles only.
    impulses=(rng.random((size,size),dtype=np.float32)<.010).astype(np.float32)
    impulses*=rng.uniform(.5,1.4,(size,size)).astype(np.float32)
    inner=gaussian_filter(impulses,.75,mode='wrap')*3.1
    rim=gaussian_filter(impulses,1.55,mode='wrap')*2.15
    pore=-.016*inner+.004*rim
    height=pore*(.62+.28*cheek+.18*oil)+.0012*fine+.00075*meso

    lipfold=np.sin(x*2.7+.5*meso)+.28*np.sin(x*5.9+.3*fine)
    height=height*(1-lips)+(.00145*lipfold+.00065*fine)*lips
    for h,amp in [(69,.0040),(81,.0032),(92,.0025)]:
        curve=h+.0015*x*x+.22*np.sin(.075*x)
        height-=amp*np.exp(-((z-curve)/.32)**2)*np.exp(-(x/48)**6)*front

    height_encoded=np.clip(.5+height/.050,0,1)

    # Renderer-compatible scattering/flatness guide. Thin/highly vascular areas
    # get more diffuse transport; oily T-zone gets less.
    scatter=np.clip(.16+.15*vascular+.06*(1-melanin)-.05*oil, .08,.38)
    thickness=np.clip(.52+.18*cheek+.10*gaussian(x,z,0,-68,28,20)-.08*nose,.25,.85)

    # Meridian seam continuity.
    for arr in (base,rough,height_encoded,scatter,thickness):
        seam=(arr[:,0]+arr[:,-1])*.5
        arr[:,0]=seam;arr[:,-1]=seam

    save_rgb(out/'skin_color.png',base)
    save_gray(out/'skin_roughness.png',rough)
    save_gray(out/'skin_height.png',height_encoded,16)
    save_gray(out/'skin_scatter.png',scatter)
    save_gray(out/'skin_thickness.png',thickness)
    save_gray(out/'skin_melanin.png',melanin)
    save_gray(out/'skin_hemoglobin.png',hemoglobin)


def eye_maps(out,seed,size=1024):
    rng=np.random.default_rng(seed+202)
    q=np.linspace(-1,1,size,dtype=np.float32)
    x,z=np.meshgrid(q,-q)
    r=np.sqrt(x*x+z*z);th=np.arctan2(z,x)

    angular=rng.standard_normal(4096).astype(np.float32)
    fibers=np.zeros_like(r)
    for sigma,w in [(1.2,.32),(4.0,.40),(13.0,.28)]:
        f=gaussian_filter1d(angular,sigma,mode='wrap')
        f/=max(f.std(),1e-6)
        coord=(th/(2*np.pi)+.5)*4096+7*np.sin(r*21+th*6)
        fibers+=w*map_coordinates(f,[coord],order=1,mode='wrap')
    crypt=np.maximum(-fibers-.62,0)*np.exp(-((r-.55)/.20)**2)
    amber=np.exp(-((r-.42)/.25)**2)
    iris=np.empty((size,size,3),np.float32)
    iris[...,0]=.067+.095*amber+.017*fibers
    iris[...,1]=.073+.031*amber+.014*fibers
    iris[...,2]=.029+.007*amber+.008*fibers
    iris*=1-.31*np.clip(crypt,0,1)[...,None]
    limbus=np.exp(-((r-.97)/.055)**2)
    iris*=1-.70*limbus[...,None]
    iris[r>1]=[.008,.011,.006]
    save_rgb(out/'iris_color.png',np.clip(iris,.002,1))

    scl=np.empty_like(iris)
    scl[:]=[.70,.675,.625]
    n=soft_noise(rng,size,32)
    scl+=n[...,None]*np.array([.006,.005,.004])
    veins=np.zeros_like(r)
    for side in (-1,1):
        for _ in range(12):
            start=rng.uniform(-.85,.85);freq=rng.uniform(3,7);phase=rng.uniform(0,6.2)
            path=start+.06*np.sin(x*freq+phase)+.04*np.sin(x*16+phase)
            dist=np.abs(z-path)
            strength=np.clip((side*x-.28)/.58,0,1)**2
            veins+=np.exp(-(dist/rng.uniform(.002,.0045))**2)*strength*rng.uniform(.08,.20)
    scl[...,0]+=veins*.075;scl[...,1]-=veins*.11;scl[...,2]-=veins*.10
    save_rgb(out/'sclera_color.png',np.clip(scl,0,1))


def generate(out,anatomy,size=2048):
    out=Path(out);out.mkdir(parents=True,exist_ok=True)
    skin_maps(out,anatomy,size)
    eye_maps(out,anatomy.p.seed)
    files=[
        'skin_color.png','skin_roughness.png','skin_height.png','skin_scatter.png',
        'skin_thickness.png','skin_melanin.png','skin_hemoglobin.png',
        'iris_color.png','sclera_color.png'
    ]
    report={
        'source':'Deterministic numeric fields only; no photographs, scans, learned textures, or anatomy assets',
        'seed':anatomy.p.seed,
        'skin_resolution':[size,size],
        'eye_resolution':[1024,1024],
        'files':{f:hashlib.sha256((out/f).read_bytes()).hexdigest() for f in files},
    }
    (out/'texture_provenance.json').write_text(json.dumps(report,indent=2)+'\n')
    return report
