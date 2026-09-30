"""Procedural mineral/soil PBR atlases. No photographs, scans or image generation."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import hashlib
from functools import lru_cache
import numpy as np
from scipy.ndimage import gaussian_filter
from scipy.spatial import cKDTree
from PIL import Image
from .model import Layer


def srgb(linear):
    a=np.clip(linear,0,1)
    return np.where(a<=.0031308,12.92*a,1.055*a**(1/2.4)-.055)


def write_pfm(path,array):
    a=np.asarray(array,dtype='<f4')
    if a.ndim!=3 or a.shape[2]!=3 or not np.isfinite(a).all():raise ValueError("Invalid texture")
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    with path.open('wb') as stream:
        stream.write(f'PF\n{a.shape[1]} {a.shape[0]}\n-1.0\n'.encode());stream.write(a[::-1].tobytes())
    return path


@dataclass
class Atlas:
    layer: Layer
    wetness: float
    albedo: np.ndarray
    normal: np.ndarray
    roughness: np.ndarray
    height: np.ndarray
    color_pfm: Path | None = None
    normal_pfm: Path | None = None
    roughness_pfm: Path | None = None

    def save(self,directory,name):
        directory=Path(directory);directory.mkdir(parents=True,exist_ok=True)
        self.color_pfm=write_pfm(directory/(name+'_color.pfm'),self.albedo)
        self.normal_pfm=write_pfm(directory/(name+'_normal.pfm'),self.normal*.5+.5)
        self.roughness_pfm=write_pfm(directory/(name+'_roughness.pfm'),np.repeat(self.roughness[:,:,None],3,axis=2))
        Image.fromarray(np.round(srgb(self.albedo)*255).astype('uint8')).save(directory/(name+'_albedo.png'))
        Image.fromarray(np.round(np.clip(self.normal*.5+.5,0,1)*255).astype('uint8')).save(directory/(name+'_normal.png'))
        orm=np.stack((np.ones_like(self.roughness),self.roughness,np.zeros_like(self.roughness)),axis=-1)
        Image.fromarray(np.round(orm*255).astype('uint8')).save(directory/(name+'_orm.png'))
        return self


def _unit_noise(rng,size,sigma):
    noise=gaussian_filter(rng.normal(size=(size,size)),sigma,mode='wrap')
    return noise/max(noise.std(),1e-9)


@lru_cache(maxsize=32)
def _dry_surface(name:str,color:tuple,roughness:float,size:int):
    """Periodic aggregate, pore and mineral fields at actual metre scales.

    This is authored subgrid appearance, not a grain-resolved erosion solver.
    Reusing the dry fields keeps wet and dry patches geometrically consistent.
    """
    seed=int.from_bytes(hashlib.sha256(name.encode()).digest()[:8],'little')
    rng=np.random.default_rng(seed)
    macro=_unit_noise(rng,size,max(1,size/19))
    meso=_unit_noise(rng,size,max(.6,size/80))
    micro=_unit_noise(rng,size,.55)
    organic='Organic' in name
    soil=organic or 'soil' in name or 'horizon' in name or 'Clay' in name
    sand='Sand' in name or 'sand' in name
    shale='shale' in name.lower()
    gravel='Gravel' in name or 'Basalt' in name
    # Nearest periodic nuclei produce distinct clods/mineral grains, rather
    # than the former cloudy noise. cKDTree's boxsize makes both axes tile.
    count=1024 if soil else 2400 if sand else 85 if gravel else 170
    nuclei=rng.uniform(0,size,(count,2))
    yy,xx=np.mgrid[:size,:size]
    # Smooth periodic domain warping removes the rigid straight-sided cell
    # network. It is only a subgrid aggregate pattern, not painted polygons.
    warp_x=_unit_noise(rng,size,max(1,size/30))*size/170
    warp_y=_unit_noise(rng,size,max(1,size/30))*size/170
    query=np.c_[((xx+warp_x)%size).ravel(),((yy+warp_y)%size).ravel()]
    distances,nearest=cKDTree(nuclei,boxsize=size).query(query,k=2)
    d0,d1=distances[:,0].reshape(size,size),distances[:,1].reshape(size,size)
    cell=nearest[:,0].reshape(size,size)
    # Fissures separate aggregates; their edges are rounded over a small
    # number of pixels instead of becoming infinite-slope normal-map spikes.
    edge=np.exp(-((d1-d0)/max(.7,size/650))**2)
    pores=np.maximum(0,-micro-1.1)**1.3
    speck=np.maximum(0,micro-1.4)
    tone=gaussian_filter(rng.uniform(-1,1,count)[cell],max(1.2,size/170),mode='wrap')
    bulge=np.exp(-(d0/max(1,np.sqrt(size*size/count)*.55))**2)
    u,v=xx/size,yy/size
    if soil:
        # Centimetre aggregate silhouettes already exist as actual meshes.
        # Repeating millimetre-high Voronoi ridges on those surfaces produced
        # an artificial polygon crust. Soil bump now describes only granular
        # sub-millimetre relief; weak warped seams remain subordinate to pores.
        relief=.00035*meso+.00015*micro-.00020*pores-.00010*edge
        relief*=.8 if organic else 1
        variation=1+.020*tone+.075*macro+.060*meso+.060*micro-.015*edge-.055*pores
        fleck=np.clip(speck*.10,0,.28)
    elif shale:
        bedding=np.sin(2*np.pi*(44*v+.13*np.sin(2*np.pi*u)+.08*np.sin(6*np.pi*u)))
        relief=.0007*bedding+.0011*bulge-.0017*edge+.00023*micro
        variation=1+.13*tone+.06*macro+.065*bedding-.09*edge
        fleck=np.clip(speck*.07,0,.28)
    elif sand:
        relief=.0011*bulge+.00038*meso+.00018*micro-.00040*pores
        variation=1+.055*tone+.05*macro+.035*meso+.025*micro-.035*pores
        fleck=np.clip(speck*.12,0,.32)
    else:
        relief=.0022*bulge-.0018*edge+.00065*meso+.00024*micro
        variation=1+.17*tone+.08*macro+.045*meso-.11*edge
        fleck=np.clip(speck*.13,0,.36)
    variation=np.clip(variation,.38,1.52)
    # Mineral inclusions have different colours as well as different heights.
    base=np.asarray(color)[None,None,:]*variation[:,:,None]
    pigment=np.stack((1+.065*macro,1+.01*macro,1-.07*macro),axis=-1)
    albedo=base*pigment
    mineral=np.array((.24,.20,.15)) if soil else np.array((.48,.43,.34))
    albedo=albedo*(1-fleck[:,:,None])+mineral*fleck[:,:,None]
    if organic:
        carbon=gaussian_filter(np.clip(np.maximum(0,-meso-.9)*.08,0,.15),.8,mode='wrap')
        albedo*=1-carbon[:,:,None]
    rough=np.clip(roughness+.065*meso+.035*tone+.06*pores-.04*speck,.48,.985)
    return (np.clip(albedo,.001,.88).astype('float32'),rough.astype('float32'),relief.astype('float32'))


def make_atlas(layer: Layer,wetness=0.0,size=512):
    if not 0<=wetness<=1 or size<16 or size>2048:raise ValueError("Invalid material atlas settings")
    albedo,dry_rough,relief=_dry_surface(layer.name,tuple(layer.color),layer.roughness,size)
    # Wet pore filling flattens the smallest relief but leaves clod silhouettes
    # to the actual geometry. These optical responses are authored, uncalibrated.
    relief=relief*(1-.38*wetness)
    dx=1.25/size
    du=(np.roll(relief,-1,1)-np.roll(relief,1,1))/(2*dx)
    dv=(np.roll(relief,-1,0)-np.roll(relief,1,0))/(2*dx)
    normal=np.stack((-du,-dv,np.ones_like(du)),axis=-1)
    normal/=np.linalg.norm(normal,axis=-1,keepdims=True)
    albedo=albedo*(1-.48*wetness)
    rough=np.clip(dry_rough-.40*wetness,.24,.985)
    return Atlas(layer,wetness,albedo.astype('float32'),normal.astype('float32'),rough.astype('float32'),relief.astype('float32'))
