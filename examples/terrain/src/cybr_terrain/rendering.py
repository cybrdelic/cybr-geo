"""Trace real GEO parts using the pinned CYBR LIGHT meshlet/spectral pipeline."""
from __future__ import annotations

from pathlib import Path
import json
import math
import hashlib
import numpy as np
from cybr_light import Scene, Meshlets, render
from .geometry import Geometry
from .materials import write_pfm
from scipy.ndimage import map_coordinates

QUALITY={"smoke":(160,120,8,4),"preview":(800,600,96,8),"production":(1600,1200,256,12)}


def elevation(state,x,y):
    """Sample the simulated surface in world metres, rather than block bounds."""
    n=state.config.grid;extent=state.config.extent
    return float(map_coordinates(state.height,[[((y/extent)+.5)*(n-1)],
                                              [((x/extent)+.5)*(n-1)]],order=1,mode='nearest')[0])


def outdoor_camera(state,view):
    extent=state.config.extent
    if state.config.preset=='soil-profile':
        # Frame the interior soil bank at human close-inspection distance.
        # The far bank, rather than a square patch skyline, fills the backdrop.
        origin=np.array([-.035*extent,-.085*extent,0.])
        target=np.array([.025*extent,.075*extent,0.])
        origin[2]=elevation(state,*origin[:2])+(.30 if view=='macro' else .55)
        target[2]=elevation(state,*target[:2])-.25
        return origin,target,46 if view=='macro' else 54
    if view=='macro':
        origin=np.array([-.035*extent,-.16*extent,0.]);target=np.array([.045*extent,-.045*extent,0.])
        origin[2]=elevation(state,*origin[:2])+.38;target[2]=elevation(state,*target[:2])+.06
        return origin,target,52
    x=.10*extent if state.config.preset=='badlands' else .025*extent
    target=np.array([x,.025*extent,0.]);origin=np.array([-.04*extent,-.28*extent,0.])
    origin[2]=elevation(state,*origin[:2])+(.58 if extent<15 else 1.55)
    target[2]=elevation(state,*target[:2])+.10
    return origin,target,56


def daylight(s,directory):
    """An authored, linear-radiance outdoor sky and explicitly sampled sun.

    No photographed lighting, backdrop, or neural image generation is used.
    The lat-long convention follows CYBR Light's Y-polar environment lookup;
    sky height is measured along the terrain's Z-up coordinate system.
    """
    width,height=512,256
    phi=2*np.pi*(np.arange(width)+.5)/width
    theta=np.pi*(np.arange(height)+.5)/height
    direction_z=np.sin(theta[:,None])*np.sin(phi[None,:])
    altitude=np.clip(direction_z,0,1)
    horizon=np.array([.52,.62,.72]);zenith=np.array([.15,.30,.54])
    sky=horizon+(zenith-horizon)*altitude[...,None]**.45
    sky[direction_z<0]=np.array([.085,.073,.059])
    path=Path(directory)/'daylight.pfm'
    if not path.exists():write_pfm(path,sky)
    s.emit('environment',(1,1,1),.75);s.emit('envmap',json.dumps(str(path.resolve())),0)
    sun=np.array([-.55,-.40,.72]);sun/=np.linalg.norm(sun)
    # Direct irradiance, rather than giant studio softboxes, gives relief a
    # readable physical scale. Sky illumination fills the occluded cavities.
    s.emit('delta_light',1,(0,0,0),-sun,(1,.91,.77),3.2,.2,.1)
    return {'lighting':'procedural Z-up daylight sky and directional sun',
            'sun_direction':sun.tolist(),'sky_strength':.75,'sun_irradiance_scale':3.2}


def pack_geometry(geometry:Geometry,path,material_map):
    geometry.validate()
    with Meshlets(path) as writer:
        for component,p in enumerate(geometry.assembly.parts):
            a=geometry.attributes[p.name];matrix=geometry.assembly.pose(p)
            v=(p.vertices@matrix[:3,:3].T+matrix[:3,3])*.001
            normal=p.normals@matrix[:3,:3].T
            for start in range(0,len(p.faces),24000):
                faces=p.faces[start:start+24000];records=np.zeros((len(faces),36),'<f4')
                records[:,:9]=v[faces].reshape(-1,9);records[:,9:18]=normal[faces].reshape(-1,9)
                records[:,18]=material_map[p.material]
                # One closed dielectric region shares an object identifier.
                records[:,19]=900000 if p.group=='water' else component+1
                records[:,20:26]=a.uv[faces].reshape(-1,6);records[:,26:35]=a.tint[faces].reshape(-1,9)
                writer.add(records)
    return {"triangles":writer.triangles,"meshlets":writer.meshlets,"indexed_vertices":writer.vertices,
            "bytes":Path(path).stat().st_size,"parts":len(geometry.assembly.parts)}


def render_terrain(geometry:Geometry,output,quality='preview',threads=4,width=None,spp=None,view='hero'):
    if quality not in QUALITY or view not in ('hero','macro','overhead','profile'):raise ValueError("Invalid view or quality")
    w,h,packets,bands=QUALITY[quality]
    if width is not None:
        if not isinstance(width,int) or width<16 or width>4096:raise ValueError("Width must be 16–4096")
        h=round(width*h/w);w=width
    if spp is not None:packets=spp
    if packets<1 or threads<1:raise ValueError("Sampling and threads must be positive")
    output=Path(output)
    if output.suffix!='.png':raise ValueError("Output must end in .png")
    s=Scene();s.settings(w,h,packets,12,threads,bands,geometry.state.config.seed,1.15)
    material_map={}
    for index,m in enumerate(geometry.assembly.materials):
        if index==geometry.water_material:
            wet=geometry.state.water>.003
            concentration=geometry.state.sediment.sum(axis=0)/np.maximum(geometry.state.water,1e-8)
            suspended=float(np.median(concentration[wet])) if wet.any() else 0.
            # An illustrative concentration response, not a measured optical
            # sediment model. Increased blue absorption gives turbid brown water.
            absorption=np.array([.25,.095,.045])+min(suspended,.10)*np.array([35.,90.,170.])
            material_map[index]=s.material('roughglass',(1,1,1),.045,ior=1.333,dispersion=.002,absorption=absorption)
        else:
            atlas=geometry.atlases[index]
            base=s.material('plastic',(1,1,1),float(np.median(atlas.roughness)),ior=1.50,texture=atlas.color_pfm)
            if getattr(atlas,'roughness_pfm',None):
                s.emit('roughness_texture',base,json.dumps(str(atlas.roughness_pfm.resolve())),1,1,1)
            wrapper=s.material('normalmap',texture=atlas.normal_pfm);s.emit('nested',wrapper,base,-1,.5)
            material_map[index]=wrapper
    bounds=geometry.assembly.bounds*.001;extent=geometry.state.config.extent
    target=np.array([0,0,(bounds[0,2]+bounds[1,2])*.48])
    if view=='overhead':
        origin=target+np.array([extent*.001,-extent*.001,extent*1.72]);up=(0,1,0);fov=42
    elif view=='profile':
        origin=target+np.array([extent*.25,-extent*1.62,extent*.40]);up=(0,0,1);fov=39
    else:
        origin,target,fov=outdoor_camera(geometry.state,view);up=(0,0,1)
    aperture=.002 if view=='macro' else 0.
    s.camera(origin,target,up,fov,aperture=aperture)
    lighting=daylight(s,getattr(geometry,'asset_directory',output.parent)/'textures')
    mesh=output.with_suffix('.clm');mesh.parent.mkdir(parents=True,exist_ok=True)
    stats=pack_geometry(geometry,mesh,material_map);s.mesh(mesh)
    report=render(s,output.with_suffix(''),metadata={
        'generator':'CYBR TERRAIN 0.2','preset':geometry.state.config.preset,'view':view,'geometry':stats,
        'lighting':lighting,'camera_origin_metres':origin.tolist(),'camera_target_metres':target.tolist(),
        'camera_aperture_radius_metres':aperture,
        'simulation_time_seconds':geometry.state.time,'morphological_factor':geometry.state.config.morphological_factor,
        'source_mesh_sha256':hashlib.sha256(mesh.read_bytes()).hexdigest(),
        'terrain_geometry':geometry.assembly.metadata,
        'material_mapping':'Procedural linear albedo, tangent normal and per-texel native roughness; moisture response and dielectric water absorption.',
        'display_transform':'ACES fitted curve and sRGB encoding, exposure 1.15',
        'water_optics':'Illustrative absorption from median suspended solid fraction; no calibrated turbidity or particle scattering',
        'image_generation_used':False})
    return report
