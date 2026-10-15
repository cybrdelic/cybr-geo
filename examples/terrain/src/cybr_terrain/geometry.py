"""Terrain state -> actual CYBR GEO Assembly, indexed meshes and PBR GLB.

Simulation uses metres. The GEO contract uses millimetres/Z-up; glTF exports
metres/Y-up. Layer walls follow the surviving thickness field, not painted bands.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
import numpy as np
from scipy.ndimage import zoom, gaussian_filter, map_coordinates, binary_dilation
import trimesh
from mechanism_lab.core import Assembly, Part, Material, View, validate
from mechanism_lab.exporters import scene as geo_scene
from .model import State, LAYERS, LOOSE_POROSITY
from .materials import Atlas, make_atlas, make_boundary_atlas, exposure_weights, srgb


@dataclass
class Attributes:
    uv: np.ndarray
    tint: np.ndarray


@dataclass
class Geometry:
    assembly: Assembly
    attributes: dict[str,Attributes]
    atlases: dict[int,Atlas]
    water_material: int
    state: State

    def validate(self):
        for p in self.assembly.parts:
            a=self.attributes[p.name]
            if len(a.uv)!=len(p.vertices) or len(a.tint)!=len(p.vertices):raise ValueError("Attribute length mismatch")
            if not np.isfinite(p.vertices).all() or not np.isfinite(p.normals).all():raise ValueError("Non-finite mesh")
            if np.min(p.faces)<0 or np.max(p.faces)>=len(p.vertices):raise ValueError("Invalid face index")
            if not np.isfinite(a.uv).all() or not np.isfinite(a.tint).all() or a.tint.min()<0:raise ValueError("Invalid surface attributes")
            if not np.allclose(np.linalg.norm(p.normals,axis=1),1,atol=2e-5):raise ValueError("Invalid surface normals")
        # GEO's own contract also checks names, materials and array dimensions.
        report=validate(self.assembly)
        return report

    def export_glb(self,path):
        s=geo_scene(self.assembly)
        clamped_materials=set()
        for part in self.assembly.parts:
            mesh=s.geometry[part.name];a=self.attributes[part.name]
            if part.material in self.atlases:
                atlas=self.atlases[part.material]
                # glTF vertex colours cannot exceed one. Absorb the scalar
                # into the texture so cross-interface pigment blends retain
                # the same linear product as the native render.
                gain=np.maximum(1,np.max(a.tint,axis=0))
                color=Image_from(srgb(atlas.albedo*gain));normal=Image_from(atlas.normal*.5+.5)
                orm=Image_from(np.stack((np.ones_like(atlas.roughness),atlas.roughness,np.zeros_like(atlas.roughness)),axis=-1))
                material=trimesh.visual.material.PBRMaterial(name=self.assembly.materials[part.material].name,
                    baseColorFactor=[255,255,255,255],baseColorTexture=color,normalTexture=normal,
                    metallicFactor=0,roughnessFactor=1,metallicRoughnessTexture=orm,doubleSided=False)
                if not atlas.repeat:clamped_materials.add(material.name)
                mesh.visual=trimesh.visual.TextureVisuals(uv=a.uv,material=material)
                mesh.visual.vertex_attributes['color']=np.c_[np.round(np.clip(a.tint/gain,0,1)*255).astype('uint8'),np.full(len(a.tint),255,'uint8')]
            else:
                # Core glTF PBR has no volumetric spectral water absorption.
                m=trimesh.visual.material.PBRMaterial(name='Water raster preview',baseColorFactor=[35,65,70,100],
                    metallicFactor=0,roughnessFactor=.08,alphaMode='BLEND',doubleSided=True)
                mesh.visual=trimesh.visual.TextureVisuals(uv=a.uv,material=m)
        def contact_samplers(tree):
            if not clamped_materials:return
            sampler=len(tree.setdefault('samplers',[]))
            tree['samplers'].append({'wrapS':33071,'wrapT':33071,'magFilter':9729,'minFilter':9729})
            for material in tree['materials']:
                if material.get('name') not in clamped_materials:continue
                pbr=material.get('pbrMetallicRoughness',{})
                textures=[pbr.get('baseColorTexture'),pbr.get('metallicRoughnessTexture'),material.get('normalTexture')]
                for texture in textures:
                    if texture is not None:tree['textures'][texture['index']]['sampler']=sampler
        path=Path(path);path.parent.mkdir(parents=True,exist_ok=True);s.export(path,tree_postprocessor=contact_samplers)
        restored=trimesh.load(path,force='scene')
        if len(restored.geometry)!=len(s.geometry):raise RuntimeError("GLB component count mismatch")
        return path


def Image_from(array):
    from PIL import Image
    return Image.fromarray(np.round(np.clip(array,0,1)*255).astype('uint8'))


def normal_field(z,dx):
    dy,dxz=np.gradient(z,dx);n=np.stack((-dxz,-dy,np.ones_like(z)),axis=-1)
    return n/np.linalg.norm(n,axis=-1,keepdims=True)


def grid_faces(n):
    a=np.arange(n*n).reshape(n,n);ll=a[:-1,:-1].ravel();lr=a[:-1,1:].ravel()
    ul=a[1:,:-1].ravel();ur=a[1:,1:].ravel()
    return np.vstack((np.stack((ll,lr,ur),axis=1),np.stack((ll,ur,ul),axis=1)))


def build_geometry(state:State,directory,subdivision=2,stones=500,contact_resolution=2048):
    if subdivision not in (1,2,3,4) or not 0<=stones<=5000:raise ValueError("Invalid geometry detail setting")
    if not isinstance(contact_resolution,int) or not 16<=contact_resolution<=2048:raise ValueError('Invalid contact texture resolution')
    state.validate();c=state.config;directory=Path(directory);n=(c.grid-1)*subdivision+1
    if n>1025:raise ValueError("Display geometry is limited to 1025 vertices per side")
    def expand(a):
        return zoom(a,(n/c.grid,n/c.grid),order=1)[:n,:n]
    z=expand(state.height);water=expand(state.water);sat=expand(state.saturation)
    axis=np.linspace(-c.extent/2,c.extent/2,n);x,y=np.meshgrid(axis,axis);dx=c.extent/(n-1)
    # Interpolate the actual surviving beds before choosing exposure. Enlarging
    # the solver's categorical exposed index with nearest-neighbour sampling
    # made whole solver cells into staircase pigment patches in close views.
    # A metre-scale layer can remain present as a thin wedge between samples;
    # it must not disappear merely because the neighbouring cell is depleted.
    beds=np.stack([expand(thickness) for thickness in state.thickness])
    layer=np.full((n,n),-1,dtype=int)
    for k,thickness in enumerate(beds):layer=np.where(thickness>1e-5,k,layer)
    bed_layer=layer.copy()
    loose_grains=np.stack([expand(grain) for grain in state.loose])
    loose=loose_grains.sum(axis=0)
    # A dusting must not erase the underlying lithology. Coverage is an
    # explicitly authored optical response to the actual deposited bulk depth:
    # full cover requires at least 1cm, or half the substrate's grain scale.
    # Deposited volume remains entirely represented in state.height already.
    substrate_grain=np.array([l.grain_scale for l in state.layers])[np.maximum(bed_layer,0)]
    full_cover_depth=np.maximum(.010,.50*substrate_grain)
    loose_bulk=loose/(1-LOOSE_POROSITY)
    deposit_coverage=np.clip(loose_bulk/full_cover_depth,0,1)
    layer=np.where(deposit_coverage>=1,len(state.layers),bed_layer)
    all_layers=(*state.layers,LAYERS['sand'])
    wet=np.clip(sat*2+water/.025,0,1);wet_bin=np.minimum(3,(wet*4).astype(int))
    rng=np.random.default_rng(c.seed+19)
    rough=gaussian_filter(rng.normal(size=(n,n)),.75);rough/=max(rough.std(),1e-9)
    medium=gaussian_filter(rng.normal(size=(n,n)),2.2);medium/=max(medium.std(),1e-9)
    coarse=gaussian_filter(rng.normal(size=(n,n)),6);coarse/=max(coarse.std(),1e-9)
    # Render-only subgrid clods, pits and resistant fragments. Their physical
    # scale stays centimetric when the domain extent changes, and the solver's
    # conserved beds are never altered by this surface realization.
    amplitude=np.array([min(max(l.grain_scale*.30,.008),dx*.20) for l in all_layers])[np.maximum(layer,0)]
    amplitude=np.where(layer>=0,amplitude,0)
    meso=.54*rough+.30*medium+.16*coarse-.14*np.maximum(rough,0)**2
    display_z=z+meso*amplitude*(1-wet*.38)
    vertices=np.stack((x,y,display_z),axis=-1).reshape(-1,3)
    normals=normal_field(display_z,dx).reshape(-1,3);uv=np.stack((x/1.25,y/1.25),axis=-1).reshape(-1,2)
    faces=grid_faces(n);top_material=np.maximum(layer,0)*4+wet_bin
    face_material=np.sort(top_material.ravel()[faces],axis=1)[:,1]
    # One shader owns every crossing triangle and a bounded two-cell halo.
    # Independent repeating textures cannot cancel their high-frequency
    # differences using vertex colour, even when the average pigment agrees.
    crossing=np.any(top_material.ravel()[faces]!=top_material.ravel()[faces[:,0,None]],axis=1)
    contact_vertices=np.zeros((n,n),bool)
    contact_vertices.ravel()[faces[crossing].ravel()]=True
    contact_vertices=binary_dilation(contact_vertices,iterations=2)
    contact_faces=np.any(contact_vertices.ravel()[faces],axis=1)
    parts=[];attrs={};materials=[];atlases={};material_map={}
    for i,l in enumerate(all_layers):
        for w in range(4):
            index=len(materials);material_map[i*4+w]=index;name=f'{i:02}_{l.name.lower().replace(" ","_")}_wet{w}'
            atlas=make_atlas(l,w/3).save(directory/'textures',name)
            materials.append(Material(name,tuple(np.asarray(l.color)*(1-.42*w/3)),rough=float(np.median(atlas.roughness)),material_source='Procedural '+l.name+'; illustrative mineral response'))
            atlases[index]=atlas
    water_material=len(materials);materials.append(Material('Water',(.92,.96,.98),rough=.015,ior=1.333,opacity=.25))
    def part(name,v,f,normals,material,uv,tint,group):
        if len(f)==0:return
        used,indices=np.unique(f,return_inverse=True);v=np.asarray(v)[used];normals=np.asarray(normals)[used]
        p=Part(name,v*1000,indices.reshape(-1,3).astype('uint32'),normals.astype('float32'),material=material,
               group=group,role='Simulated terrain '+group,provenance='designed-concept',tags=('terrain','SI-to-mm'))
        local_tint=np.asarray(tint)[used]
        if material in atlases:
            # Keep native textured reflectance below one, and ensure that
            # GLB rebasing never clips the atlas on any individual channel.
            ceiling=.995/np.maximum(atlases[material].albedo.max(axis=(0,1)),1e-9)
            local_tint=np.minimum(local_tint,ceiling)
        parts.append(p);attrs[name]=Attributes(np.asarray(uv)[used].astype('float32'),local_tint.astype('float32'))
    tint=np.clip(1+.055*medium.reshape(-1,1)+.025*coarse.reshape(-1,1),.68,1.18)*np.ones((len(vertices),3))
    # Weathered boundaries are a blend across a small subgrid fringe; actual
    # interface heights remain authoritative. This removes the former polygon
    # staircase pigment edges without changing any simulated strata.
    weights=exposure_weights(bed_layer,wet_bin,len(state.layers))
    blended=np.zeros((n,n,3))
    for key,weight in weights.items():blended+=weight[:,:,None]*np.asarray(all_layers[key//4].color)
    # Actual gravel/sand/fines composition colours the graded coating; pigment
    # changes continuously even where the substrate switches to a full mantle.
    fractions=loose_grains/np.maximum(loose[None,:,:],1e-20)
    deposit_color=sum(fractions[i,:,:,None]*np.asarray(color) for i,color in enumerate(((.24,.23,.20),(.49,.35,.20),(.23,.13,.06))))
    blended=blended*(1-deposit_coverage[:,:,None])+deposit_color*deposit_coverage[:,:,None]
    for key in np.unique(face_material):
        selected=faces[(face_material==key)&~contact_faces]
        primary=all_layers[max(0,min(len(all_layers)-1,int(key)//4))]
        local_tint=tint*np.clip(blended.reshape(-1,3)/np.maximum(primary.color,.001),.30,3.5)
        bin_wet=(int(key)%4)/3
        local_tint*=((1-.48*wet.ravel())/(1-.48*bin_wet))[:,None]
        part('surface_'+str(key),vertices,selected,normals,material_map.get(int(key),0),uv,local_tint,'surface')
    contact_material=None
    if contact_faces.any():
        contact_material=len(materials)
        atlas_weights=exposure_weights(layer,wet_bin,len(all_layers))
        contact=make_boundary_atlas(atlases,atlas_weights,wet,blended,c.extent,contact_resolution)
        contact.save(directory/'textures','surface_contact_world')
        atlases[contact_material]=contact
        materials.append(Material('surface_contact_world',(.5,.5,.5),rough=float(np.median(contact.roughness)),
                                  material_source='Blended actual bed exposure and graded deposited coating; clamped world-UV PBR'))
        world_uv=np.stack(((x+c.extent/2)/c.extent,(y+c.extent/2)/c.extent),axis=-1).reshape(-1,2)
        # Pigment and moisture responses are already baked; retain only the
        # same weathering tint used by the unmodified interior materials.
        part('surface_contact',vertices,faces[contact_faces],normals,contact_material,world_uv,tint,'surface')
    # The four vertical cut faces use every surviving geological interface.
    foundation=expand(state.foundation);levels=[foundation]
    for t in beds:levels.append(levels[-1]+t)
    # Extend or clip the highest surviving bed to the displayed microrelief,
    # so the exposed block closes against the surface without hairline gaps.
    for j in range(1,len(levels)):
        above=expand(state.thickness[j:].sum(axis=0)) if j<len(state.layers) else np.zeros_like(z)
        levels[j]=np.where((above<1e-9)&(loose<1e-9),display_z,np.minimum(levels[j],display_z))
    levels.append(display_z)
    sides=[(np.arange(n),np.zeros(n,int),(0,-1,0)),(np.full(n,n-1),np.arange(n),(1,0,0)),
           (np.arange(n-1,-1,-1),np.full(n,n-1),(0,1,0)),(np.zeros(n,int),np.arange(n-1,-1,-1),(-1,0,0))]
    for side,(ix,iy,normal) in enumerate(sides):
        for k in range(len(levels)-1):
            low=levels[k][iy,ix];high=levels[k+1][iy,ix]
            v=np.vstack((np.c_[x[iy,ix],y[iy,ix],low],np.c_[x[iy,ix],y[iy,ix],high]))
            a=np.arange(n-1);f=np.vstack((np.c_[a,a+1,a+n+1],np.c_[a,a+n+1,a+n]))
            valid=(high[a]+high[a+1]-low[a]-low[a+1])>1e-8
            f=f[np.tile(valid,2)]
            cross=np.cross(v[f[:,1]]-v[f[:,0]],v[f[:,2]]-v[f[:,0]])
            valid_area=np.linalg.norm(cross,axis=1)>1e-12;f=f[valid_area];cross=cross[valid_area]
            f[np.einsum('ij,j->i',cross,normal)<0]=f[np.einsum('ij,j->i',cross,normal)<0][:,::-1]
            normals_side=np.tile(normal,(len(v),1))
            along=np.r_[np.arange(n),np.arange(n)]*dx/1.25
            side_uv=np.c_[along,v[:,2]/1.25]
            # Irregular weathering replaces uniform painted-looking bands;
            # bedding microrelief itself is carried by the shale normal map.
            side_tint=np.ones((len(v),3))*(.90+.065*np.sin(v[:,2]*17+.8*np.sin(along*3))+.035*np.cos(along*11))[:,None]
            mat=material_map[min(k,len(all_layers)-1)*4]
            part(f'cut_{side}_layer_{k}',v,f,normals_side,mat,side_uv,side_tint,'strata')
    # A planar foundation closes the block, with an intentional immutable base.
    floor=float(foundation.min())-.35
    for side,(ix,iy,normal) in enumerate(sides):
        v=np.vstack((np.c_[x[iy,ix],y[iy,ix],np.full(n,floor)],np.c_[x[iy,ix],y[iy,ix],foundation[iy,ix]]))
        a=np.arange(n-1);f=np.vstack((np.c_[a,a+n+1,a+1],np.c_[a,a+n,a+n+1]))
        cross=np.cross(v[f[:,1]]-v[f[:,0]],v[f[:,2]]-v[f[:,0]])
        bad=np.einsum('ij,j->i',cross,normal)<0;f[bad]=f[bad][:,::-1]
        part(f'base_wall_{side}',v,f,np.tile(normal,(len(v),1)),0,np.c_[v[:,0]/1.25,v[:,2]/1.25],np.ones((len(v),3)),'foundation')
    v=np.array([[-c.extent/2,-c.extent/2,floor],[c.extent/2,-c.extent/2,floor],[c.extent/2,c.extent/2,floor],[-c.extent/2,c.extent/2,floor]])
    part('block_bottom',v,np.array([[0,2,1],[0,3,2]]),np.tile((0,0,-1),(4,1)),0,v[:,:2]/1.25,np.ones((4,3)),'foundation')
    # Close each connected triangulated water region, including bank edges.
    wf=faces[np.mean(water.ravel()[faces],axis=1)>.003]
    if len(wf):
        eta=z+np.maximum(water,.0001)+.002
        wv=np.c_[x.ravel(),y.ravel(),eta.ravel()];wn=normal_field(eta,dx).reshape(-1,3)
        part('water_surface',wv,wf,wn,water_material,uv,np.ones((n*n,3)),'water')
        part('water_bottom',np.c_[x.ravel(),y.ravel(),z.ravel()+.001],wf[:,::-1],-normal_field(z,dx).reshape(-1,3),water_material,uv,np.ones((n*n,3)),'water')
        edges=np.vstack((wf[:,[0,1]],wf[:,[1,2]],wf[:,[2,0]]));canonical=np.sort(edges,axis=1)
        unique,inv,count=np.unique(canonical,axis=0,return_inverse=True,return_counts=True);boundary=edges[count[inv]==1]
        wall_v=[];wall_f=[];wall_n=[]
        for edge in boundary:
            a,b=edge;corners=np.array([wv[a],wv[b],[wv[b,0],wv[b,1],z.ravel()[b]+.001],[wv[a,0],wv[a,1],z.ravel()[a]+.001]])
            norm=-np.cross(corners[1]-corners[0],corners[2]-corners[0]);norm/=max(np.linalg.norm(norm),1e-20)
            start=len(wall_v);wall_v.extend(corners);wall_n.extend([norm]*4);wall_f.extend([[start,start+2,start+1],[start,start+3,start+2]])
        if wall_v:
            v=np.asarray(wall_v);part('water_banks',v,np.asarray(wall_f),np.asarray(wall_n),water_material,v[:,:2]/1.25,np.ones((len(v),3)),'water')
    # Actual aggregate meshes and fractured stones. These render-only particles
    # are not additional material in the conservative solver's inventory.
    loose_fraction=state.loose[0]/np.maximum(state.loose.sum(axis=0),1e-12)
    substrate_gravel=np.array([l.fractions[0] for l in state.layers])[np.maximum(state.exposed,0)]
    probability=.035+.6*expand(substrate_gravel)+.7*expand(loose_fraction)
    slope=np.hypot(*np.gradient(z,dx));probability*=np.exp(-slope*2)*(1-np.minimum(water/.1,.95))
    probability/=probability.sum()
    focus=np.exp(-((x/(c.extent*.23))**2+((y+c.extent*.18)/(c.extent*.22))**2))
    fine_probability=probability*(.30+2.8*focus);fine_probability/=fine_probability.sum()
    fine_count=min(stones*45,22500)
    scatter_count=0
    if stones:
        # A dozen asymmetric convex prototypes avoid the old repeated spheres.
        from scipy.spatial import ConvexHull
        prototypes=[]
        for _ in range(16):
            cloud=rng.normal(size=(22,3));cloud/=np.linalg.norm(cloud,axis=1,keepdims=True)
            cloud*=rng.uniform(.70,1.10,(len(cloud),1))
            cloud[:,2]=np.clip(cloud[:,2],-.58,rng.uniform(.45,.83))
            hull=ConvexHull(cloud);faces_chunk=hull.simplices.copy()
            cross=np.cross(cloud[faces_chunk[:,1]]-cloud[faces_chunk[:,0]],cloud[faces_chunk[:,2]]-cloud[faces_chunk[:,0]])
            inward=np.einsum('ij,ij->i',cross,cloud[faces_chunk].mean(axis=1))<0
            faces_chunk[inward]=faces_chunk[inward][:,::-1]
            prototypes.append((cloud,faces_chunk))
        indices=rng.choice(n*n,size=stones,p=probability.ravel())
        groups={}
        for idx in indices:
            iy,ix=divmod(int(idx),n);cloud,faces_chunk=prototypes[rng.integers(len(prototypes))]
            key=int(top_material[iy,ix]);key=key if key in material_map else 0
            bed=all_layers[key//4]
            radius=np.clip(rng.lognormal(np.log(.083),.59),.024,.30)
            # Fine-rich soil breaks into small aggregates, whereas resistant
            # bedrock and alluvial gravel produce larger lithic fragments.
            if bed.fractions[0]<.20 and ('horizon' in bed.name or 'soil' in bed.name):radius*=.35
            # A modest authored lithic admixture represents fragments from the
            # available parent beds; aggregates retain the current soil colour.
            # No invented mineral species or simulated grain inventory is added.
            if rng.random()<.30:
                parent=[j for j,l in enumerate(all_layers[:-1])
                        if 'basalt' in l.name.lower() or 'sandstone' in l.name.lower()
                        or 'parent material' in l.name.lower() or 'carbonate' in l.name.lower()]
                if parent:key=int(rng.choice(parent))*4+int(top_material[iy,ix])%4
            scales=radius*np.array([rng.uniform(.85,1.55),rng.uniform(.7,1.2),rng.uniform(.45,.85)])
            local=cloud*scales;angle=rng.uniform(0,2*np.pi);co,si=np.cos(angle),np.sin(angle)
            rotation=np.array([[co,-si,0],[si,co,0],[0,0,1]]);local=local@rotation.T
            # Jitter avoids the artificial one-stone-per-grid-point layout.
            px=np.clip(x[iy,ix]+rng.uniform(-dx*.45,dx*.45),axis[0],axis[-1]);py=np.clip(y[iy,ix]+rng.uniform(-dx*.45,dx*.45),axis[0],axis[-1])
            pz=float(map_coordinates(display_z,[[(py-axis[0])/dx],[(px-axis[0])/dx]],order=1,mode='nearest')[0])
            local+=np.array([px,py,pz+radius*.07])
            # Explicit independent face normals retain fracture planes.
            vertices_chunk=local[faces_chunk].reshape(-1,3)
            ns=np.cross(local[faces_chunk[:,1]]-local[faces_chunk[:,0]],local[faces_chunk[:,2]]-local[faces_chunk[:,0]])
            ns/=np.linalg.norm(ns,axis=1,keepdims=True)
            vv,ff,nn,tt=groups.setdefault(key,([],[],[],[]));start=len(vv)
            vv.extend(vertices_chunk);ff.extend(np.arange(start,start+len(vertices_chunk)).reshape(-1,3));nn.extend(np.repeat(ns,3,axis=0))
            pigment=np.asarray((1,.97,.92))*rng.uniform(.66,1.03)
            tt.extend(np.tile(pigment,(len(vertices_chunk),1)));scatter_count+=1
        for key,(vv,ff,nn,tt) in groups.items():
            v=np.asarray(vv)
            part('fractured_chunks_'+str(key),v,np.asarray(ff),np.asarray(nn),material_map[key],v[:,:2]/1.25,np.asarray(tt),'stones')
        # A separate dense population resolves centimetre soil aggregates in
        # close views. Indexed twenty-triangle clods bound the geometry budget.
        indices=rng.choice(n*n,size=fine_count,p=fine_probability.ravel())
        iy,ix=np.divmod(indices,n);px=np.clip(x[iy,ix]+rng.uniform(-dx*.5,dx*.5,fine_count),axis[0],axis[-1]);py=np.clip(y[iy,ix]+rng.uniform(-dx*.5,dx*.5,fine_count),axis[0],axis[-1])
        pz=map_coordinates(display_z,[(py-axis[0])/dx,(px-axis[0])/dx],order=1,mode='nearest')
        grain=trimesh.creation.icosphere(subdivisions=0);gv=np.asarray(grain.vertices);gf=np.asarray(grain.faces)
        radius=np.clip(rng.lognormal(np.log(.012),.48,fine_count),.004,.040)
        scales=radius[:,None]*rng.uniform([.75,.65,.42],[1.30,1.10,.84],(fine_count,3))
        local=gv[None,:,:]*scales[:,None,:]*rng.uniform(.83,1.14,(fine_count,len(gv),1))
        angle=rng.uniform(0,2*np.pi,fine_count);co=np.cos(angle)[:,None];si=np.sin(angle)[:,None]
        vx=local[:,:,0].copy();vy=local[:,:,1].copy();local[:,:,0]=co*vx-si*vy;local[:,:,1]=si*vx+co*vy
        local+=np.stack((px,py,pz+radius*.08),axis=1)[:,None,:]
        # Area weighted vertex normals describe the actual deformed clods.
        fn=np.cross(local[:,gf[:,1]]-local[:,gf[:,0]],local[:,gf[:,2]]-local[:,gf[:,0]])
        normals_clod=np.zeros_like(local)
        for face in range(len(gf)):
            for corner in gf[face]:normals_clod[:,corner]+=fn[:,face]
        normals_clod/=np.linalg.norm(normals_clod,axis=2,keepdims=True)
        keys=top_material[iy,ix]
        for key in np.unique(keys):
            selected=np.flatnonzero(keys==key);count=len(selected);key=int(key);mat=material_map.get(key,0)
            v=local[selected].reshape(-1,3);f=(gf[None,:,:]+np.arange(count)[:,None,None]*len(gv)).reshape(-1,3)
            t=np.repeat(rng.uniform(.65,1.08,(count,1)),len(gv),axis=0)*np.asarray((1,.98,.94))
            part('soil_aggregates_'+str(key),v,f,normals_clod[selected].reshape(-1,3),mat,v[:,:2]/1.25,t,'aggregates')
    target=(0,0,float(np.median(z))*1000)
    assembly=Assembly('cybr_terrain_'+c.preset,parts,materials,views={'hero':View(scale=c.extent*650,target=target)},
        metadata={'generator':'CYBR TERRAIN 0.2','simulation_units':'metres','geometry_units':'millimetres',
                  'grid':c.grid,'display_grid':n,'display_mesogeometry_max_scale_m':float(amplitude.max()),
                  'strata':'actual surviving interfaces','water_mesh_threshold_m':.003,
                  'exposure':'highest interpolated surviving bed, bulk thickness >10 micrometres',
                  'deposit_coating':'authored linear optical coverage from actual deposited bulk depth; gravel/sand/fines pigment mix',
                  'deposit_full_coverage_min_bulk_m':.010,'deposit_full_coverage_substrate_grain_multiplier':.50,
                  'deposit_full_coverage_fraction':float((deposit_coverage>=1).mean()),
                  'deposit_mean_optical_coverage':float(deposit_coverage.mean()),
                  'scatter':'render-only fractured stones and soil aggregates; not additional simulated solid inventory',
                  'fractured_chunks':scatter_count,'soil_aggregates':fine_count,
                  'aggregate_radius_m':[.004,.040],'stone_radius_m':[.0084,.30],
                  'scatter_detail_distribution':'substrate/slope/moisture weighted, with additional detail in lower-y central close-view region',
                  'chunk_lithology':'authored 30% admixture of available parent-bed materials; fine aggregates follow exposed material',
                  'material_atlas_period_m':1.25,'material_atlas_resolution':512,
                  'surface_contact_material':contact_material,'surface_contact_faces':int(contact_faces.sum()),
                  'surface_contact_halo_display_cells':2,'surface_contact_weight_support_display_cells':2,
                  'surface_contact_atlas_resolution':contact_resolution if contact_material is not None else None,
                  'surface_contact_texel_size_m':c.extent/contact_resolution if contact_material is not None else None,
                  'surface_contact_mapping':'one clamped normalized world-UV material for crossing triangles and halo; existing periodic atlases retained in interiors',
                  'surface_detail':'centimetric multiscale weathering plus periodic aggregate/mineral/pore normal maps'})
    result=Geometry(assembly,attrs,atlases,water_material,state);result.validate();return result
