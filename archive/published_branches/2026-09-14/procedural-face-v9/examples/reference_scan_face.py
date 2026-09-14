"""Legacy reference-only portrait assembly, based on the attributed Infinite head scan.

The photographed source is licensed CC BY 3.0, not an original scan by CYBR.
Geometry remains a real textured 3D surface in millimetres. This recipe uses
the same Assembly / Part / Material / View contract as the mechanical models.
"""
from pathlib import Path
import numpy as np
import trimesh
from scipy.ndimage import gaussian_filter, map_coordinates
from PIL import Image
from mechanism_lab.core import Assembly, Material, Part, View

ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT / 'assets' / 'portrait'


def subdivide_face(vertices, faces, uvs, levels=2):
    """Loop subdivision on welded topology with independent UV corner seams.

    Skin is welded for position stencils and normals. UVs are interpolated in
    their own chart. This avoids splitting the scalp silhouette at UV seams.
    """
    _, unique, inverse = np.unique(np.round(vertices, 7), axis=0,
                                    return_index=True, return_inverse=True)
    v=vertices[unique].copy(); f=inverse[faces]
    uv=uvs[faces]
    for _ in range(levels):
        pairs=np.concatenate([f[:,[0,1]],f[:,[1,2]],f[:,[2,0]]])
        edges, ei, counts=np.unique(np.sort(pairs,axis=1),axis=0,
                                    return_inverse=True,return_counts=True)
        opp=np.concatenate([f[:,2],f[:,0],f[:,1]])
        opposite=np.zeros((len(edges),3));np.add.at(opposite,ei,v[opp])
        ep=.375*(v[edges[:,0]]+v[edges[:,1]])+.125*opposite
        boundary=counts==1
        ep[boundary]=.5*(v[edges[boundary,0]]+v[edges[boundary,1]])
        valence=np.bincount(edges.ravel(),minlength=len(v))
        neighbor=np.zeros_like(v)
        np.add.at(neighbor,edges[:,0],v[edges[:,1]])
        np.add.at(neighbor,edges[:,1],v[edges[:,0]])
        beta=np.where(valence==3,3/16,3/(8*np.maximum(valence,1)))
        vp=(1-valence*beta)[:,None]*v+beta[:,None]*neighbor
        bedges=edges[boundary];bc=np.bincount(bedges.ravel(),minlength=len(v))
        bn=np.zeros_like(v)
        np.add.at(bn,bedges[:,0],v[bedges[:,1]])
        np.add.at(bn,bedges[:,1],v[bedges[:,0]])
        bm=bc==2;vp[bm]=.75*v[bm]+.125*bn[bm]
        ab,bc,ca=(ei.reshape(3,-1)+len(v))
        a,b,c=f.T
        f=np.concatenate([np.stack([a,ab,ca],1),np.stack([ab,b,bc],1),
                          np.stack([ca,bc,c],1),np.stack([ab,bc,ca],1)])
        ua,ub,uc=uv[:,0],uv[:,1],uv[:,2]
        uab=(ua+ub)/2;ubc=(ub+uc)/2;uca=(uc+ua)/2
        uv=np.concatenate([np.stack([ua,uab,uca],1),np.stack([uab,ub,ubc],1),
                           np.stack([uca,ubc,uc],1),np.stack([uab,ubc,uca],1)])
        v=np.concatenate([vp,ep])
    # One render vertex per distinct position/UV pair. Positions remain welded.
    corner_uv=uv.reshape(-1,2)
    keys=np.column_stack([f.ravel(),np.round(corner_uv*1e7).astype(np.int64)])
    _,take,remap=np.unique(keys,axis=0,return_index=True,return_inverse=True)
    geometric_index=f.ravel()[take]
    return v, f, v[geometric_index], remap.reshape(-1,3), corner_uv[take], geometric_index


def refined_skin(mesh):
    source_v=np.asarray(mesh.vertices)
    gv,gf,v,f,uv,gi=subdivide_face(source_v,np.asarray(mesh.faces),np.asarray(mesh.visual.uv),2)
    normals=np.zeros_like(gv)
    crosses=np.cross(gv[gf[:,1]]-gv[gf[:,0]],gv[gf[:,2]]-gv[gf[:,0]])
    for k in range(3):np.add.at(normals,gf[:,k],crosses)
    normals/=np.maximum(np.linalg.norm(normals,axis=1)[:,None],1e-12)
    # Fine scan-derived displacement: remove broad offsets already represented by
    # the scan, then retain only <= 25 micrometres of fine geometric relief.
    height=np.asarray(Image.open(ASSETS/'Infinite-Level_02_Disp_NoSmoothUV-4096.jpg').convert('L'),np.float32)/255.
    relief=height-gaussian_filter(height,4.)
    sampled=map_coordinates(relief,[(1-uv[:,1])*(height.shape[0]-1),uv[:,0]*(height.shape[1]-1)],order=1,mode='nearest')
    disp=np.clip(sampled*.18,-.025,.025)/40.
    # Average duplicate UV seam samples before moving the shared position.
    sums=np.bincount(gi,weights=disp,minlength=len(gv))
    counts=np.bincount(gi,minlength=len(gv))
    gv=gv+normals*(sums/np.maximum(counts,1))[:,None]
    v=gv[gi]
    return v,f,normals[gi],uv


def build():
    source = trimesh.load(ASSETS / 'LeePerrySmith.glb', process=False)
    mesh = next(iter(source.geometry.values()))
    # Source is Y-up, front +Z. CYBR GEO is Z-up, with this face looking -Y.
    rotation = np.array([[1., 0., 0.], [0., 0., -1.], [0., 1., 0.]])
    vertices,faces,normals,uv = refined_skin(mesh)
    vertices = vertices @ rotation.T * 40.
    normals = normals @ rotation.T
    skin = Part('Human_face_and_neck', vertices, faces, normals,
                material=0, group='anatomy', provenance='reference-scan',
                role='Infinite head scan by Lee Perry-Smith; CC BY 3.0',
                tags=('organic-surface', 'uv-mapped', 'reference-scan'))
    # Extra render data is explicit recipe data rather than inferred material IDs.
    skin.portrait_uv = uv
    assembly = Assembly(
        'human_face', [skin],
        [Material('Skin / scanned color and tangent normal', (.42, .22, .16),
                  rough=.43, ior=1.4, coat=.06, coat_rough=.35,
                  material_source='Lee Perry-Smith texture set / CC BY 3.0')],
        views={
            'portrait': View(az=-75, el=2.0, target=(0,-15,28), scale=136,
                             focal_length_mm=85, f_stop=16, floor=False),
            'front': View(az=-90, el=0, target=(0,-15,28), scale=136,
                          focal_length_mm=85, f_stop=16, floor=False),
            'profile': View(az=-38, el=1.0, target=(0,-10,28), scale=137,
                            focal_length_mm=85, f_stop=16, floor=False),
            'detail': View(az=-79, el=0.0, target=(0,-65,35), scale=79,
                           focal_length_mm=100, f_stop=11, floor=False),
        },
        metadata={
            'units': 'mm',
            'anatomy_source': 'Infinite, 3D Head Scan by Lee Perry-Smith',
            'license': 'CC BY 3.0',
            'source_url': 'https://github.com/mrdoob/three.js/tree/dev/examples/models/gltf/LeePerrySmith',
            'renderer_reference': 'CYBR GEO ORBIT v9, commit 40797a6fa756ce209b5e21dd83f2df1e41dbd5fb',
            'geometry_operations': ['Two Loop subdivision levels on welded topology',
                                    'Independent UV corner interpolation',
                                    '4K scan height detail, high-pass filtered, capped at 25 micrometres',
                                    'Millimetre / Z-up CYBR GEO assembly integration'],
            'limitations': ['Scan-derived anatomy, not an original procedural human generator.',
                            'The supplied skin color texture has finite 1024px resolution.',
                            'Surface material is a skin-like BSDF, not measured multilayer tissue.'],
        })
    return assembly
