"""Portable geometry handoff. Materials/rendering remain external authorities."""
from pathlib import Path
import json
from .model import save_snapshot


def export_obj(mesh,path):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    uv=mesh.attributes.get('uv')
    normal=mesh.attributes.get('normal')
    if normal is not None:
        count={'vertex':len(mesh.vertices), 'corner':3*len(mesh.faces)}.get(normal.domain)
        if count is None or normal.values.shape!=(count,3):
            raise ValueError('OBJ normal handoff supports vertex or face-corner Nx3 data')
    if uv is not None:
        count={'vertex':len(mesh.vertices), 'corner':3*len(mesh.faces)}.get(uv.domain)
        if count is None or uv.values.shape!=(count,2):
            raise ValueError('OBJ handoff supports vertex or face-corner 2D UVs')
    with path.open('w') as out:
        out.write('# CYBR GEO evaluated surface; no embedded material authority\n')
        for x,y,z in mesh.vertices:out.write(f'v {x:.12g} {y:.12g} {z:.12g}\n')
        if uv is not None:
            for u,v in uv.values:out.write(f'vt {u:.12g} {v:.12g}\n')
        if normal is not None:
            for x,y,z in normal.values:out.write(f'vn {x:.12g} {y:.12g} {z:.12g}\n')
        for j,face in enumerate(mesh.faces):
            indices=[]
            for k,i in enumerate(face):
                ti=str(3*j+k+1 if uv.domain=='corner' else i+1) if uv is not None else ''
                ni=str(3*j+k+1 if normal.domain=='corner' else i+1) if normal is not None else ''
                indices.append(f'{i+1}/{ti}/{ni}' if normal is not None else (f'{i+1}/{ti}' if uv is not None else str(i+1)))
            out.write('f '+' '.join(indices)+'\n')
    return path


def export_bundle(mesh,directory,units='unitless',material_references=None):
    """Retain attributes and external material IDs without copying material logic."""
    if units not in ('unitless','mm','cm','m'):raise ValueError('Declare supported units explicitly')
    references=dict(material_references or {})
    if not all(isinstance(k,str) and isinstance(v,str) for k,v in references.items()):
        raise ValueError('Material references must map region names to external ID strings')
    for name in references:
        if name!='*' and 'region:'+name not in mesh.attributes:raise ValueError('Material region does not exist: '+name)
    directory=Path(directory);directory.mkdir(parents=True,exist_ok=True)
    save_snapshot(mesh,directory/'geometry.npz');export_obj(mesh,directory/'surface.obj')
    manifest=dict(schema='cybrgeo-geometry/0.1',units=units,geometry='geometry.npz',preview_exchange='surface.obj',
                  material_references=references,validation=mesh.report(),
                  integration_status='portable geometry handoff; native CYBR MATERIAL/LIGHT adapter not implemented')
    (directory/'manifest.json').write_text(json.dumps(manifest,indent=2))
    return directory/'manifest.json'
