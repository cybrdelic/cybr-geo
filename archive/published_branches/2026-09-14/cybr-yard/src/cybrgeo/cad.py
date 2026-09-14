"""STEP assembly importer with hierarchical placement, labels and instance colors."""
from pathlib import Path
import hashlib
import numpy as np
from .core import Assembly, Material, from_shape, safe_name

def import_step(path, tolerance=.08, axis='native', split_solids=True) -> Assembly:
    import cadquery as cq
    path=Path(path);source=cq.Assembly.importStep(str(path))
    parts=[];palette=[];cad={};audit=[]
    for shape,full_name,location,color in source:
        placed=shape.moved(location)
        if axis=='motor-x':
            # Source shaft is Y axis. Set X shaft axis, preserve handedness.
            placed=placed.rotate((0,0,0),(0,0,1),-90).translate((-40,0,0))
        elif axis!='native':raise ValueError('axis must be native or motor-x')
        solids=placed.Solids()
        # Non-solid vendor shell components are retained, not discarded.
        bodies=solids if split_solids and solids else [placed]
        rgba=color.toTuple() if color else (.48,.5,.53,1)
        rgb=tuple(float(v) for v in rgba[:3])
        material=Material(name='Vendor_'+str(len(palette)),color=rgb,
                          metal=.8 if max(rgb)-min(rgb)<.16 and sum(rgb)>.65 else .15,rough=.3)
        try:mi=next(i for i,m in enumerate(palette) if m.color==rgb)
        except StopIteration:mi=len(palette);palette.append(material)
        base=safe_name(full_name.split('/')[-1])
        for j,body in enumerate(bodies):
            name=f'{len(parts)+1:03d}_{base}'+(f'_solid{j+1}' if len(bodies)>1 else '')
            p=from_shape(name,body,mi,tolerance,role='Manufacturer STEP geometry',
                         group='vendor',metadata={'source_node':full_name,'is_solid':bool(body.Solids())})
            parts.append(p);cad[name]=body
        audit.append({'source_node':full_name,'solid_count':len(solids),'retained_bodies':len(bodies)})
    return Assembly(path.stem,parts,palette,
      {'source':str(path.name),'source_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),
       'provenance':'Imported manufacturer geometry, not an independently recovered production design',
       'import_audit':audit,'coordinate_mapping':axis},cad)
