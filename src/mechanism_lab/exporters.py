"""Unit-explicit geometry, analytic CAD, manifest and standard glTF animation export."""
from __future__ import annotations
import csv,json,struct,math
from pathlib import Path
import numpy as np
import trimesh
from .core import Assembly,Part,validate,pose_cad

# Native mm / Z-up -> standard glTF metres / Y-up.
NATIVE_TO_GLTF=np.array([[.001,0,0,0],[0,0,.001,0],[0,-.001,0,0],[0,0,0,1]],float)

# Procedural microgeometry in the native tracer must be an explicit material
# choice. It is never inferred from metallicity: doing that silently invented
# machining marks on every metal. Pattern ids 5-7 are consumed only by the
# photographic tracer; the legacy renderer safely treats unknown ids as plain.
MICROFINISH_PATTERNS={
    'none':0,
    'machined':1,
    'turned':1,
    'bead-blasted':2,
    'brushed':3,
    'polymer':4,
    'drawn-wire':5,
    'copper-wire':6,
    'anodized':7,
}

def linear_to_srgb(x):
    x=np.clip(np.asarray(x),0,1)
    return np.where(x<=.0031308,12.92*x,1.055*np.power(x,1/2.4)-.055)

def scene(assembly, time_seconds=0,explode=0):
    s=trimesh.Scene();materials=[]
    for m in assembly.materials:
        rgba=np.r_[np.round(linear_to_srgb(m.color)*255).astype(np.uint8),round(m.opacity*255)]
        materials.append(trimesh.visual.material.PBRMaterial(name=m.name,baseColorFactor=rgba,metallicFactor=m.metal,roughnessFactor=m.rough))
    for p in assembly.parts:
        mesh=trimesh.Trimesh(p.vertices,p.faces,vertex_normals=p.normals,process=False)
        mesh.visual=trimesh.visual.TextureVisuals(material=materials[p.material])
        s.add_geometry(mesh,node_name=p.name,geom_name=p.name,transform=assembly.pose(p,time_seconds,explode))
    s.apply_transform(NATIVE_TO_GLTF)
    return s

def export_glb(assembly,path,time_seconds=0,explode=0):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    scene(assembly,time_seconds,explode).export(path)
    return path

def export_step(assembly,path,individual=True):
    """Export only existing BReps; never pass tessellated substitutes off as analytic CAD."""
    import cadquery as cq
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    cad=cq.Assembly(name=assembly.name);included=[];omitted=[]
    folder=path.parent/'step_parts'
    if individual:folder.mkdir(exist_ok=True)
    for p in assembly.parts:
        if p.cad is None:omitted.append(p.name);continue
        m=assembly.materials[p.material];color=linear_to_srgb(m.color)
        world=pose_cad(p.cad,assembly.pose(p))
        cad.add(world,name=p.name,color=cq.Color(*color))
        if individual:cq.exporters.export(world,str(folder/(p.name+'.step')))
        included.append(p.name)
    if not included:raise ValueError('No analytic BRep bodies; use GLB/STL for mesh-only geometry')
    cad.export(str(path),exportType='STEP',mode='default')
    report=dict(units='mm',axis='X-shaft/Z-up',analytic_components=included,mesh_only_components_not_in_STEP=omitted,
                full_mesh_assembly=path.stem.replace('_analytic','')+'.glb',manufacturing_validated=False)
    path.with_suffix('.coverage.json').write_text(json.dumps(report,indent=2)+'\n')
    return report

def export_stls(assembly,directory):
    directory=Path(directory);directory.mkdir(parents=True,exist_ok=True)
    for p in assembly.parts:
        mesh=trimesh.Trimesh(p.vertices,p.faces,process=False)
        mesh.apply_transform(assembly.pose(p,0,0))
        mesh.export(directory/(p.name+'.stl'))
    (directory/'UNITS.txt').write_text('All STL coordinates are millimetres in assembled world pose at time zero; shaft X, up Z. STL does not encode units.\n')

def export_bom(assembly,directory):
    directory=Path(directory);directory.mkdir(parents=True,exist_ok=True)
    rows=[dict(id=f'{i+1:03}',name=p.name,group=p.group,material=assembly.materials[p.material].name,
               role=p.role,provenance=p.provenance,triangles=len(p.faces),analytic_cad=p.cad is not None) for i,p in enumerate(assembly.parts)]
    (directory/'parts.json').write_text(json.dumps(rows,indent=2)+'\n')
    with (directory/'parts.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    return rows

def export_meshbin(assembly,path, time_seconds=0,explode=0):
    path=Path(path);n=sum(len(p.faces) for p in assembly.parts)
    with path.open('wb') as f:
        f.write(struct.pack('<I',n))
        for p in assembly.parts:
            T=assembly.pose(p,time_seconds,explode)
            v=p.vertices@T[:3,:3].T+T[:3,3];norm=p.normals@T[:3,:3].T
            records=np.c_[v[p.faces].reshape(-1,9),norm[p.faces].reshape(-1,9),np.full(len(p.faces),p.material),np.full(len(p.faces),10)]
            f.write(records.astype('<f4').tobytes())
    with path.with_suffix('.materials').open('w') as f:
        for m in assembly.materials:
            finish=str(m.microfinish).strip().lower()
            if finish not in MICROFINISH_PATTERNS:
                raise ValueError(f'Unknown microfinish {m.microfinish!r} on material {m.name!r}; use {sorted(MICROFINISH_PATTERNS)}')
            pattern=MICROFINISH_PATTERNS[finish]
            f.write(' '.join(map(str,(*m.color,m.metal,m.rough,pattern)))+'\n')

# A tiny standard glTF writer adds node TRS animation to the exported static GLB.
# Geometry stays immutable, only prescribed rigid transforms change.
def export_animated_glb(assembly,path,duration=8.,fps=24,mode='motion'):
    from scipy.spatial.transform import Rotation
    if duration<=0 or fps<=0:raise ValueError('duration and fps must be positive')
    raw=scene(assembly).export(file_type='glb')
    length,kind=struct.unpack_from('<II',raw,12);doc=json.loads(raw[20:20+length])
    bpos=20+length;binlen,binkind=struct.unpack_from('<II',raw,bpos)
    data=bytearray(raw[bpos+8:bpos+8+binlen]);data=data[:doc['buffers'][0]['byteLength']]
    doc.setdefault('bufferViews',[]);doc.setdefault('accessors',[])
    def accessor(a,type_,bounds=False):
        a=np.asarray(a,dtype='<f4')
        while len(data)%4:data.append(0)
        offset=len(data);data.extend(a.tobytes())
        vi=len(doc['bufferViews']);doc['bufferViews'].append(dict(buffer=0,byteOffset=offset,byteLength=a.nbytes))
        ac=dict(bufferView=vi,componentType=5126,count=len(a),type=type_)
        if bounds:ac.update(min=np.atleast_1d(a.min(axis=0)).tolist(),max=np.atleast_1d(a.max(axis=0)).tolist())
        idx=len(doc['accessors']);doc['accessors'].append(ac);return idx
    times=np.linspace(0,duration,round(duration*fps)+1,dtype=np.float32);time_id=accessor(times,'SCALAR',True)
    samplers=[];channels=[];residual=0.
    lookup={n.get('name'):i for i,n in enumerate(doc['nodes'])}
    roots=doc['scenes'][doc.get('scene',0)]['nodes']
    if len(roots)!=1 or doc['nodes'][roots[0]].get('mesh') is not None:
        raise ValueError('Expected a single non-mesh world root')
    root=doc['nodes'][roots[0]]
    for key in ('translation','rotation','scale'):root.pop(key,None)
    root['matrix']=NATIVE_TO_GLTF.T.reshape(-1).tolist()
    for p in assembly.parts:
        ni=lookup[p.name];node=doc['nodes'][ni];node.pop('matrix',None)
        transforms=[]
        for t in times:
            e=(.5-.5*math.cos(math.tau*t/duration)) if mode=='explode' else 0
            transforms.append(assembly.pose(p,float(t) if mode=='motion' else 0,e))
        transforms=np.asarray(transforms)
        quat=Rotation.from_matrix(transforms[:,:3,:3]).as_quat().astype(np.float32)
        for k in range(1,len(quat)):
            if np.dot(quat[k-1],quat[k])<0:quat[k]*=-1
        translations=transforms[:,:3,3].astype(np.float32)
        node['translation']=translations[0].tolist();node['rotation']=quat[0].tolist();node['scale']=[1.,1.,1.]
        for data_values,type_,target in [(quat,'VEC4','rotation'),(translations,'VEC3','translation')]:
            if np.max(np.abs(data_values-data_values[0]))<1e-8:continue
            aid=accessor(data_values,type_);si=len(samplers);samplers.append(dict(input=time_id,output=aid,interpolation='LINEAR'))
            channels.append(dict(sampler=si,target=dict(node=ni,path=target)))
        recovered=Rotation.from_quat(quat).as_matrix();residual=max(residual,float(np.max(np.abs(recovered-transforms[:,:3,:3]))))
    doc['animations']=[dict(name=f'{assembly.name}_{mode}',samplers=samplers,channels=channels)]
    doc['buffers'][0]['byteLength']=len(data)
    js=json.dumps(doc,separators=(',',':')).encode();js+=b' '*((-len(js))%4);data.extend(b'\0'*((-len(data))%4))
    body=struct.pack('<II',len(js),0x4E4F534A)+js+struct.pack('<II',len(data),0x004E4942)+data
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(struct.pack('<III',0x46546C67,2,12+len(body))+body)
    report=dict(file=path.name,duration=duration,samples=len(times),animated_channels=len(channels),rotation_roundtrip_error=residual,
                note='Prescribed rigid-body kinematics. Not electromagnetic or contact-force simulation.')
    path.with_suffix('.animation.json').write_text(json.dumps(report,indent=2)+'\n')
    return report
