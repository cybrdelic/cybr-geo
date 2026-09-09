"""Write standard glTF 2.0 TRS animation channels into the exported GLBs.
All source geometry is retained. Y-up metres conversion is encoded in node TRS.
"""
from model import *
from movies import trajectory,velocity,FPS,smooth
import struct

def read_glb(path):
    b=path.read_bytes();magic,version,length=struct.unpack_from('<III',b,0)
    if magic!=0x46546c67 or version!=2 or length!=len(b):raise ValueError('Invalid glTF header')
    jlen,jtype=struct.unpack_from('<II',b,12);doc=json.loads(b[20:20+jlen]);offset=20+jlen;blen,btype=struct.unpack_from('<II',b,offset)
    if btype!=0x004e4942:raise ValueError('Missing BIN chunk')
    return doc,bytearray(b[offset+8:offset+8+blen])


def write_animation(src,dest,parts,mode):
    doc,buf=read_glb(src)
    def accessor(a,kind,limits=False):
        nonlocal buf
        a=np.ascontiguousarray(a,dtype='<f4')
        while len(buf)%4:buf.append(0)
        offset=len(buf);buf.extend(a.tobytes());vi=len(doc['bufferViews']);doc['bufferViews'].append(dict(buffer=0,byteOffset=offset,byteLength=a.nbytes))
        item=dict(bufferView=vi,componentType=5126,count=len(a),type=kind)
        if limits:item.update(min=np.atleast_1d(a.min(0)).tolist(),max=np.atleast_1d(a.max(0)).tolist())
        ai=len(doc['accessors']);doc['accessors'].append(item);return ai
    dur=19 if mode=='kinematic' else 12;times=np.arange(int(dur*FPS)+1,dtype=np.float32)/FPS;ti=accessor(times,'SCALAR',True)
    cs,ds=trajectory(dur) if mode=='kinematic' else (np.zeros(len(times)),np.zeros(len(times)))
    name_to_part={p.name:p for p in parts};channels=[];samplers=[];Q=rotation_x(-math.pi/2)
    max_error=0.
    for ni,node in enumerate(doc['nodes']):
        name=node.get('name')
        if name not in name_to_part:continue
        p=name_to_part[name];node.pop('matrix',None);node['scale']=[.001,.001,.001];node['rotation']=[-2**-.5,0,0,2**-.5];node['translation']=[0,0,0]
        rots=[];translations=[]
        for i,t in enumerate(times):
            c=float(cs[i]);d=float(ds[i]);e=0.
            if mode=='explode':
                if t<2:e=0.
                elif t<5.5:e=smooth((float(t)-2)/3.5)
                elif t<9.2:e=1.
                else:e=1-smooth((float(t)-9.2)/2.8)
            T=transform(p,c,d,e);R=Q@T[:3,:3]
            angle=math.atan2(R[2,1],R[1,1]);q=np.array([math.sin(angle/2),0,0,math.cos(angle/2)])
            if rots and np.dot(q,rots[-1])<0:q=-q
            rots.append(q);translations.append(Q@T[:3,3]*.001)
            # Independent axis-matrix reconstruction against the model pose.
            qp=Q@(T[:3,:3]@p.vertices[0]+T[:3,3])*.001
            rp=rotation_x(angle)@p.vertices[0]*.001+translations[-1]
            max_error=max(max_error,float(np.max(np.abs(qp-rp))))
        if mode=='kinematic':
            ai=accessor(rots,'VEC4');si=len(samplers);samplers.append(dict(input=ti,output=ai,interpolation='LINEAR'));channels.append(dict(sampler=si,target=dict(node=ni,path='rotation')))
        if mode=='explode' or p.motion in ['planetA','planetB']:
            ai=accessor(translations,'VEC3');si=len(samplers);samplers.append(dict(input=ti,output=ai,interpolation='LINEAR'));channels.append(dict(sampler=si,target=dict(node=ni,path='translation')))
    doc['animations']=[dict(name='Prescribed differential kinematics' if mode=='kinematic' else 'Reference exploded inspection',samplers=samplers,channels=channels)]
    doc.setdefault('extras',{})['engineering_status']='Visual / ideal-kinematic model. No torque bias, contact forces, stress, lubrication or manufacturing qualification.'
    doc['asset']['generator']='TORSEN inspection v3 / procedural meshes + standard glTF TRS animation'
    doc['buffers'][0]['byteLength']=len(buf)
    jb=json.dumps(doc,separators=(',',':')).encode();jb+=b' '*((-len(jb))%4);buf.extend(b'\0'*((-len(buf))%4))
    result=struct.pack('<III',0x46546c67,2,12+8+len(jb)+8+len(buf))+struct.pack('<II',len(jb),0x4e4f534a)+jb+struct.pack('<II',len(buf),0x004e4942)+buf
    dest.write_bytes(result)
    check,_=read_glb(dest)
    return dict(file=str(dest.relative_to(ROOT)),seconds=dur,channels=len(channels),keyframes=len(times),maximum_sampled_transform_error_m=max_error,file_bytes=len(result),valid_glb_header=True)

if __name__=='__main__':
    reports=[]
    for src,out,parts,mode in [('working_variant.glb','working_variant_animated.glb','working_variant_parts.npz','kinematic'),('kinematic_core.glb','kinematic_core_animated.glb','kinematic_core_parts.npz','kinematic'),('reference_unchanged.glb','reference_explosion_animated.glb','reference_parts.npz','explode')]:
        reports.append(write_animation(GEOM/src,GEOM/out,load_parts(GEOM/parts),mode))
    (ROOT/'validation'/'animated_glb_checks.json').write_text(json.dumps(reports,indent=2));print(json.dumps(reports,indent=2))
