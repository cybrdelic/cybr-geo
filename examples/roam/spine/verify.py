"""Exact all-pair interference audit. No collision exclusions, no swallowed errors."""
import os,sys,json,hashlib,copy,math,time
from pathlib import Path
import numpy as np
import cadquery as cq
from model import OUT,Model,DEFAULT,POSES,RANGES,AXES,pose_cad,fingerprint,T

def receipt_errors(receipt=None):
    d=receipt or json.loads((OUT/'cad/export-receipt.json').read_text());errors=[]
    if d['source_sha256']!=fingerprint():errors.append('source')
    for name,digest in d['files'].items():
        p=OUT/name
        if not p.exists() or hashlib.file_digest(p.open('rb'),'sha256').hexdigest()!=digest:errors.append(name)
    return errors
def load():
    d=json.loads((OUT/'cad/assembly.json').read_text());m=Model();m.frames=d['frames'];m.connections=d['connections']
    if d['source_sha256']!=fingerprint():raise ValueError('Source differs from exported CAD. Rebuild first.')
    errors=receipt_errors()
    if errors:raise ValueError('Changed export: '+', '.join(errors[:4]))
    def routed(n):return n.endswith('_harness_route') or n.endswith('_service_loop')
    # Regenerate parametric routing bodies from the fingerprinted source. Importing
    # their large OCCT sweep serialization is much slower than constructing them.
    # The saved BReps are still hashed above, and the complete STEP is round-tripped.
    m.parts=[dict(p,shape=None if routed(p['name']) else cq.Shape.importBrep(str(OUT/'cad'/(p['name']+'.brep')))) for p in d['parts']]
    for p in m.parts:
        if routed(p['name']):p['shape']=m.world_shape(p,DEFAULT)
    return m,d
def bounds(s):
    b=s.BoundingBox();return np.array([b.xmin,b.ymin,b.zmin,b.xmax,b.ymax,b.zmax])
CACHE={}
def audit(shapes,transforms=None,features=None,broad=None):
    names=list(shapes);bs=np.array([bounds(shapes[n]) for n in names]);bad=[];errors=[];exact=0;pairs=len(names)*(len(names)-1)//2
    for i,a in enumerate(names):
        js=np.where(np.all(np.minimum(bs[i,3:],bs[i+1:,3:])-np.maximum(bs[i,:3],bs[i+1:,:3])>1e-6,axis=1))[0]+i+1
        for j in js:
            b=names[j]
            if broad and (a in broad or b in broad):
                aa=np.asarray(broad.get(a,[bs[i]]));bb=np.asarray(broad.get(b,[bs[j]]))
                if not np.any(np.all(np.minimum(aa[:,None,3:],bb[None,:,3:])-np.maximum(aa[:,None,:3],bb[None,:,:3])>1e-6,axis=2)):continue
            exact+=1
            key=None
            if transforms is not None:
                rel=np.linalg.inv(transforms[a])@transforms[b]
                key=(a,b,tuple(np.round(rel[:3].ravel(),6)),(features or {}).get(a),(features or {}).get(b))
            try:
                if key in CACHE:
                    volume=CACHE[key]
                    if volume>.01:bad.append({'a':a,'b':b,'mm3':round(volume,5)})
                    continue
                common=shapes[a].intersect(shapes[b])
                if not common.isValid():raise ValueError('invalid Boolean result')
                volume=common.Volume()
                if key is not None:CACHE[key]=volume
                if volume>.01:bad.append({'a':a,'b':b,'mm3':round(volume,5)})
            except Exception as e:errors.append({'a':a,'b':b,'error':str(e)})
    return {'pairs':pairs,'exact_operations':exact,'overlaps':bad,'errors':errors,'passed':not bad and not errors}
def world(m,q):
    f,_=m.poses(q);return {p['name']:m.world_shape(p,q,f) for p in m.parts}
def evaluate(m,q):
    f,_=m.poses(q)
    shapes={p['name']:m.world_shape(p,q,f) for p in m.parts}
    features={s+'_gas_rod_envelope':q[s][0] for s in ['left','right','tray']}
    for p in m.parts:
        if p['name'].endswith('_harness_route') or p['name'].endswith('_service_loop'):features[p['name']]=tuple(q[p['name'].split('_')[0]])
    broad={}
    for n,d in getattr(m,'_route_data',{}).items():
        cs=np.asarray(d['bezier']);r=d['radius_mm'];broad[n]=np.c_[cs.min(axis=1)-r,cs.max(axis=1)+r]
    result=audit(shapes,{p['name']:f[p['group']] for p in m.parts},features,broad)
    from wiring import cached_pipe
    cached_pipe.cache_clear() # current shapes remain owned by Model; discard obsolete sweep cache
    return result
def run(mode='nominal'):
    m,d=load();start=time.perf_counter();cases={}
    for name,q in (POSES.items() if mode!='nominal' else [('Working',DEFAULT)]):
        r=audit(world(m,q));cases[name]=dict(q=q,**r);print(json.dumps({'case':name,'exact':r['exact_operations'],'overlaps':len(r['overlaps']),'first':r['overlaps'][:20],'errors':r['errors'][:2]}),flush=True)
    result={'source_sha256':fingerprint(),'passed':all(r['passed'] for r in cases.values()),'cases':cases,'elapsed_seconds':time.perf_counter()-start,
            'scope':'Every unordered pair of all nominal solids including equipment/caster envelopes, in listed discrete poses. Not continuous motion or strength qualification.'}
    (OUT/'verification'/('nominal.json' if mode=='nominal' else 'poses.json')).write_text(json.dumps(result,indent=2))
    return result
if __name__=='__main__':r=run(sys.argv[1] if len(sys.argv)>1 else 'nominal');os._exit(0 if r['passed'] else 1)
