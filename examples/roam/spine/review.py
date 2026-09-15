"""Bound evidence for original CAD, selected positions, service and intended motion."""
import os,sys,json,hashlib,copy,time,math
import numpy as np
import cadquery as cq
from model import OUT,DEFAULT,POSES,RANGES,AXES,PIVOT,T,R,cyl,pose_cad,fingerprint
from verify import load,evaluate,audit,bounds,world,receipt_errors

def hashes():
    files=[OUT/n for n in ['model.py','wiring.py','dynamic.py','engineering.py','verify.py','review.py','export_inspection.py','build_checked.py','requirements.json','cad/assembly.json','cad/workstation.step','cad/workstation.glb','cache/manifest.json','cache/meshes.npz','inspection/manifest.json','inspection/joint.glb','inspection/section.glb']]
    files+=list((OUT/'cad').glob('*.brep'))+list((OUT/'motion').glob('*.json'))
    files.append(OUT/'cad/export-receipt.json')
    return {str(p.relative_to(OUT)).replace('\\','/'):hashlib.file_digest(p.open('rb'),'sha256').hexdigest() for p in files}
def material(shape,probe):return shape.intersect(probe).Volume()/probe.Volume()>.999
SERVICE_BOUNDS={}
def interference(moving,others):
    bb=bounds(moving);bad=[];names=[];bs=[]
    for n,s in others.items():
        if id(s) not in SERVICE_BOUNDS:SERVICE_BOUNDS[id(s)]=(s,bounds(s))
        names.append(n);bs.append(SERVICE_BOUNDS[id(s)][1])
    bs=np.asarray(bs)
    for i in np.where(np.all(np.minimum(bb[3:],bs[:,3:])-np.maximum(bb[:3],bs[:,:3])>1e-6,axis=1))[0]:
        n=names[i];c=moving.intersect(others[n])
        if not c.isValid():raise ValueError('invalid service Boolean '+n)
        if c.Volume()>.01:bad.append({'other':n,'mm3':c.Volume()})
    return bad
def service(m):
    base=world(m,DEFAULT);fs,_=m.poses(DEFAULT);results=[]
    # Remove equipment first, before touching load-bearing hardware.
    base={n:s for n,s in base.items() if not n.endswith('monitor_envelope') and n!='keyboard_mouse_tray' and not n.endswith('_harness_route') and not n.endswith('_service_loop')}
    for side in ['left','right','tray']:
        for label,frame in [('shoulder','arm1'),('elbow','arm2'),('swivel','head'),('roll_joint','roll')]:
            stem=side+'_'+label;others=base.copy();steps=[];axis=fs[side+'_'+frame][:3,2]
            if side=='tray' and label=='roll_joint':
                for name in list(others):
                    if name.startswith('tray_roll_lock_'):others.pop(name)
            for suffix,travel,spacing in [('locknut',-20,2),('shoulder_screw',80,5),('top_spacer',60,5),('bush',20,2)]:
                name=stem+'_'+suffix;s=others.pop(name);bad=[]
                for d in np.linspace(0,travel,int(abs(travel)/spacing)+1):
                    clashes=interference(s.translate(tuple(axis*d)),others)
                    if clashes:bad.append({'travel_mm':float(d),'clashes':clashes})
                steps.append({'part':name,'travel_mm':travel,'sample_mm':spacing,'passed':not bad,'failures':bad})
            results.append({'joint':stem,'scope':'Working position; equipment and harness disconnected/removed first; tray roll clamp hardware removed first. Listed pivot parts removed sequentially. Harness and clamp removal are prerequisites, not validated by this sequence.','steps':steps,'passed':all(x['passed'] for x in steps)})
        # Open gimbal serviced on bench, after detaching the head from the arm.
        keep=[p['name'] for p in m.parts if p['group'] in [side+'_head',side+'_pitch',side+'_roll'] and not p['name'].endswith('monitor_envelope') and p['name']!='keyboard_mouse_tray' and not p['name'].endswith('_harness_route') and not p['name'].endswith('_service_loop') and not p['name'].startswith('tray_pitch_lock_') and not p['name'].startswith('tray_roll_lock_')]
        for sign in [-1,1]:
            others={n:base[n] for n in keep};steps=[];axis=fs[side+'_pitch'][:3,0]*sign
            plan=[(side+f'_pitch_nut_{sign}',axis,-20),(side+f'_pitch_bolt_{sign}',axis,60),(side+f'_pitch_outer_spacer_{sign}',axis,40)]
            # Bottom screws must come out before the independently manufactured cheek.
            plan += [(side+f'_fork_fix_{sign}_{j}',-fs[side+'_head'][:3,2],50) for j in (0,1)]
            plan += [(side+f'_fixed_cheek_{sign}',fs[side+'_head'][:3,2],180),(side+f'_pitch_thrust_{sign}',axis,35),(side+f'_pitch_bush_{sign}',axis,20)]
            for name,ax,travel in plan:
                s=others.pop(name);bad=[]
                for d in np.linspace(0,travel,int(abs(travel)/5)+1):
                    c=interference(s.translate(tuple(ax*d)),others)
                    if c:bad.append({'travel_mm':float(d),'clashes':c})
                steps.append({'part':name,'travel_mm':travel,'sample_mm':5,'passed':not bad,'failures':bad})
            results.append({'joint':side+f'_pitch_{sign}','scope':'Detached/unloaded head on bench, wiring and tray clamp hardware removed first. Remove screwed-on outer cheek before extracting flange bushing. Not an in-place cartridge extraction.','steps':steps,'passed':all(x['passed'] for x in steps)})
    SERVICE_BOUNDS.clear()
    return results
def probe_report(m):
    d={p['name']:p['shape'] for p in m.parts};checks=[]
    for side in ['left','right','tray']:
        for label in ['shoulder','elbow','swivel','roll_joint']:
            stem=side+'_'+label
            for name,probe,description in [(stem+'_bush',cyl(9.5,8.2,8.8,8.5),'retaining flange exists'),(stem+'_shoulder_screw',cyl(4.8,-37,-31),'male nominal thread core exists through nut'),(stem+'_locknut',cyl(6.5,-37,-31,5.5),'nut body exists around nominal engaged thread')]:
                checks.append({'part':name,'check':description,'passed':material(d[name],probe)})
    return checks
def coverage_valid(cases):
    expected={(s,i) for s in RANGES for i in range(len(RANGES[s]))}
    if len(cases)!=len(expected) or expected!={(x['support'],x['axis']) for x in cases}:return False
    for x in cases:
        side,i=x['support'],x['axis'];lo,hi=RANGES[side][i];spacing=180 if side=='base' else 50 if i==0 else 30
        coords=np.linspace(lo,hi,math.ceil((hi-lo)/spacing)+1)
        if x['limits']!=[lo,hi] or len(x['cases'])!=len(coords):return False
        if not np.allclose([c['coordinate'] for c in x['cases']],coords,rtol=0,atol=1e-9):return False
        if any(c['passed'] != (not c['errors'] and not c['overlaps']) for c in x['cases']):return False
    return True
def run():
    start=time.perf_counter();m,meta=load();poses={};previous=None;reused=None
    if '--reuse-motion' in sys.argv:
        digest=sys.argv[sys.argv.index('--reuse-motion')+1];raw=(OUT/'verification/review.json').read_bytes()
        if hashlib.sha256(raw).hexdigest()!=digest:raise ValueError('Motion evidence receipt changed')
        previous=json.loads(raw);now=hashes();old=previous['fingerprints']
        if set(now)!=set(old) or any(now[n]!=old[n] for n in now if n!='review.py'):raise ValueError('A geometry, verifier or presentation input changed; full review required')
        if not previous['unchanged_during_review'] or not coverage_valid(previous['motion']):raise ValueError('Incomplete or invalid cached movement coverage')
        reused={'report_sha256':digest,'verified_all_other_fingerprints':True,'scope':'Only previously evaluated motion samples; selected poses, service, probes, defects and precise STEP comparison rerun.'}
    if previous is None:
        from dynamic import dynamic_geometry
        (OUT/'motion').mkdir(exist_ok=True)
        for name,q in POSES.items():
            (OUT/'motion'/(name.lower().replace(' ','-')+'.json')).write_text(json.dumps({'q':q,'meshes':dynamic_geometry(m,q)},separators=(',',':')))
    before=hashes()
    for name,q in POSES.items():
        poses[name]=evaluate(m,q);print(json.dumps({'pose':name,'passed':poses[name]['passed'],'clashes':len(poses[name]['overlaps'])}),flush=True)
    (OUT/'verification/selected-poses.json').write_text(json.dumps(poses,indent=2))
    if not all(x['passed'] for x in poses.values()):raise ValueError('Selected poses failed; see selected-poses.json')
    # Cover the full requested adjustment envelope, with the other coordinates at
    # the default position. Collision regions are recorded, never silently trimmed.
    if previous is not None:
        motion=previous['motion'];print(json.dumps({'reused_motion_samples':sum(len(x['cases']) for x in motion),'bound_report':reused['report_sha256']}),flush=True)
    else:
        motion=[]
        for side,ranges in RANGES.items():
            for i,(lo,hi) in enumerate(ranges):
                spacing=180 if side=='base' else 50 if i==0 else 30;values=np.linspace(lo,hi,math.ceil((hi-lo)/spacing)+1);cases=[]
                for v in values:
                    q=copy.deepcopy(DEFAULT);q[side][i]=float(v)
                    try:r=evaluate(m,q)
                    except Exception as e:r={'passed':False,'overlaps':[],'errors':[{'geometry':str(e)}]}
                    cases.append({'coordinate':float(v),'passed':r['passed'],'overlaps':r['overlaps'],'errors':r['errors']})
                motion.append({'support':side,'axis':i,'name':'extension' if side=='base' else AXES[i],'limits':[lo,hi],'max_spacing':spacing,'cases':cases})
                (OUT/'verification/movement-progress.json').write_text(json.dumps({'source_sha256':fingerprint(),'motion':motion},indent=2))
                print(json.dumps({'support':side,'axis':i,'samples':len(cases),'blocked':sum(not c['passed'] for c in cases)}),flush=True)
            print(json.dumps({'motion_support':side,'covered_axes':len(ranges)}),flush=True)
    access=service(m);print(json.dumps({'service_cases':len(access),'passed':all(x['passed'] for x in access),'failures':[x['joint'] for x in access if not x['passed']]}),flush=True)
    probes=probe_report(m);d={p['name']:p['shape'] for p in m.parts}
    bad_bush=d['left_elbow_bush'].translate((.5,0,0));flange=cyl(9.5,8.2,8.8,8.5);thread=cyl(4.8,-37,-31)
    defects=[{'defect':'bearing displaced radially 0.5 mm','rejected':bool(audit({'plate':d['left_link_2_root'],'bush':bad_bush})['overlaps'])},
             {'defect':'retaining flange removed','rejected':not material(d['left_elbow_bush'].cut(cyl(10.1,8,10)),flange)},
             {'defect':'male thread engagement removed','rejected':not material(d['left_elbow_shoulder_screw'].cut(cyl(5.1,-47,-30)),thread)},
             {'defect':'one required coordinate coverage omitted','rejected':not coverage_valid(motion[:-1])},
             {'defect':'source bytes changed after export','rejected':hashlib.sha256((OUT/'model.py').read_bytes()+b'\n# deliberate change'+(OUT/'wiring.py').read_bytes()).hexdigest()!=meta['source_sha256']}]
    stale=json.loads((OUT/'cad/export-receipt.json').read_text());key=next(k for k in stale['files'] if k.endswith('.brep'));stale['files'][key]='0'*64
    defects.append({'defect':'analytic BRep differs from the export receipt','rejected':key in receipt_errors(stale)})
    imported=cq.importers.importStep(str(OUT/'cad/workstation.step')).val().Solids();roundtrip={'integration_tolerance':1e-9,'allowed_volume_error_mm3':.1,'bodies':len(imported),'all_valid':all(s.isValid() for s in imported),'volume_error_mm3':abs(sum(s.Volume(1e-9) for s in imported)-sum(m.world_shape(p,DEFAULT).Volume(1e-9) for p in m.parts))}
    jac={}
    for side in ['left','right','tray']:
        f,_=m.poses(DEFAULT);A=f[side+'_roll'];columns=[]
        for i in range(6):
            q=copy.deepcopy(DEFAULT);eps=.01;q[side][i]+=eps;B=m.poses(q)[0][side+'_roll'];D=(B[:3,:3]@A[:3,:3].T-np.eye(3))/eps
            columns.append(np.r_[(B[:3,3]-A[:3,3])/eps/500,[D[2,1],D[0,2],D[1,0]]])
        jac[side]={'rank':int(np.linalg.matrix_rank(np.array(columns).T,tol=1e-7)),'coordinate_count':6,'note':'Local differential mobility at Working only; extended straight-arm configurations can be singular.'}
    insp=json.loads((OUT/'inspection/manifest.json').read_text());after=hashes()
    selected_pass=all(r['passed'] for r in poses.values());accesspass=all(x['passed'] for x in access)
    result={'schema':1,'reused_motion_evidence':reused,'fingerprints':before,'unchanged_during_review':before==after,'selected_poses_passed':selected_pass,'service_passed':accesspass,
            'geometry_review_passed':selected_pass and accesspass and all(x['passed'] for x in probes) and all(x['rejected'] for x in defects) and coverage_valid(motion) and all(x['passed'] for x in insp['exploded_cases']) and roundtrip['all_valid'] and roundtrip['bodies']==len(m.parts) and roundtrip['volume_error_mm3']<.1 and before==after,
            'fabrication_release':False,'parts':len(m.parts),'poses':poses,'motion':motion,'motion_coverage_complete':coverage_valid(motion),'full_envelope_collision_free':all(c['passed'] for x in motion for c in x['cases']),
            'service':access,'retention_probes':probes,'deliberate_defects':defects,'STEP_roundtrip':roundtrip,'mobility':jac,'elapsed_seconds':time.perf_counter()-start,
            'limits':'All pair exact nominal solid interference. Selected poses and listed component removal paths only. The complete adjustment envelope contains blocked configurations. Supplier envelopes, friction locks, hardware specifications, cable routing and load capacity require further qualification.'}
    (OUT/'verification/review.json').write_text(json.dumps(result,indent=2));print(json.dumps({k:result[k] for k in ['geometry_review_passed','selected_poses_passed','service_passed','parts','elapsed_seconds']}),flush=True)
    return result
if __name__=='__main__':
    try:r=run();os._exit(0 if r['geometry_review_passed'] else 1)
    except BaseException:
        import traceback;traceback.print_exc();sys.stderr.flush();os._exit(1)
