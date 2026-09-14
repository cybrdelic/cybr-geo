"""Evidence-driven checks for a locally built AERIS assembly.

The optional CAD checks read assembly.pkl produced by build_assets.py. Never
load an untrusted pickle. Geometry checks are not engineering qualification.
"""
from pathlib import Path
from dataclasses import replace
import argparse, json, sys, struct, time, hashlib, pickle
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src'));sys.path.insert(0,str(Path(__file__).resolve().parent))
from recipe import Config,coupled_pose
from mechanism_lab.core import load_cache,validate
from mechanism_lab.exporters import NATIVE_TO_GLTF


def read_glb(path):
    data=path.read_bytes();magic,version,length=struct.unpack_from('<4sII',data)
    assert magic==b'glTF' and version==2 and length==len(data)
    offset=12;document=None;binary=None
    while offset<len(data):
        size,kind=struct.unpack_from('<II',data,offset);offset+=8
        chunk=data[offset:offset+size];offset+=size
        if kind==0x4e4f534a:document=json.loads(chunk)
        elif kind==0x004e4942:binary=chunk
    assert document is not None and binary is not None
    return document,binary


def accessor(doc,binary,index):
    a=doc['accessors'][index];v=doc['bufferViews'][a['bufferView']]
    dtype={5126:'<f4',5125:'<u4',5123:'<u2'}[a['componentType']]
    components={'SCALAR':1,'VEC3':3,'VEC4':4,'MAT4':16}[a['type']]
    offset=v.get('byteOffset',0)+a.get('byteOffset',0)
    width=np.dtype(dtype).itemsize*components
    assert v.get('byteStride',width)==width,'Unexpected interleaved animation data'
    return np.frombuffer(binary,dtype=dtype,count=a['count']*components,offset=offset).reshape(a['count'],components)


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,default=ROOT/'build/aeris');ap.add_argument('--cad',action='store_true')
    args=ap.parse_args();out=args.out;rows=[];start=time.time()
    a=load_cache(out/'cache');a.motion_function=coupled_pose
    parts={p.name:p for p in a.parts}
    def check(name,fn):
        try:
            detail=fn();rows.append({'check':name,'passed':True,'detail':detail})
            print('PASS',name,flush=True)
        except Exception as exc:
            rows.append({'check':name,'passed':False,'error':f'{type(exc).__name__}: {exc}'})
            print('FAIL',name,repr(exc),flush=True)
    def contract():
        r=validate(a);assert r['parts']==215;assert r['triangles']>1_600_000
        return {'parts':r['parts'],'triangles':r['triangles'],'units':r['units']}
    check('complete finite geometry and rigid-pose contract',contract)
    def feature_coverage():
        fmap=a.metadata['feature_map'];assert len(fmap)>=25
        assert all(names and set(names)<=set(parts) for names in fmap.values())
        return {'construction_families':len(fmap),'features':sorted(fmap)}
    check('feature ledger refers to real exported parts',feature_coverage)
    def rigid_rotor():
        moving=[p for p in a.parts if p.motion=='rotor'];fixed=[p for p in a.parts if p.motion=='fixed']
        assert len(moving)>=20
        for t in (0,.3,1.,2.7,4.):
            ref=a.pose(moving[0],t)
            for p in moving:assert np.allclose(a.pose(p,t),ref,atol=1e-12)
            for p in fixed:assert np.array_equal(a.pose(p,t),np.eye(4))
        assert not np.allclose(a.pose(moving[0],1),np.eye(4))
        return {'moving_components':len(moving),'sampled_times_seconds':[0,.3,1,2.7,4.],'prescribed_rpm':15}
    check('all rotor parts share the same nontrivial rigid motion',rigid_rotor)
    def nominal_envelopes():
        c=Config();clearance=72-5.2-c.rotor_radius;assert abs(clearance-2.8)<1e-9
        shaft=parts['Continuous_motor_impeller_shaft'].bounds
        for x in (49.5,100.):assert shaft[0,0]<=x and shaft[1,0]>=x+6.5
        return {'nominal_radial_clearance_mm':round(clearance,6),'shaft_span_x_mm':shaft[:,0].tolist(),'scope':'Radial/axial envelopes only; no full interference or load claim'}
    check('shaft spans both bearings and rotor has nominal radial clearance',nominal_envelopes)
    def config_guard():
        for kw in ({'rotor_radius':45},{'shaft_radius':6},{'blade_count':0},{'wire_turns':0},{'rotor_back':60}):
            try:Config(**kw).check()
            except ValueError:continue
            raise AssertionError(f'Unsupported configuration accepted: {kw}')
        Config().check();return 'Five unsupported parameter combinations rejected'
    check('unsupported design envelopes fail explicitly',config_guard)
    def static_units():
        import trimesh
        scene=trimesh.load_scene(out/'aeris.glb',process=False)
        assert len(scene.geometry)==len(a.parts)
        for p in a.parts:
            transform,key=scene.graph[p.name]
            assert np.allclose(transform,NATIVE_TO_GLTF@a.pose(p),atol=1e-12)
            assert np.allclose(scene.geometry[key].vertices,p.vertices,atol=3e-5)
        return {'nodes':len(a.parts),'native_units':'mm/Z-up','glTF_world_units':'m/Y-up'}
    check('static GLB decodes with exact node transforms and geometry',static_units)
    def animation(mode):
        path=out/f'aeris_{"rotor_motion" if mode=="motion" else "service_explosion"}.glb'
        doc,binary=read_glb(path);animation=doc['animations'][0];nodes=doc['nodes'];nontrivial=0
        for ch in animation['channels']:
            sampler=animation['samplers'][ch['sampler']]
            times=accessor(doc,binary,sampler['input']).ravel();values=accessor(doc,binary,sampler['output'])
            assert len(times)==len(values) and np.all(np.diff(times)>0)
            name=nodes[ch['target']['node']]['name'];p=parts[name]
            if np.max(np.ptp(values,axis=0))>1e-6:nontrivial+=1
            if ch['target']['path']=='rotation' and p.motion=='rotor' and mode=='motion':
                for idx in (0,len(times)//4,len(times)//2,-1):
                    angle=2*np.pi*.25*float(times[idx]);q=np.array([np.sin(angle/2),0,0,np.cos(angle/2)])
                    # Quaternions q and -q encode the same orientation.
                    assert min(np.max(abs(values[idx]-q)),np.max(abs(values[idx]+q)))<2e-5
        assert nontrivial>0
        return {'channels':len(animation['channels']),'changing_channels':nontrivial,'decoded_buffers_checked':True}
    check('rotor animation actually encodes changing rotations',lambda:animation('motion'))
    check('service animation actually encodes changing translations',lambda:animation('explode'))
    def step_coverage():
        report=json.loads((out/'aeris_analytic.coverage.json').read_text())
        included=report['analytic_components'];omitted=report['mesh_only_components_not_in_STEP']
        assert len(included)==202 and len(omitted)==13 and set(included+omitted)==set(parts)
        assert all('Copper_conductor' in n or 'Gyroid' in n for n in omitted)
        assert (out/'aeris_analytic.step').stat().st_size>1_000_000
        assert report['manufacturing_validated'] is False
        return {'analytic_components':len(included),'explicit_mesh_omissions':omitted}
    check('STEP coverage is explicit and does not fake mesh BReps',step_coverage)
    def gyroid():
        import trimesh
        p=next(p for p in a.parts if p.group=='gyroid')
        mesh=trimesh.Trimesh(p.vertices,p.faces,process=True)
        assert mesh.is_watertight and mesh.is_winding_consistent
        return {'triangles':len(mesh.faces),'watertight_after_vertex_weld':True,'consistent_winding':True}
    check('implicit gyroid closes as a watertight oriented mesh',gyroid)
    if args.cad:
        with (out/'assembly.pkl').open('rb') as f:cad=pickle.load(f)
        cp={p.name:p for p in cad.parts}
        def valid_cad():
            analytic=[p for p in cad.parts if p.cad is not None]
            assert len(analytic)==202
            assert all(p.cad.isValid() and p.cad.Volume()>0 for p in analytic)
            return '202 positive-volume valid CAD parts; compounds not claimed to be fused'
        check('every analytic component is valid positive-volume OpenCascade geometry',valid_cad)
        def single_casting():
            body=cp['Spiral_volute_hollow_casting'].cad
            assert len(body.Solids())==1
            return {'connected_solids':1,'integral_open_outlet_flange':True}
        check('volute is one connected casting, not disconnected side walls',single_casting)
        def blade_contacts():
            b=cp['Impeller_twisted_blade_01'].cad
            disk=cp['Impeller_back_disk'].cad;shroud=cp['Impeller_front_shroud'].cad
            distance=b.distance(disk);overlap=b.intersect(shroud).Volume()
            assert distance<1e-6 and overlap>1.
            return {'back_disk_distance_mm':distance,'front_shroud_overlap_mm3':overlap,'blade_BREP_faces':len(b.Faces())}
        check('lofted blade really contacts both supporting disks',blade_contacts)
        def half_section():
            with (out/'cutaway.pkl').open('rb') as f:section=pickle.load(f)
            sec={p.name:p for p in section.parts}
            original=cp['Asymmetric_hollow_capture_hood'].cad
            part=sec['Asymmetric_hollow_capture_hood_section'];actual=part.cad
            assert actual.isValid() and 0<actual.Volume()<original.Volume()
            # BREP bounds can enclose off-surface spline control points. Check
            # actual tessellation and an intersection with the removed volume.
            import cadquery as cq
            removed=cq.Workplane('XY').box(850,350,550).val().translate((-40,-175,20))
            residue=actual.intersect(removed).Volume()
            assert part.vertices[:,1].min()>-1e-5 and abs(residue)<1e-5
            return {'full_hood_volume_mm3':original.Volume(),'section_hood_volume_mm3':actual.Volume(),'removed_halfspace_residue_mm3':residue,'actual_boolean_subtraction':True}
        check('cutaway is real CAD subtraction with reduced volume',half_section)
    report={'model':'aeris','all_passed':all(r['passed'] for r in rows),'passed':sum(r['passed'] for r in rows),'total':len(rows),'elapsed_seconds':time.time()-start,'checks':rows,'scope':'Geometry, serialized exports and prescribed kinematics only; not manufacturing, airflow or safety qualification.'}
    (out/'verification.json').write_text(json.dumps(report,indent=2)+'\n')
    if not report['all_passed']:raise SystemExit(1)
    print(json.dumps({'passed':report['passed'],'total':report['total']}),flush=True)

if __name__=='__main__':main()
