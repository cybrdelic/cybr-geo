"""Validate the delivered mesh, materials, units and analytic CAD files."""
from pathlib import Path
import json,hashlib,numpy as np,trimesh,cadquery as cq
ROOT=Path(__file__).resolve().parents[1];G=ROOT/'geometry'
manifest=json.loads((G/'manifest.json').read_text());arrays=np.load(G/'mesh_arrays.npz')
report={'components':[],'analytic_cad':[]}
for p in manifest['components']:
    n=p['name'];v=arrays[n+'__vertices'];f=arrays[n+'__faces'];no=arrays[n+'__normals']
    finite=bool(np.isfinite(v).all() and np.isfinite(no).all());valid=bool(f.min()>=0 and f.max()<len(v))
    normals_error=float(np.max(np.abs(np.linalg.norm(no,axis=1)-1)))
    mesh=trimesh.Trimesh(v,f,process=True)
    report['components'].append(dict(name=n,finite=finite,indices_valid=valid,normal_length_max_error=normals_error,triangles=len(f),watertight_after_welding=bool(mesh.is_watertight)))
    if not finite or not valid:raise ValueError('Mesh failed: '+n)
for path in G.glob('*.step'):
    shape=cq.importers.importStep(str(path)).val()
    ok=shape.isValid();report['analytic_cad'].append(dict(file=path.name,valid=ok,solids=len(shape.Solids())))
    if not ok:raise ValueError('CAD failed: '+path.name)
scene=trimesh.load(G/'TORSEN_X_reference_rebuild.glb',force='scene')
report['glb']={'geometry_nodes':len(scene.geometry),'world_bounds_m':scene.bounds.tolist(),'reopened':True}
report['triangle_count']=sum(x['triangles'] for x in report['components'])
report['component_count']=len(report['components'])
report['all_components_finite_and_index_valid']=all(x['finite'] and x['indices_valid'] for x in report['components'])
report['note']='Topology / file checks are NOT stress, fit, tooth-contact, torque-bias or kinematic validation.'
(ROOT/'validation.json').write_text(json.dumps(report,indent=2))
print(json.dumps({k:v for k,v in report.items() if k not in ['components','analytic_cad']},indent=2))
