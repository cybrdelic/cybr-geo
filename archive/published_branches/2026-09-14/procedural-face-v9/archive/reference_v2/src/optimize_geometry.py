"""Create a compact rendering/export mesh while retaining analytic STEP parts.
The very fine OCC tessellation of the curved housing is reduced using a
normal-aware quadric metric. Surface distance is sampled and reported.
"""
from pathlib import Path
import importlib.util,json,shutil,numpy as np,vtk
from vtk.util.numpy_support import numpy_to_vtk,vtk_to_numpy,numpy_to_vtkIdTypeArray
spec=importlib.util.spec_from_file_location('build',str(Path(__file__).with_name('build_geometry.py')))
b=importlib.util.module_from_spec(spec);spec.loader.exec_module(b)
m=json.loads((b.OUT/'manifest.json').read_text());a=np.load(b.OUT/'mesh_arrays.npz')
report={}
for p in m['components']:
    name=p['name'];v=a[name+'__vertices'];f=a[name+'__faces'];n=a[name+'__normals']
    if p['group']=='carrier':
        poly=vtk.vtkPolyData();pts=vtk.vtkPoints();pts.SetData(numpy_to_vtk(v,deep=True));poly.SetPoints(pts)
        cells=vtk.vtkCellArray();q=np.column_stack([np.full(len(f),3,np.int64),f.astype(np.int64)]).ravel()
        cells.SetCells(len(f),numpy_to_vtkIdTypeArray(q,deep=True));poly.SetPolys(cells)
        nn=numpy_to_vtk(n,deep=True);nn.SetName('Normals');poly.GetPointData().SetNormals(nn)
        dec=vtk.vtkQuadricDecimation();dec.SetInputData(poly);dec.SetTargetReduction(.88)
        dec.VolumePreservationOn();dec.AttributeErrorMetricOn();dec.NormalsAttributeOn();dec.SetNormalsWeight(.15)
        dec.Update();out=dec.GetOutput()
        vnew=vtk_to_numpy(out.GetPoints().GetData()).copy()
        fnew=vtk_to_numpy(out.GetPolys().GetData()).reshape(-1,4)[:,1:].copy()
        nnew=vtk_to_numpy(out.GetPointData().GetNormals()).copy()
        nnew/=np.maximum(np.linalg.norm(nnew,axis=1)[:,None],1e-10)
        # Validation is against actual source triangles, not just vertices.
        loc=vtk.vtkStaticCellLocator();loc.SetDataSet(poly);loc.BuildLocator()
        rng=np.random.default_rng(20260909);ids=rng.choice(len(vnew),min(5000,len(vnew)),replace=False)
        distances=[]
        for vi in ids:
            closest=[0.,0.,0.];cell=vtk.reference(0);sub=vtk.reference(0);d2=vtk.reference(0.)
            loc.FindClosestPoint(vnew[vi],closest,cell,sub,d2);distances.append(float(d2)**.5)
        report=dict(original_triangles=len(f),compact_triangles=len(fnew),
                    tested_vertices=len(ids),max_sampled_distance_mm=max(distances),
                    p99_sampled_distance_mm=float(np.quantile(distances,.99)),
                    note='One-way sampled distance, not a certified Hausdorff bound. Analytic STEP remains unchanged.')
        print(report,flush=True)
        v,f,n=vnew,fnew,nnew
    b.PARTS.append(dict(name=name,vertices=v,faces=f,normals=n,material=p['material'],group=p['group']))
(b.OUT/'tessellation_validation.json').write_text(json.dumps(report,indent=2))
backup=b.ROOT/'working_full_resolution';backup.mkdir(exist_ok=True)
for fname in ['TORSEN_X_reference_rebuild.glb','mesh_arrays.npz','manifest.json']:
    shutil.copy2(b.OUT/fname,backup/fname)
b.export_all()
