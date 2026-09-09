"""Geometry/metadata contract. Internal coordinates are millimetres, X-axis shafts, Z-up.

Recipes supply parts, named motion and views. Rendering, export, drawings, video and
packaging consume this contract; they never need a motor-specific rewrite.
"""
from __future__ import annotations
from dataclasses import dataclass, field, replace, asdict
from pathlib import Path
from typing import Any, Callable
import hashlib, json, math, os
import numpy as np

@dataclass(frozen=True)
class Material:
    name: str
    color: tuple[float, float, float]
    metal: float = 0.0
    rough: float = 0.4
    def as_dict(self): return dict(name=self.name, color=list(self.color), metal=self.metal, rough=self.rough)

@dataclass
class Part:
    name: str
    vertices: np.ndarray
    faces: np.ndarray
    normals: np.ndarray
    material: int = 0
    group: str = 'structure'
    motion: str = 'fixed'
    center: np.ndarray = field(default_factory=lambda: np.zeros(3))
    explode: np.ndarray = field(default_factory=lambda: np.zeros(3))
    role: str = ''
    provenance: str = 'designed-concept'
    cad: Any = field(default=None, repr=False)
    tags: tuple[str, ...] = ()
    @property
    def bounds(self): return np.array([self.vertices.min(axis=0), self.vertices.max(axis=0)])
    def moved(self, offset, prefix=''):
        """Bake a translation without changing local shaft/rotation pivots."""
        offset = np.asarray(offset, float)
        return replace(self, name=prefix+self.name, vertices=self.vertices+offset,
                       center=self.center+offset,
                       cad=self.cad.translate(tuple(offset)) if self.cad is not None else None)
    def metadata(self):
        return dict(name=self.name, material=self.material, group=self.group,
                    motion=self.motion, center=np.asarray(self.center).tolist(),
                    explode=np.asarray(self.explode).tolist(), role=self.role,
                    provenance=self.provenance, tags=list(self.tags),
                    vertices=len(self.vertices), triangles=len(self.faces),
                    has_analytic_cad=self.cad is not None, bounds_mm=self.bounds.tolist())

@dataclass
class View:
    az: float = 45.0
    el: float = 25.0
    scale: float = 65.0  # orthographic half-height in mm
    target: tuple[float,float,float] = (23,0,0)
    explode: float = 0.0
    hide: tuple[str,...] = ()  # exact group names
    section: tuple[float,float,float] | None = None  # clipping plane normal
    title: str = ''
    note: str = ''

@dataclass
class Assembly:
    name: str
    parts: list[Part]
    materials: list[Material]
    views: dict[str,View] = field(default_factory=dict)
    metadata: dict = field(default_factory=dict)
    # Signature: (part, time_seconds, explosion) -> homogeneous 4x4 mm transform
    motion_function: Callable | None = field(default=None, repr=False)
    def pose(self, p, t=0.0, explode=0.0):
        if self.motion_function is not None: return self.motion_function(p,t,explode)
        T=np.eye(4);T[:3,3]=np.asarray(p.explode)*explode;return T
    @property
    def bounds(self):
        if not self.parts:
            raise ValueError('Assembly has no parts')
        extrema=[]
        for p in self.parts:
            T=self.pose(p,0,0)
            vertices=p.vertices@T[:3,:3].T+T[:3,3]
            extrema.append((vertices.min(axis=0),vertices.max(axis=0)))
        return np.array([np.min([b[0] for b in extrema],axis=0),
                         np.max([b[1] for b in extrema],axis=0)])
    def select(self, groups=None, names=None):
        parts=[p for p in self.parts if (groups is None or p.group in groups) and (names is None or p.name in names)]
        if not parts: raise ValueError('Selection contains no geometry')
        return replace(self,parts=parts)

def project_root() -> Path:
    configured=os.environ.get('MECHANISM_LAB_ROOT')
    if configured:return Path(configured).expanduser().resolve()
    candidate=Path(__file__).resolve().parents[2]
    if (candidate/'assets').exists():return candidate
    if (Path.cwd()/'assets').exists():return Path.cwd().resolve()
    raise RuntimeError('Set MECHANISM_LAB_ROOT to the extracted project directory containing assets/')

def rotation_x(a):
    c,s=math.cos(a),math.sin(a)
    return np.array([[1,0,0],[0,c,-s],[0,s,c]],float)

def axis_pose(angle, center=(0,0,0), explode=(0,0,0), amount=0):
    R=rotation_x(angle);pivot=np.asarray(center,float)
    T=np.eye(4);T[:3,:3]=R;T[:3,3]=pivot-R@pivot+np.asarray(explode)*amount
    return T

def smooth_normals(vertices,faces,angle=40):
    """Split only real hard edges; preserves analytic smooth tessellation elsewhere."""
    import vtk
    from vtk.util.numpy_support import numpy_to_vtk,numpy_to_vtkIdTypeArray,vtk_to_numpy
    pd=vtk.vtkPolyData();pts=vtk.vtkPoints();pts.SetData(numpy_to_vtk(np.asarray(vertices,float),deep=True));pd.SetPoints(pts)
    cells=vtk.vtkCellArray();cells.SetData(numpy_to_vtkIdTypeArray(np.arange(len(faces)+1,dtype=np.int64)*3,deep=True),numpy_to_vtkIdTypeArray(np.asarray(faces,dtype=np.int64).ravel(),deep=True));pd.SetPolys(cells)
    cl=vtk.vtkCleanPolyData();cl.SetInputData(pd);cl.SetTolerance(1e-8)
    no=vtk.vtkPolyDataNormals();no.SetInputConnection(cl.GetOutputPort());no.SetFeatureAngle(angle);no.SplittingOn();no.ConsistencyOn();no.AutoOrientNormalsOn();no.Update();q=no.GetOutput()
    return (vtk_to_numpy(q.GetPoints().GetData()).copy(),
            vtk_to_numpy(q.GetPolys().GetConnectivityArray()).reshape(-1,3).copy(),
            vtk_to_numpy(q.GetPointData().GetNormals()).copy())

def mesh_part(name, vertices, faces, material=0, **kw):
    v,f,n=smooth_normals(vertices,faces)
    return Part(name,v,f,n,material,**kw)

def cad_part(name, shape, material=0, tolerance=.022, angular=.045, **kw):
    import cadquery as cq
    if isinstance(shape,cq.Workplane):shape=shape.val()
    if not shape.isValid():raise ValueError(f'{name}: OpenCascade returned invalid CAD')
    v,f=shape.tessellate(tolerance,angular)
    return mesh_part(name,[[p.x,p.y,p.z] for p in v],f,material,cad=shape,**kw)

def validate(assembly: Assembly, expensive=False):
    names=[p.name for p in assembly.parts]
    if len(set(names))!=len(names):raise ValueError('Duplicate part names')
    import trimesh
    results=[]
    for p in assembly.parts:
        if p.vertices.ndim!=2 or p.vertices.shape[1]!=3 or p.faces.ndim!=2 or p.faces.shape[1]!=3:
            raise ValueError(f'Geometry arrays must be Nx3: {p.name}')
        if p.normals.shape!=p.vertices.shape:
            raise ValueError(f'Vertex/normal shape mismatch: {p.name}')
        if not np.issubdtype(p.faces.dtype,np.integer):
            raise ValueError(f'Face indices must be integers: {p.name}')
        if not len(p.faces) or not np.isfinite(p.vertices).all():raise ValueError(f'Empty/nonfinite geometry: {p.name}')
        if not np.isfinite(p.normals).all():raise ValueError(f'Nonfinite normals: {p.name}')
        if p.faces.min()<0 or p.faces.max()>=len(p.vertices):raise ValueError(f'Invalid face indices: {p.name}')
        if not 0<=p.material<len(assembly.materials):raise ValueError(f'Invalid material: {p.name}')
        T=assembly.pose(p,0,0)
        if T.shape!=(4,4) or not np.isfinite(T).all() or not np.allclose(T[3],[0,0,0,1]):
            raise ValueError(f'Invalid initial transform: {p.name}')
        if not np.allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-7) or np.linalg.det(T[:3,:3])<0:
            raise ValueError(f'Pose must be a proper rigid transform: {p.name}')
        row=dict(name=p.name,finite=True,valid_indices=True,analytic_valid=p.cad.isValid() if p.cad else None)
        if expensive:
            mesh=trimesh.Trimesh(p.vertices,p.faces,process=True)
            row.update(watertight=bool(mesh.is_watertight),consistent_winding=bool(mesh.is_winding_consistent))
        results.append(row)
    return dict(model=assembly.name,parts=len(names),triangles=sum(len(p.faces) for p in assembly.parts),units='mm',bounds_mm=assembly.bounds.tolist(),checks=results,
                disclaimer='Geometry and transform checks only; not load, contact, fit or safety validation.')

def save_cache(assembly, directory):
    directory=Path(directory);directory.mkdir(parents=True,exist_ok=True)
    arrays={f'p{i}_{k}':getattr(p,k) for i,p in enumerate(assembly.parts) for k in ['vertices','faces','normals']}
    np.savez_compressed(directory/'meshes.npz',**arrays)
    info=dict(schema=1,name=assembly.name,units='mm',axis='X-shaft/Z-up',parts=[p.metadata() for p in assembly.parts],
              materials=[m.as_dict() for m in assembly.materials],metadata=assembly.metadata,views={k:asdict(v) for k,v in assembly.views.items()})
    (directory/'manifest.json').write_text(json.dumps(info,indent=2)+'\n')

def load_cache(directory):
    directory=Path(directory);info=json.loads((directory/'manifest.json').read_text());a=np.load(directory/'meshes.npz')
    parts=[]
    for i,d in enumerate(info['parts']):
        kw={k:d[k] for k in ['name','material','group','motion','center','explode','role','provenance','tags']}
        kw.update({k:a[f'p{i}_{k}'].copy() for k in ['vertices','faces','normals']});parts.append(Part(**kw))
    a.close()
    return Assembly(info['name'],parts,[Material(**m) for m in info['materials']],views={k:View(**v) for k,v in info.get('views',{}).items()},metadata=info['metadata'])

def pose_cad(shape,T):
    """Apply a rigid 4x4 pose through gp_Trsf, preserving the shape's existing location."""
    import cadquery as cq
    from OCP.gp import gp_Trsf
    T=np.asarray(T,float)
    if not np.allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-8) or np.linalg.det(T[:3,:3])<0:
        raise ValueError('CAD pose must be a proper rigid transform')
    tr=gp_Trsf();tr.SetValues(*[float(v) for v in T[:3,:4].ravel()])
    return shape.moved(cq.Location(tr))
