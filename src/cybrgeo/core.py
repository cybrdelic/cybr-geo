"""Typed, millimetre-native assemblies with named parts and explicit provenance.

GLB output is transformed to metres and Y-up. STL/BREP/STEP remain millimetres.
Saved scene JSON is data, never executable code. NPZ loads disallow pickle.
"""
from __future__ import annotations
from dataclasses import dataclass, field, asdict
from pathlib import Path
from typing import Any, Iterable
import hashlib, json, re
import numpy as np
import trimesh

@dataclass(frozen=True)
class Material:
    name: str = "Machined alloy"
    color: tuple[float, float, float] = (.48, .50, .53)
    metal: float = .9
    rough: float = .28

@dataclass
class Part:
    name: str
    vertices: np.ndarray
    faces: np.ndarray
    normals: np.ndarray
    material: int = 0
    group: str = "assembly"
    explode: np.ndarray = field(default_factory=lambda: np.zeros(3))
    role: str = "Author-created geometry; not qualified manufacturing CAD"
    motion: str = "fixed"
    center: np.ndarray = field(default_factory=lambda: np.zeros(3))
    metadata: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self):
        self.vertices = np.asarray(self.vertices, dtype=np.float64)
        self.faces = np.asarray(self.faces, dtype=np.int64)
        self.normals = np.asarray(self.normals, dtype=np.float64)
        self.explode = np.asarray(self.explode, dtype=float)
        self.center = np.asarray(self.center, dtype=float)
        if not self.name or not re.fullmatch(r"[A-Za-z0-9_.-]+", self.name):
            raise ValueError(f"Unsafe part name: {self.name!r}")
        if self.vertices.ndim != 2 or self.vertices.shape[1] != 3 or not len(self.vertices):
            raise ValueError(f"{self.name}: nonempty Nx3 vertices required")
        if self.faces.ndim != 2 or self.faces.shape[1] != 3 or not len(self.faces):
            raise ValueError(f"{self.name}: nonempty Mx3 triangle faces required")
        if self.normals.shape != self.vertices.shape:
            raise ValueError(f"{self.name}: vertex normal count mismatch")
        if not np.isfinite(self.vertices).all() or not np.isfinite(self.normals).all():
            raise ValueError(f"{self.name}: nonfinite geometry")
        if self.faces.min() < 0 or self.faces.max() >= len(self.vertices):
            raise ValueError(f"{self.name}: invalid face indices")
        if self.explode.shape != (3,) or self.center.shape != (3,):
            raise ValueError("center and explode must have three coordinates")

    @property
    def bounds(self) -> np.ndarray:
        return np.array([self.vertices.min(axis=0), self.vertices.max(axis=0)])

    def transformed(self, matrix: np.ndarray) -> Part:
        from copy import deepcopy
        out = deepcopy(self)
        out.vertices = trimesh.transform_points(self.vertices, matrix)
        normal_matrix = np.linalg.inv(matrix[:3, :3]).T
        out.normals = self.normals @ normal_matrix.T
        out.normals /= np.maximum(np.linalg.norm(out.normals, axis=1, keepdims=True), 1e-15)
        out.center = trimesh.transform_points(self.center[None], matrix)[0]
        out.explode = matrix[:3, :3] @ self.explode
        return out

@dataclass
class Assembly:
    name: str
    parts: list[Part]
    materials: list[Material] = field(default_factory=lambda: [Material()])
    metadata: dict[str, Any] = field(default_factory=dict)
    # B-rep objects stay optional; never silently convert meshes into 'analytic CAD'.
    cad: dict[str, Any] = field(default_factory=dict, repr=False)

    def __post_init__(self):
        names = [p.name for p in self.parts]
        if not names or len(names) != len(set(names)):
            raise ValueError("Assembly needs nonempty, unique part names")
        for p in self.parts:
            if not 0 <= p.material < len(self.materials):
                raise ValueError(f"{p.name}: material index outside palette")

    @property
    def bounds(self):
        return np.array([np.min([p.bounds[0] for p in self.parts],axis=0),
                         np.max([p.bounds[1] for p in self.parts],axis=0)])

    def save(self, folder: str | Path) -> Path:
        root = Path(folder); root.mkdir(parents=True, exist_ok=True)
        payload = {}; entries = []
        for i, p in enumerate(self.parts):
            key = f"p{i:05d}"
            for attr in ("vertices", "faces", "normals"):
                payload[f"{key}_{attr}"] = getattr(p, attr)
            entries.append(dict(key=key, name=p.name, material=p.material, group=p.group,
                                explode=p.explode.tolist(), center=p.center.tolist(),
                                role=p.role, motion=p.motion, metadata=p.metadata))
        np.savez_compressed(root/'meshes.npz', **payload)
        meta = dict(schema="cybrgeo.scene/1", name=self.name, units="mm", up="Z",
                    materials=[asdict(m) for m in self.materials], parts=entries,
                    metadata=self.metadata,
                    meshes_sha256=hashlib.sha256((root/'meshes.npz').read_bytes()).hexdigest())
        (root/'scene.json').write_text(json.dumps(meta, indent=2)+'\n')
        if self.cad:
            import cadquery as cq
            d = root/'brep'; d.mkdir(exist_ok=True)
            a = cq.Assembly(name=self.name)
            for n, s in self.cad.items():
                s.exportBrep(str(d/f'{n}.brep'))
                a.add(s, name=n)
            a.export(str(root/'assembly.step'))
        return root/'scene.json'

    @classmethod
    def load(cls, path: str | Path, with_cad: bool = False) -> Assembly:
        path = Path(path); path = path/'scene.json' if path.is_dir() else path
        info = json.loads(path.read_text()); root = path.parent
        if info.get('schema') != 'cybrgeo.scene/1' or info.get('units') != 'mm':
            raise ValueError("Unsupported scene schema or units")
        if hashlib.sha256((root/'meshes.npz').read_bytes()).hexdigest() != info['meshes_sha256']:
            raise ValueError("Mesh archive SHA256 mismatch")
        parts=[]
        with np.load(root/'meshes.npz', allow_pickle=False) as arr:
            for entry in info['parts']:
                e=dict(entry); key=e.pop('key')
                parts.append(Part(**e, **{k:arr[f'{key}_{k}'].copy() for k in ('vertices','faces','normals')}))
        result=cls(info['name'],parts,[Material(**m) for m in info['materials']],info['metadata'])
        if with_cad:
            import cadquery as cq
            for p in parts:
                f=root/'brep'/f'{p.name}.brep'
                if f.exists(): result.cad[p.name]=cq.Shape.importBrep(str(f))
        return result

    def export_glb(self, path: str | Path, poses: dict[str,np.ndarray] | None = None):
        sc=trimesh.Scene()
        for p in self.parts:
            m=self.materials[p.material]
            rgba=[*np.clip(np.power(m.color,1/2.2)*255,0,255).astype(np.uint8),255]
            material=trimesh.visual.material.PBRMaterial(name=m.name,baseColorFactor=rgba,
                         metallicFactor=m.metal,roughnessFactor=m.rough)
            mesh=trimesh.Trimesh(p.vertices,p.faces,vertex_normals=p.normals,process=False)
            mesh.visual=trimesh.visual.TextureVisuals(material=material)
            sc.add_geometry(mesh,geom_name=p.name,node_name=p.name,
                            transform=np.eye(4) if poses is None else poses.get(p.name,np.eye(4)))
        # mm Z-up -> metres Y-up, right-handed rotation, no mirror.
        sc.apply_transform(np.array([[.001,0,0,0],[0,0,.001,0],[0,-.001,0,0],[0,0,0,1]]))
        sc.metadata.update(units="m", source_units="mm", provenance=self.metadata)
        Path(path).parent.mkdir(parents=True,exist_ok=True); sc.export(str(path))

    def export_parts(self, folder: str | Path):
        folder=Path(folder);folder.mkdir(parents=True,exist_ok=True)
        for p in self.parts:
            mesh=trimesh.Trimesh(p.vertices,p.faces,process=False)
            mesh.export(folder/f'{p.name}.stl')
            Assembly(p.name,[p],self.materials).export_glb(folder/f'{p.name}.glb')
        (folder/'UNITS.txt').write_text('STL: millimetres, Z-up. GLB: metres, Y-up.\n')

    def validate(self) -> dict[str,Any]:
        return dict(schema="cybrgeo.validation/1", name=self.name,part_count=len(self.parts),
                    triangles=sum(len(p.faces) for p in self.parts),
                    bounds_mm=self.bounds.tolist(),analytic_brep_parts=len(self.cad),
                    unique_names=True,finite_vertices=True,valid_indices=True,
                    note="Structural data checks only. No load, tolerance, interference or thermal certification.")

def translation(v) -> np.ndarray:
    result=np.eye(4);result[:3,3]=v;return result

def rotation(angle: float, axis=(1,0,0), center=(0,0,0)) -> np.ndarray:
    return trimesh.transformations.rotation_matrix(angle,axis,point=center)

def from_shape(name: str, shape, material: int=0, tolerance: float=.08,
               angular_tolerance: float=.16, **kwargs) -> Part:
    """Tessellate CAD and split normals at actual creases, retaining analytic input separately."""
    vs,fs=shape.tessellate(tolerance,angular_tolerance)
    v=np.array([p.toTuple() for p in vs]);f=np.array(fs)
    if not len(f):raise ValueError(f"{name}: CAD has no tessellatable faces")
    import vtk
    from vtk.util.numpy_support import numpy_to_vtk,numpy_to_vtkIdTypeArray,vtk_to_numpy
    pts=vtk.vtkPoints();pts.SetData(numpy_to_vtk(v,deep=True))
    cells=vtk.vtkCellArray();cells.SetCells(len(f),numpy_to_vtkIdTypeArray(np.c_[np.full(len(f),3),f].astype(np.int64).ravel(),deep=True))
    data=vtk.vtkPolyData();data.SetPoints(pts);data.SetPolys(cells)
    clean=vtk.vtkCleanPolyData();clean.SetInputData(data);clean.SetTolerance(1e-8)
    normals=vtk.vtkPolyDataNormals();normals.SetInputConnection(clean.GetOutputPort());normals.SetFeatureAngle(38);normals.ConsistencyOn();normals.SplittingOn();normals.Update();d=normals.GetOutput()
    return Part(name,vtk_to_numpy(d.GetPoints().GetData()).copy(),
                vtk_to_numpy(d.GetPolys().GetData()).reshape(-1,4)[:,1:].copy(),
                vtk_to_numpy(d.GetPointData().GetNormals()).copy(),material=material,**kwargs)

def safe_name(text: str) -> str:
    return re.sub(r'[^A-Za-z0-9_.-]+','_',text).strip('_') or 'part'
