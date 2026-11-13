"""Geometry values and explicit attribute domains. No object catalogue."""
from dataclasses import dataclass, field as member
from pathlib import Path
from types import MappingProxyType
import json
from functools import lru_cache
import numpy as np

def mesh_edges(faces):
    """Canonical edge attribute order: lexicographically sorted undirected pairs.

    Corner attributes use face-major flattened order (3*face + local_corner).
    """
    f = np.asarray(faces)
    return np.unique(np.sort(np.concatenate((f[:,[0,1]], f[:,[1,2]], f[:,[2,0]])),axis=1),axis=0)

@dataclass(frozen=True)
class Attribute:
    values: np.ndarray
    domain: str = 'vertex'
    interpolation: str = 'linear'

@dataclass(frozen=True)
class Mesh:
    vertices: np.ndarray
    faces: np.ndarray
    attributes: dict = member(default_factory=dict)
    closed: bool = True
    connected: bool = True

    def __post_init__(self):
        v = np.array(self.vertices, dtype=float, copy=True)
        raw = np.asarray(self.faces)
        if raw.ndim != 2 or raw.shape[1] != 3 or not np.issubdtype(raw.dtype, np.integer):
            raise ValueError('Faces must be integer triangles')
        f = np.array(raw, dtype=np.int64, copy=True)
        if v.ndim != 2 or v.shape[1] != 3 or len(v) < 3 or not np.isfinite(v).all():
            raise ValueError('Vertices must be finite Nx3 coordinates')
        if not len(f) or f.min() < 0 or f.max() >= len(v):
            raise ValueError('Faces must reference existing vertices')
        attributes = {}
        for name, attribute in self.attributes.items():
            if attribute.domain not in ('vertex','face','edge','corner') or attribute.interpolation not in ('linear','nearest'):
                raise ValueError('Attribute domain/interpolation must be explicit and supported')
            values = np.array(attribute.values, copy=True)
            count = {'vertex': len(v), 'face': len(f), 'corner': 3*len(f),
                     'edge': len(mesh_edges(f))}[attribute.domain]
            if not values.ndim or len(values) != count or not np.issubdtype(values.dtype,np.number) or not np.isfinite(values).all():
                raise ValueError(f'Invalid attribute: {name}')
            values.flags.writeable = False
            attributes[name] = Attribute(values,attribute.domain,attribute.interpolation)
        v.flags.writeable = False; f.flags.writeable = False
        object.__setattr__(self,'vertices',v); object.__setattr__(self,'faces',f)
        object.__setattr__(self,'attributes',MappingProxyType(attributes))
        report = self.report()
        if report['degenerate_faces'] or report['nonmanifold_edges'] or report['nonmanifold_vertices'] or report['winding_errors']:
            raise ValueError(f'Invalid surface topology: {report}')
        if self.closed and report['boundary_edges']:
            raise ValueError('Expected a closed surface; boundaries remain')
        if self.connected and report['components'] != 1:
            raise ValueError(f"Expected one connected surface, got {report['components']}")

    def report(self):
        v,f=self.vertices,self.faces
        topology=_topology(len(v),f.shape,f.tobytes())
        areas=np.linalg.norm(np.cross(v[f[:,1]]-v[f[:,0]],v[f[:,2]]-v[f[:,0]]),axis=1)
        return dict(vertices=len(v),faces=len(f),**topology,
                    degenerate_faces=int(np.sum(areas==0)),self_intersections_checked=False)


@lru_cache(maxsize=8)
def _topology(vertex_count, face_shape, face_bytes):
    """Exact connectivity cache. Geometry validity is never inferred from this."""
    f=np.frombuffer(face_bytes,dtype=np.int64).reshape(face_shape)
    directed = np.concatenate((f[:,[0,1]],f[:,[1,2]],f[:,[2,0]]))
    edges,inverse,counts = np.unique(np.sort(directed,axis=1),axis=0,return_inverse=True,return_counts=True)
    signs = np.where(directed[:,0] < directed[:,1],1,-1)
    orientation = np.bincount(inverse,weights=signs)
    parents = np.arange(vertex_count)
    def root(a):
        while parents[a] != a:
            parents[a] = parents[parents[a]]; a = parents[a]
        return a
    for a,b in edges:
        ra,rb = root(a),root(b)
        if ra != rb: parents[rb] = ra
    # A vertex shared by separate fans is not a coherent surface connection.
    links=[[] for _ in range(vertex_count)]
    for a,b,c in f:
        links[a].append((b,c));links[b].append((c,a));links[c].append((a,b))
    bad_vertices=0
    for link in links:
        adjacency={}
        for a,b in link:
            adjacency.setdefault(a,set()).add(b);adjacency.setdefault(b,set()).add(a)
        if not adjacency:
            bad_vertices+=1;continue
        seen=set();pending=[next(iter(adjacency))]
        while pending:
            node=pending.pop()
            if node in seen: continue
            seen.add(node);pending.extend(adjacency[node]-seen)
        degrees=[len(n) for n in adjacency.values()]
        if len(seen)!=len(adjacency) or any(d not in (1,2) for d in degrees) or degrees.count(1) not in (0,2):
            bad_vertices+=1
    return dict(components=len({root(i) for i in range(vertex_count)}),
                boundary_edges=int(np.sum(counts==1)),nonmanifold_edges=int(np.sum(counts>2)),
                winding_errors=int(np.sum((counts==2)&(orientation!=0))),nonmanifold_vertices=bad_vertices)


def save_snapshot(mesh, path):
    """Real evaluated geometry and attributes, consumable by independent renderers."""
    path = Path(path); path.parent.mkdir(parents=True,exist_ok=True)
    arrays = dict(vertices=mesh.vertices,faces=mesh.faces)
    metadata = dict(closed=mesh.closed,connected=mesh.connected,attributes={},report=mesh.report())
    for index,(name,attr) in enumerate(mesh.attributes.items()):
        key=f'attribute_{index}'; arrays[key]=attr.values
        metadata['attributes'][name]=dict(array=key,domain=attr.domain,interpolation=attr.interpolation)
    arrays['metadata']=np.array(json.dumps(metadata))
    np.savez_compressed(path,**arrays)
    return path


def load_snapshot(path):
    with np.load(path,allow_pickle=False) as data:
        meta=json.loads(str(data['metadata']))
        attrs={name:Attribute(data[a['array']],a['domain'],a['interpolation']) for name,a in meta['attributes'].items()}
        return Mesh(data['vertices'],data['faces'],attrs,meta['closed'],meta['connected'])
