"""Small explicit topology editor with honest feature identity and lineage.

Edits return new values. A reference resolves exactly only while its feature
survives unchanged; descendants() is an explicit one-to-many provenance query,
not a promise that a feature retains its geometric or semantic meaning.
Topology checks do not prove absence of intersections or physical soundness.
"""
from dataclasses import dataclass
from uuid import uuid4
import numpy as np
from .model import Mesh, Attribute, mesh_edges

@dataclass(frozen=True)
class FeatureRef:
    owner: str
    domain: str
    identity: str

class Editable:
    def __init__(self, mesh, *, _owner=None, _ids=None, _ancestors=None):
        self.mesh = mesh
        self.owner = _owner or uuid4().hex
        counts = dict(vertex=len(mesh.vertices), face=len(mesh.faces),
                      edge=len(mesh_edges(mesh.faces)), corner=3*len(mesh.faces))
        self._ids = _ids or {d: tuple(uuid4().hex for _ in range(n)) for d,n in counts.items()}
        self._ancestors = _ancestors or {d: tuple(frozenset((i,)) for i in ids) for d,ids in self._ids.items()}

    def ref(self, domain, index):
        return FeatureRef(self.owner, domain, self._ids[domain][index])

    def resolve(self, ref):
        """Return exact current index; raise if foreign, removed, or replaced."""
        if ref.owner != self.owner: raise ValueError('Reference belongs to a different editable mesh')
        try: return self._ids[ref.domain].index(ref.identity)
        except ValueError: raise KeyError('Feature was removed or replaced; ask descendants explicitly') from None

    def descendants(self, ref):
        if ref.owner != self.owner: raise ValueError('Reference belongs to a different editable mesh')
        return tuple(i for i,a in enumerate(self._ancestors[ref.domain]) if ref.identity in a)

    def boundaries(self):
        """Oriented boundary loops following the existing incident face winding."""
        f=self.mesh.faces; counts={}
        for tri in f:
            for a,b in zip(tri,np.roll(tri,-1)):
                key=tuple(sorted((int(a),int(b))))
                counts.setdefault(key,[]).append((int(a),int(b)))
        nxt={a:b for es in counts.values() if len(es)==1 for a,b in es}
        loops=[]
        while nxt:
            start=next(iter(nxt)); loop=[start]; node=nxt.pop(start)
            while node!=start:
                loop.append(node);node=nxt.pop(node)
            loops.append(tuple(loop))
        return tuple(loops)

    def _finish(self, vertices, faces, weights, parents, unchanged, *, edge_defaults=None, closed=None):
        """weights maps each new vertex to source vertices; parents maps faces."""
        old=self.mesh; faces=np.asarray(faces,dtype=np.int64); vertices=np.asarray(vertices,float)
        # Remove vertices made unused by patch replacement / welding.
        used=np.unique(faces); remap=np.full(len(vertices),-1,dtype=int);remap[used]=np.arange(len(used))
        faces=remap[faces];vertices=vertices[used];weights=[weights[i] for i in used]
        def blend(values,w,mode):
            if mode=='nearest':return values[max(w,key=w.get)]
            return sum(values[i]*t for i,t in w.items())
        ids={};anc={}
        ids['vertex']=[];anc['vertex']=[]
        for w in weights:
            exact=next(iter(w)) if len(w)==1 and next(iter(w.values()))==1 else None
            # Duplicated source vertices may inherit values, never identities.
            old_id=self._ids['vertex'][exact] if exact is not None else None
            same=exact is not None and np.array_equal(vertices[len(ids['vertex'])],old.vertices[exact]) and old_id not in ids['vertex']
            ident=old_id if same else uuid4().hex
            ids['vertex'].append(ident)
            anc['vertex'].append(frozenset({ident}).union(*(self._ancestors['vertex'][i] for i in w)))
        ids['face']=[];anc['face']=[]
        for j,p in enumerate(parents):
            ident=self._ids['face'][p] if unchanged[j] else uuid4().hex
            ids['face'].append(ident)
            anc['face'].append(frozenset((ident,)) | (self._ancestors['face'][p] if p is not None else frozenset()))
        old_edges=mesh_edges(old.faces);edges=mesh_edges(faces)
        edge_lookup={tuple(e):k for k,e in enumerate(old_edges)}
        old_by_ids={tuple(sorted(self._ids['vertex'][i] for i in e)):k for k,e in enumerate(old_edges)}
        edge_sources=[];ids['edge']=[];anc['edge']=[]
        for e in edges:
            key=tuple(sorted(ids['vertex'][i] for i in e)); exact=old_by_ids.get(key)
            support=set(weights[e[0]])|set(weights[e[1]])
            p=edge_lookup.get(tuple(sorted(support))) if exact is None else exact
            edge_sources.append(p); ident=self._ids['edge'][exact] if exact is not None else uuid4().hex
            ids['edge'].append(ident);anc['edge'].append(frozenset((ident,)) | (self._ancestors['edge'][p] if p is not None else frozenset()))
        ids['corner']=[];anc['corner']=[];corner_sources=[]
        for j,tri in enumerate(faces):
            p=parents[j]
            for vi in tri:
                source={} if p is None else {3*p+k:weights[vi].get(int(v),0) for k,v in enumerate(old.faces[p]) if weights[vi].get(int(v),0)}
                if source:
                    total=sum(source.values());source={i:t/total for i,t in source.items()}
                corner_sources.append(source)
                exact=next(iter(source)) if unchanged[j] and len(source)==1 else None
                ident=self._ids['corner'][exact] if exact is not None else uuid4().hex
                ids['corner'].append(ident);anc['corner'].append(frozenset((ident,)).union(*(self._ancestors['corner'][i] for i in source)))
        attrs={}
        for name,a in old.attributes.items():
            if a.domain=='vertex': vals=[blend(a.values,w,a.interpolation) for w in weights]
            elif a.domain=='face':
                if any(p is None for p in parents):raise ValueError('Bridge requires explicit attributes; strip or assign new face attributes first')
                vals=[a.values[p] for p in parents]
            elif a.domain=='corner':
                if any(not w for w in corner_sources):raise ValueError('New corner has no source; supply attributes after this edit')
                vals=[blend(a.values,w,a.interpolation) for w in corner_sources]
            else:
                missing=any(p is None for p in edge_sources)
                if missing and (edge_defaults is None or name not in edge_defaults):raise ValueError(f'New edges need explicit edge_defaults[{name!r}]')
                vals=[a.values[p] if p is not None else edge_defaults[name] for p in edge_sources]
            attrs[name]=Attribute(np.asarray(vals),a.domain,a.interpolation)
        mesh=Mesh(vertices,faces,attrs,old.closed if closed is None else closed,old.connected)
        return Editable(mesh,_owner=self.owner,_ids={d:tuple(x) for d,x in ids.items()},_ancestors={d:tuple(x) for d,x in anc.items()})

    def split_edge(self,a,b,t=.5, *, edge_defaults=None):
        if not 0<t<1:raise ValueError('Split t must be inside (0,1)')
        m=self.mesh; a=int(a);b=int(b)
        if tuple(sorted((a,b))) not in set(map(tuple,mesh_edges(m.faces))):raise ValueError('Not an edge')
        n=len(m.vertices);v=[*m.vertices,(1-t)*m.vertices[a]+t*m.vertices[b]]
        w=[{i:1.} for i in range(n)]+[{a:1-t,b:t}];f=[];p=[];u=[]
        for j,tri in enumerate(m.faces):
            if a in tri and b in tri:
                for k in range(3):
                    x,y,z=np.roll(tri,-k)
                    if {x,y}=={a,b}:
                        f.extend(((x,n,z),(n,y,z)));p.extend((j,j));u.extend((False,False));break
            else:f.append(tri);p.append(j);u.append(True)
        return self._finish(v,f,w,p,u,edge_defaults=edge_defaults)

    def extrude_faces(self, indices, offset, *, edge_defaults=None):
        """Extrude a connected or disjoint patch by a common finite XYZ offset.

        Interior patch vertices move, boundary vertices are duplicated, and side
        walls close the patch. No collision or offset-surface guarantee.
        """
        return self._patch(indices,offset=offset,edge_defaults=edge_defaults)

    def inset_faces(self,indices,fraction, *, edge_defaults=None):
        """Scale selected patch vertices toward its centroid and add a rim.

        This is a centroid inset, not a constant-distance polygon offset; concave
        patches may overlap. Use planar convex patches for a predictable result.
        """
        if not 0<fraction<1:raise ValueError('Inset fraction must lie in (0,1)')
        return self._patch(indices,fraction=fraction,edge_defaults=edge_defaults)

    def _patch(self,indices,offset=None,fraction=None,edge_defaults=None):
        m=self.mesh;chosen=set(map(int,indices))
        if not chosen or min(chosen)<0 or max(chosen)>=len(m.faces):raise ValueError('Select existing faces')
        sel=m.faces[sorted(chosen)];vid=np.unique(sel);n=len(m.vertices)
        if offset is not None:
            delta=np.asarray(offset,float)
            if delta.shape!=(3,) or not np.isfinite(delta).all() or not np.any(delta):raise ValueError('Offset must be finite nonzero XYZ')
            new=m.vertices[vid]+delta
        else:new=(1-fraction)*m.vertices[vid]+fraction*m.vertices[vid].mean(axis=0)
        v=[*m.vertices,*new];mapping={int(old):n+i for i,old in enumerate(vid)}
        w=[{i:1.} for i in range(n)]+[{int(i):1.} for i in vid]
        f=[];p=[];u=[];edges={}
        for j,tri in enumerate(m.faces):
            f.append([mapping[int(i)] for i in tri] if j in chosen else tri);p.append(j);u.append(j not in chosen)
            if j in chosen:
                for a,b in zip(tri,np.roll(tri,-1)):edges.setdefault(tuple(sorted((int(a),int(b)))),[]).append((int(a),int(b),j))
        boundary=[es[0] for es in edges.values() if len(es)==1]
        if not boundary:raise ValueError('Selected patch has no boundary')
        for a,b,j in boundary:
            f.extend(((a,b,mapping[b]),(a,mapping[b],mapping[a])));p.extend((j,j));u.extend((False,False))
        return self._finish(v,f,w,p,u,edge_defaults=edge_defaults)

    def weld(self,pairs, *, tolerance=1e-8, edge_defaults=None):
        """Merge explicitly paired coincident vertices; never automatic proximity weld.

        Each pair keeps the first vertex position and averaged attributes.
        Invalid/nonmanifold outcomes are rejected by Mesh validation.
        """
        m=self.mesh;rep=np.arange(len(m.vertices));groups={i:{i} for i in rep}
        if tolerance<0 or not np.isfinite(tolerance):raise ValueError('Invalid tolerance')
        for a,b in pairs:
            if not 0<=a<len(rep) or not 0<=b<len(rep):raise ValueError('Unknown vertex')
            a,b=int(rep[a]),int(rep[b])
            if a==b:continue
            if np.linalg.norm(m.vertices[a]-m.vertices[b])>tolerance:raise ValueError('Weld pair exceeds tolerance')
            groups[a]|=groups.pop(b);rep[rep==b]=a
        faces=rep[m.faces];keep=np.array([len(set(t))==3 for t in faces])
        w=[{k:1/len(groups.get(i,{i})) for k in groups.get(i,{i})} for i in range(len(rep))]
        unchanged=[bool(np.array_equal(t,m.faces[j])) for j,t in enumerate(faces) if keep[j]]
        return self._finish(m.vertices,faces[keep],w,np.where(keep)[0],unchanged,edge_defaults=edge_defaults)

    def bridge(self,loop_a,loop_b, *, edge_defaults=None):
        """Bridge equal-size open boundary loops. Caller chooses correspondence.

        Both inputs follow existing face winding, starting at matching vertices.
        The second loop is walked in reverse to preserve surface orientation.
        Unequal-loop resampling and arbitrary attribute creation are not implied.
        """
        a=list(map(int,loop_a));b=list(map(int,loop_b));m=self.mesh
        loops=self.boundaries()
        def valid(loop):
            return any(len(loop)==len(l) and any(tuple(loop)==l[k:]+l[:k] for k in range(len(l))) for l in loops)
        if len(a)!=len(b) or len(a)<3 or not valid(a) or not valid(b) or set(a)&set(b):raise ValueError('Need distinct equal-size oriented boundary loops')
        b=[b[0],*b[:0:-1]];f=m.faces.tolist();p=list(range(len(f)));u=[True]*len(f)
        for i in range(len(a)):
            j=(i+1)%len(a);f.extend(((a[j],a[i],b[i]),(a[j],b[i],b[j])));p.extend((None,None));u.extend((False,False))
        return self._finish(m.vertices,f,[{i:1.} for i in range(len(m.vertices))],p,u,edge_defaults=edge_defaults,closed=False)
