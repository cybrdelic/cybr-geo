"""Persistent tiled heightfields. Local edits do not materialize a whole Mesh.

A tile owns cells, including a duplicate boundary sample ring. Inclusive sample
edits update every owner of a shared sample. A persistent binary tree shares all
untouched leaves and branches. Cached z bounds are rebuilt only along edit paths.
This specialized storage complements Mesh; it does not accelerate arbitrary Mesh.
"""
from dataclasses import dataclass
import numpy as np
from .model import Mesh


def _readonly(a):
    a = np.array(a, dtype=float, copy=True)
    if not np.isfinite(a).all():
        raise ValueError('heights must be finite')
    a.flags.writeable = False
    return a


@dataclass(frozen=True)
class _Node:
    lo: int
    hi: int
    low: float
    high: float
    data: object = None
    left: object = None
    right: object = None


def _branch(a, b):
    return _Node(a.lo, b.hi, min(a.low,b.low), max(a.high,b.high), left=a, right=b)


def _build(arrays, lo, hi):
    if hi-lo == 1:
        a = _readonly(arrays[lo])
        return _Node(lo,hi,float(a.min()),float(a.max()),data=a)
    mid=(lo+hi)//2
    return _branch(_build(arrays,lo,mid),_build(arrays,mid,hi))


def _get(n, i):
    while n.data is None:
        n=n.left if i<n.left.hi else n.right
    return n.data


def _replace(n, changes, counts):
    # Sorted dirty keys partition once per level; no scan of all leaves.
    if not changes:
        return n
    counts[0] += 1
    if n.data is not None:
        a=changes[0][1]
        return _Node(n.lo,n.hi,float(a.min()),float(a.max()),data=a)
    split=0
    while split<len(changes) and changes[split][0]<n.left.hi:
        split+=1
    return _branch(_replace(n.left,changes[:split],counts),
                   _replace(n.right,changes[split:],counts))


@dataclass(frozen=True)
class EditCost:
    tiles: int
    nodes: int
    copied_height_bytes: int
    evaluated_samples: int


@dataclass(frozen=True)
class TiledHeightfield:
    """Square cells on an xy grid, z-up. Units are caller-defined world units."""
    nx: int
    ny: int
    cells: int
    spacing: float
    origin: tuple
    _root: _Node

    @classmethod
    def sample(cls, fn, tiles=(8,8), cells=32, spacing=1., origin=(0.,0.)):
        nx,ny=tiles
        if any(not isinstance(v,int) or v<1 for v in (nx,ny,cells)):
            raise ValueError('tile counts and cells must be positive integers')
        if not np.isfinite(spacing) or spacing<=0 or len(origin)!=2 or not np.isfinite(origin).all():
            raise ValueError('finite origin and positive spacing required')
        arrays=[]
        for ty in range(ny):
            for tx in range(nx):
                x,y=np.meshgrid(origin[0]+(tx*cells+np.arange(cells+1))*spacing,
                                origin[1]+(ty*cells+np.arange(cells+1))*spacing)
                a=np.asarray(fn(x,y),dtype=float)
                if a.shape!=x.shape:
                    raise ValueError('evaluator must return one height per sample')
                arrays.append(a)
        return cls(nx,ny,cells,float(spacing),tuple(origin),_build(arrays,0,nx*ny))

    def tile(self, x, y):
        if not 0<=x<self.nx or not 0<=y<self.ny:
            raise IndexError('tile out of range')
        # View cannot be made writable through ordinary setflags on this view.
        return _get(self._root,y*self.nx+x).view()

    @property
    def bounds(self):
        return ((self.origin[0],self.origin[1],self._root.low),
                (self.origin[0]+self.nx*self.cells*self.spacing,
                 self.origin[1]+self.ny*self.cells*self.spacing,self._root.high))

    def edit(self, samples, fn):
        """Replace inclusive integer sample rectangle (x0,y0,x1,y1).

        fn(x,y,old_z) is evaluated once on unique edited samples, then scattered
        to owning tiles. Pure callback recommended. Failure preserves snapshot.
        Cost O(edited samples + dirty tiles*cells² + dirty tiles*log(tile count)).
        """
        x0,y0,x1,y1=samples
        if any(not isinstance(v,int) for v in samples) or not (0<=x0<=x1<=self.nx*self.cells and 0<=y0<=y1<=self.ny*self.cells):
            raise ValueError('invalid inclusive sample rectangle')
        c=self.cells
        xs=np.arange(x0,x1+1); ys=np.arange(y0,y1+1)
        old=np.empty((len(ys),len(xs)))
        # Fill unique input from canonical owner, using tile-sized slices.
        txs=range(max(0,(x0-1)//c), min(self.nx-1,x1//c)+1)
        tys=range(max(0,(y0-1)//c), min(self.ny-1,y1//c)+1)
        owners=[]
        for ty in tys:
            for tx in txs:
                a,b=max(x0,tx*c),max(y0,ty*c)
                d,e=min(x1,(tx+1)*c),min(y1,(ty+1)*c)
                dst=(slice(b-y0,e-y0+1),slice(a-x0,d-x0+1))
                src=(slice(b-ty*c,e-ty*c+1),slice(a-tx*c,d-tx*c+1))
                old[dst]=self.tile(tx,ty)[src]
                owners.append((tx,ty,dst,src))
        x,y=np.meshgrid(self.origin[0]+xs*self.spacing,self.origin[1]+ys*self.spacing)
        values=_readonly(fn(x,y,old))
        if values.shape!=old.shape:
            raise ValueError('edit must return one value per sample')
        changes=[]
        for tx,ty,dst,src in owners:
            a=np.array(self.tile(tx,ty),copy=True)
            a[src]=values[dst];a.flags.writeable=False
            changes.append((ty*self.nx+tx,a))
        count=[0]
        root=_replace(self._root,changes,count)
        result=TiledHeightfield(self.nx,self.ny,c,self.spacing,self.origin,root)
        return result,EditCost(len(changes),count[0],sum(a.nbytes for _,a in changes),old.size)

    def heights(self):
        """Explicit O(total samples) materialization, for export/full algorithms."""
        c=self.cells; out=np.empty((self.ny*c+1,self.nx*c+1))
        for ty in range(self.ny):
            for tx in range(self.nx):
                out[ty*c:(ty+1)*c+1,tx*c:(tx+1)*c+1]=self.tile(tx,ty)
        return out

    def mesh(self):
        z=self.heights(); y,x=np.indices(z.shape)
        v=np.column_stack((self.origin[0]+x.ravel()*self.spacing,
                           self.origin[1]+y.ravel()*self.spacing,z.ravel()))
        n=z.shape[1]; i=(y[:-1,:-1]*n+x[:-1,:-1]).ravel()
        f=np.concatenate((np.column_stack((i,i+1,i+n+1)),np.column_stack((i,i+n+1,i+n))))
        return Mesh(v,f,closed=False)


@dataclass(frozen=True)
class HeightLOD:
    heights: np.ndarray
    stride: int
    max_vertical_error: float
    within_budget: bool
    guarantee: str = 'exact PL vertical deviation (floating-point arithmetic); not analytic source error'


def height_lod(heights, stride, budget):
    """Certified deviation for nested regular-grid piecewise-linear surfaces.

    Both grids split every cell from lower-left to upper-right. Coarse grid
    triangles are unions of fine triangles, so their difference is affine on
    every fine triangle and |difference| attains its maximum at a fine vertex.
    This exact vertical maximum also bounds Hausdorff distance, but is NOT the
    exact Hausdorff distance. Does not certify normals, silhouettes or collision.
    Independent neighboring tile LODs require seam stitching; not provided here.
    """
    h=np.asarray(heights,dtype=float)
    if h.ndim!=2 or min(h.shape)<2 or not np.isfinite(h).all():
        raise ValueError('finite 2D sample grid required')
    if not isinstance(stride,int) or stride<1 or any((s-1)%stride for s in h.shape):
        raise ValueError('stride must divide both cell counts')
    if not np.isfinite(budget) or budget<0:
        raise ValueError('nonnegative finite error budget required')
    coarse=h[::stride,::stride]
    yy,xx=np.indices(h.shape); u=xx/stride;v=yy/stride
    ix=np.minimum(u.astype(int),coarse.shape[1]-2);iy=np.minimum(v.astype(int),coarse.shape[0]-2)
    u-=ix;v-=iy
    a=coarse[iy,ix];b=coarse[iy,ix+1];c=coarse[iy+1,ix];d=coarse[iy+1,ix+1]
    reconstruction=np.where(u>=v,a+(b-a)*u+(d-b)*v,a+(d-c)*u+(c-a)*v)
    error=float(np.max(np.abs(h-reconstruction)))
    return HeightLOD(_readonly(coarse),stride,error,error<=budget)
