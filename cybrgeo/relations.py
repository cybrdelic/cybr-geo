"""Generic rigid reference-frame kinematics and local point-closure solving."""
from dataclasses import dataclass
from types import MappingProxyType
import numpy as np
from .deform import rotation

@dataclass(frozen=True)
class Pose:
    coordinates: dict
    transforms: dict
    closure_error: float

class Kinematics:
    """Axes, anchors and geometry are in a shared authored reference frame.

    A joint is a mathematical relationship, not proof of physical mating hardware.
    Closed-loop solves are local, require drivers, and can fail near singularities.
    """
    def __init__(self):
        self.joints={};self.closures=[]

    def fixed(self,name,parent=None):
        return self._add(name,parent,'fixed',(0,0,0),(0,0,1),(0,0))

    def revolute(self,name,parent,origin,axis=(0,0,1),limits=(-360,360)):
        return self._add(name,parent,'revolute',origin,axis,limits)

    def prismatic(self,name,parent,axis=(1,0,0),limits=(-100,100)):
        return self._add(name,parent,'prismatic',(0,0,0),axis,limits)

    def _add(self,name,parent,kind,origin,axis,limits):
        if not isinstance(name,str) or not name or name in self.joints or (parent is not None and parent not in self.joints):
            raise ValueError('Joint needs a unique name and an existing parent')
        axis=np.asarray(axis,float);origin=np.asarray(origin,float);limits=np.asarray(limits,float)
        if axis.shape!=(3,) or origin.shape!=(3,) or limits.shape!=(2,) or not np.isfinite(np.r_[axis,origin,limits]).all() or np.linalg.norm(axis)<1e-12 or limits[0]>limits[1]:
            raise ValueError('Invalid joint axis, origin or limits')
        self.joints[name]=(parent,kind,origin.copy(),axis/np.linalg.norm(axis),limits.copy())
        return self

    def coincident(self,body_a,point_a,body_b,point_b):
        a,b=np.asarray(point_a,float),np.asarray(point_b,float)
        if body_a not in self.joints or body_b not in self.joints or a.shape!=(3,) or b.shape!=(3,) or not np.isfinite([a,b]).all():
            raise ValueError('Closure requires existing bodies and finite reference points')
        self.closures.append((body_a,a.copy(),body_b,b.copy()));return self

    def _transforms(self,coordinates):
        transforms={}
        for name,(parent,kind,origin,axis,limits) in self.joints.items():
            value=coordinates.get(name,0.)
            local=np.eye(4)
            if kind=='revolute':local=rotation(axis,value,origin)
            elif kind=='prismatic':local[:3,3]=axis*value
            transforms[name]=local if parent is None else transforms[parent]@local
        return transforms

    def _residual(self,coordinates):
        transforms=self._transforms(coordinates)
        values=[(transforms[a]@np.r_[pa,1]-transforms[b]@np.r_[pb,1])[:3] for a,pa,b,pb in self.closures]
        return np.concatenate(values) if values else np.zeros(0)

    def solve(self,driven,guess=None,tolerance=1e-7):
        movable={name:j for name,j in self.joints.items() if j[1]!='fixed'}
        if set(driven)-movable.keys():raise ValueError('Unknown or fixed driver')
        coordinates={name:float((guess or {}).get(name,0.)) for name in movable};coordinates.update(driven)
        if not all(np.isfinite(v) for v in coordinates.values()) or not np.isfinite(tolerance) or tolerance<=0:
            raise ValueError('Coordinates and positive tolerance must be finite')
        free=[name for name in movable if name not in driven]
        if self.closures and not driven:raise ValueError('Closed mechanism requires specified drivers')
        if self.closures:
            for _ in range(50):
                error=self._residual(coordinates)
                if not free:
                    if np.max(np.abs(error))>tolerance:raise ValueError('Prescribed coordinates conflict with closure')
                    break
                columns=[]
                for name in free:
                    trial=dict(coordinates);trial[name]+=1e-4
                    columns.append((self._residual(trial)-error)/1e-4)
                jacobian=np.column_stack(columns)
                if np.linalg.matrix_rank(jacobian,tol=1e-9)<len(free):
                    raise ValueError('Mechanism is underconstrained or locally singular')
                if np.max(np.abs(error))<=tolerance:break
                delta=np.linalg.lstsq(jacobian,-error,rcond=None)[0]
                delta*=min(1.,20/max(np.max(np.abs(delta)),1e-12))
                for name,value in zip(free,delta):coordinates[name]+=value
            else:raise ValueError('Closure did not converge; supply a nearer branch guess')
        for name,value in coordinates.items():
            low,high=movable[name][-1]
            if not low<=value<=high:raise ValueError('Coordinate outside limits: '+name)
        transforms=self._transforms(coordinates)
        for matrix in transforms.values():matrix.flags.writeable=False
        error=self._residual(coordinates)
        return Pose(MappingProxyType(dict(coordinates)),MappingProxyType(transforms),float(np.max(np.abs(error))) if len(error) else 0.)
