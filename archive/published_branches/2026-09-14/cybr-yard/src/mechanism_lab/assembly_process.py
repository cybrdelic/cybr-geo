"""Reversible, ordered mechanical service procedures over actual CAD parts.

An exploded layout is not a procedure. A procedure specifies retained units,
release dependencies, tool access, and explicit translation/helical paths. This
module validates the sampled rigid geometry; it does not simulate a mechanic,
elastic fits, tightening torque, or gravity. Purchased/bonded units remain whole.
"""
from __future__ import annotations
from dataclasses import dataclass,field,asdict,replace
from pathlib import Path
import json,math
import numpy as np
from .core import pose_cad


def rigid(axis=(1.,0.,0.),angle=0.,pivot=(0.,0.,0.),translation=(0.,0.,0.)):
    axis=np.asarray(axis,float);axis=axis/np.linalg.norm(axis)
    x,y,z=axis;c,s=math.cos(angle),math.sin(angle)
    skew=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
    R=c*np.eye(3)+(1-c)*np.outer(axis,axis)+s*skew
    T=np.eye(4);T[:3,:3]=R;p=np.asarray(pivot,float)
    T[:3,3]=p-R@p+np.asarray(translation,float)
    return T


@dataclass
class ToolAccess:
    origin:tuple[float,float,float]
    axis:tuple[float,float,float]
    radius:float=1.3
    reach:float=80.
    handle_radius:float=8.
    handle_length:float=35.
    bend:tuple[float,float,float]|None=None
    description:str='Straight hex driver'

    def __post_init__(self):
        self.origin=tuple(map(float,self.origin));self.axis=tuple(map(float,self.axis))
        if self.bend is not None:self.bend=tuple(map(float,self.bend))

    def shape(self):
        import cadquery as cq
        o=np.asarray(self.origin,float);a=np.asarray(self.axis,float);a/=np.linalg.norm(a)
        tip=cq.Solid.makeCylinder(self.radius,self.reach,cq.Vector(*o),cq.Vector(*a))
        end=o+a*self.reach
        if self.bend is not None:
            b=np.asarray(self.bend,float);b/=np.linalg.norm(b)
            return tip.fuse(cq.Solid.makeCylinder(self.radius,self.handle_length,cq.Vector(*end),cq.Vector(*b)))
        return tip.fuse(cq.Solid.makeCylinder(self.handle_radius,self.handle_length,cq.Vector(*end),cq.Vector(*a)))


@dataclass
class Move:
    id:str
    title:str
    parts:tuple[str,...]
    delta:tuple[float,float,float]
    duration:float=1.
    axis:tuple[float,float,float]=(1.,0.,0.)
    turns:float=0.
    pivot:tuple[float,float,float]=(0.,0.,0.)
    requires:tuple[str,...]=()
    tool:ToolAccess|None=None
    note:str=''

    def __post_init__(self):
        self.delta=tuple(map(float,self.delta));self.axis=tuple(map(float,self.axis));self.pivot=tuple(map(float,self.pivot))

    def transform(self,u):
        return rigid(self.axis,math.tau*self.turns*u,self.pivot,np.asarray(self.delta)*u)


@dataclass
class AssemblyProcess:
    name:str
    moves:list[Move]
    retained_units:dict[str,list[str]]=field(default_factory=dict)
    stationary:list[str]=field(default_factory=list)
    limitations:list[str]=field(default_factory=list)

    def bind(self,assembly):
        names={p.name for p in assembly.parts};seen=set();self._starts=[]
        units=[set(unit) for unit in self.retained_units.values()]
        if any(unit-names for unit in units):raise ValueError('Unknown retained-unit component')
        state={p.name:np.asarray(assembly.pose(p,0.,0.),float).copy() for p in assembly.parts};clock=0.;self._times=[]
        for move in self.moves:
            if not move.parts or set(move.parts)-names:raise ValueError(f'{move.id}: unknown/empty moving parts')
            if move.id in seen or set(move.requires)-seen:raise ValueError(f'{move.id}: unmet release dependencies {move.requires}')
            if set(move.parts)&set(self.stationary):raise ValueError(f'{move.id}: a grounded part cannot move')
            if any(set(move.parts)&unit and not unit<=set(move.parts) for unit in units):
                raise ValueError(f'{move.id}: operation splits a retained service unit')
            if move.duration<=0 or not np.isfinite(move.delta).all():raise ValueError('Invalid operation')
            self._starts.append({name:T.copy() for name,T in state.items()})
            self._times.append(clock);clock+=move.duration
            for name in move.parts:state[name]=move.transform(1.)@state[name]
            seen.add(move.id)
        self._final=state;self.duration=clock;self._names=names
        return self

    def at_move(self,index,u):
        state={name:T.copy() for name,T in self._starts[index].items()}
        move=self.moves[index];T=move.transform(float(np.clip(u,0,1)))
        for name in move.parts:state[name]=T@state[name]
        return state

    def poses(self,time,reverse=False):
        if reverse:time=self.duration-time
        if time>=self.duration:return self._final
        time=max(0.,time)
        i=min(len(self.moves)-1,int(np.searchsorted(self._times,time,side='right')-1))
        u=(time-self._times[i])/self.moves[i].duration
        u=u*u*(3-2*u)
        return self.at_move(i,u)

    def assembly(self,assembly,reverse=False):
        self.bind(assembly)
        def pose(p,t,e):
            clock=self.duration-t if reverse else t
            if clock>=self.duration:return self._final[p.name]
            clock=max(0.,clock)
            i=min(len(self.moves)-1,int(np.searchsorted(self._times,clock,side='right')-1))
            move=self.moves[i];T=self._starts[i][p.name]
            if p.name not in move.parts:return T
            u=(clock-self._times[i])/move.duration;u=u*u*(3-2*u)
            return move.transform(u)@T
        return replace(assembly,motion_function=pose,metadata=dict(assembly.metadata,
            service_procedure=self.name,procedure_duration_seconds=self.duration))

    def write(self,path):
        payload=asdict(self);payload.update(duration_seconds=self.duration,
            validation_scope='Nominal sampled rigid CAD paths and tool envelopes; no force/elastic/contact dynamics')
        Path(path).write_text(json.dumps(payload,indent=2)+'\n')
        return payload


def _bounds(shape):
    b=shape.BoundingBox()
    return np.array([[b.xmin,b.ymin,b.zmin],[b.xmax,b.ymax,b.zmax]])


def _overlap(a,b,tolerance=1e-6):
    return bool(np.all(np.minimum(a[1],b[1])-np.maximum(a[0],b[0])>tolerance))


def validate_process(assembly,process,linear_step_mm=1.,angular_step_degrees=15.,volume_tolerance_mm3=1e-4,progress=None,collision_envelopes=None,operation_indices=None,floor_z=-2.):
    """Check EVERY moving analytic component against EVERY remaining component.

    Fasteners have modeled nominal clearance envelopes. Actual helical nut
    sweeps are tested against the actual helical screw. AABB rejection is only
    a broad phase; all surviving pairs use OpenCascade solid intersections.
    The report explicitly records sampling, not a continuous-motion certificate.
    """
    process.bind(assembly);parts={p.name:p for p in assembly.parts}
    analytic={n:p.cad for n,p in parts.items() if p.cad is not None}
    collision_envelopes=collision_envelopes or {}
    analytic.update(collision_envelopes)
    report=dict(passed=True,linear_step_mm=linear_step_mm,angular_step_degrees=angular_step_degrees,
                volume_tolerance_mm3=volume_tolerance_mm3,operations=[],collisions=[],tool_obstructions=[],floor_collisions=[],
                unchecked_mesh_parts=[n for n,p in parts.items() if p.cad is None and n not in collision_envelopes],
                conservative_envelope_parts=list(collision_envelopes),
                method='All analytic moving/stationary pairs, AABB broad phase then exact CAD intersection at sampled poses',
                continuous_certificate=False,forces_simulated=False)
    for i,move in enumerate(process.moves):
        if operation_indices is not None and i not in operation_indices:continue
        start=process._starts[i];moving=set(move.parts)
        fixed={n:pose_cad(c,start[n]) for n,c in analytic.items() if n not in moving}
        fixed_bounds={n:_bounds(s) for n,s in fixed.items()}
        row=dict(id=move.id,title=move.title,moving_parts=list(move.parts),samples=0,cad_tests=0,max_overlap_mm3=0.,tool_tests=0)
        if move.tool is not None:
            tool=move.tool.shape();tb=_bounds(tool)
            for name,shape in fixed.items():
                if not _overlap(tb,fixed_bounds[name]):continue
                row['tool_tests']+=1
                vol=sum(abs(s.Volume()) for s in tool.intersect(shape).Solids())
                if vol>volume_tolerance_mm3:
                    report['tool_obstructions'].append(dict(operation=move.id,part=name,overlap_mm3=vol))
        samples=max(1,math.ceil(np.linalg.norm(move.delta)/linear_step_mm),math.ceil(abs(move.turns)*360/angular_step_degrees))
        for j in range(samples+1):
            u=j/samples;T=move.transform(u);row['samples']+=1
            for name in sorted(moving & analytic.keys()):
                shape=pose_cad(analytic[name],T@start[name]);bounds=_bounds(shape)
                if bounds[0,2]<floor_z-1e-3 and not any(r['operation']==move.id and r['part']==name for r in report['floor_collisions']):
                    report['floor_collisions'].append(dict(operation=move.id,part=name,u=u,minimum_z=float(bounds[0,2])))
                for other,ob in fixed_bounds.items():
                    if not _overlap(bounds,ob):continue
                    row['cad_tests']+=1
                    vol=sum(abs(s.Volume()) for s in shape.intersect(fixed[other]).Solids())
                    row['max_overlap_mm3']=max(row['max_overlap_mm3'],vol)
                    if vol>volume_tolerance_mm3:
                        # One recorded collision per pair/operation is enough
                        # to fail the entire procedure; continue auditing others.
                        key=(move.id,name,other)
                        if not any((r['operation'],r['moving'],r['obstacle'])==key for r in report['collisions']):
                            report['collisions'].append(dict(operation=move.id,moving=name,obstacle=other,u=u,overlap_mm3=vol))
        report['operations'].append(row)
        if progress:progress(row,report)
    report['passed']=not(report['collisions'] or report['tool_obstructions'] or report['unchecked_mesh_parts'] or report['floor_collisions'])
    report['geometric_paths_passed']=not(report['collisions'] or report['tool_obstructions'] or report['floor_collisions'])
    return report
