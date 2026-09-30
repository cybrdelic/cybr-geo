"""Conservative local-inertial runoff and three-class erosion/deposition.

Signed face discharges are shared by water and sediment. Donor limiting makes
dry fronts positive; every internal transfer has equal and opposite updates.
This is an illustrative process model, not a calibrated flood/soil predictor.
"""
from __future__ import annotations

from dataclasses import dataclass, asdict
import numpy as np
from .model import State, LOOSE_POROSITY, GRAINS

G=9.81


@dataclass
class Ledger:
    rain: float = 0
    inlet: float = 0
    outflow: float = 0
    evaporation: float = 0
    infiltration: float = 0
    eroded: float = 0
    deposited: float = 0
    thermal_transferred: float = 0


class Simulator:
    def __init__(self,state: State):
        state.validate();self.state=state;self.config=state.config;self.ledger=Ledger()
        self.initial_water=state.water_volume();self.initial_solids=state.solid_volumes().copy()
        self.exported_solids=np.zeros(3);self.history=[];self.last_dt=0

    def timestep(self):
        s=self.state;z=s.height;eta=z+s.water
        # Discharges live on faces: dividing them by a newly emptied cell
        # invents enormous velocities and stalls at a receding wet front.
        # Use the same hydrostatic face depth as the flow update.
        hx=np.maximum(0,np.maximum(eta[:,:-1],eta[:,1:])-np.maximum(z[:,:-1],z[:,1:]))
        hy=np.maximum(0,np.maximum(eta[:-1],eta[1:])-np.maximum(z[:-1],z[1:]))
        vx=np.where(hx>1e-6,np.abs(s.qx)/np.maximum(hx,1e-4),0)+np.sqrt(G*hx)
        vy=np.where(hy>1e-6,np.abs(s.qy)/np.maximum(hy,1e-4),0)+np.sqrt(G*hy)
        maximum=max(float(vx.max()),float(vy.max()),float(np.sqrt(G*s.water).max()),1e-5)
        return min(self.config.max_dt,self.config.cfl*self.config.dx/maximum)

    def _remove(self,wanted):
        """Remove a solid depth from the exposed stack, preserving its mixture."""
        s=self.state;remaining=np.maximum(wanted,0).copy();removed=np.zeros_like(s.sediment)
        # Removal can leave roundoff at complete depletion. Never divide a
        # negative roundoff inventory by the dry-cell epsilon.
        s.loose=np.maximum(s.loose,0)
        available=s.loose.sum(axis=0);take=np.minimum(remaining,available)
        fractions=s.loose/np.maximum(available,1e-30)
        parcel=fractions*take;s.loose=np.maximum(s.loose-parcel,0)
        removed+=parcel;remaining=np.maximum(remaining-take,0)
        for k in range(len(s.layers)-1,-1,-1):
            layer=s.layers[k];solid=s.thickness[k]*(1-layer.porosity)
            take=np.minimum(remaining,solid)
            s.thickness[k]-=take/(1-layer.porosity)
            s.thickness[k]=np.maximum(s.thickness[k],0)
            removed+=np.asarray(layer.fractions)[:,None,None]*take;remaining=np.maximum(remaining-take,0)
        return removed

    def _flow(self,dt):
        s=self.state;c=self.config;dx=c.dx;z=s.height;eta=z+s.water
        hx=np.maximum(0,np.maximum(eta[:,:-1],eta[:,1:])-np.maximum(z[:,:-1],z[:,1:]))
        hy=np.maximum(0,np.maximum(eta[:-1],eta[1:])-np.maximum(z[:-1],z[1:]))
        def face(q,h,slope):
            updated=(q-G*h*dt*slope)/(1+G*dt*c.manning**2*np.abs(q)/np.maximum(h,1e-6)**(7/3))
            return np.where(h>1e-7,updated,0)
        qx=face(s.qx,hx,(eta[:,1:]-eta[:,:-1])/dx)
        qy=face(s.qy,hy,(eta[1:]-eta[:-1])/dx)
        # A free-draining edge. There is no implicit external water source.
        out=np.zeros((4,c.grid))
        if c.boundary=="open":
            for i,h in enumerate((s.water[:,0],s.water[:,-1],s.water[0],s.water[-1])):
                out[i]=.35*np.sqrt(G)*h**1.5
        outgoing=np.zeros_like(z)
        outgoing[:,:-1]+=np.maximum(qx,0);outgoing[:,1:]+=np.maximum(-qx,0)
        outgoing[:-1]+=np.maximum(qy,0);outgoing[1:]+=np.maximum(-qy,0)
        outgoing[:,0]+=out[0];outgoing[:,-1]+=out[1];outgoing[0]+=out[2];outgoing[-1]+=out[3]
        scale=np.minimum(1,s.water*dx/np.maximum(outgoing*dt,1e-30))
        qx*=np.where(qx>=0,scale[:,:-1],scale[:,1:]);qy*=np.where(qy>=0,scale[:-1],scale[1:])
        out*=np.array([scale[:,0],scale[:,-1],scale[0],scale[-1]])
        old=s.water.copy();self._advect_sediment(old,qx,qy,out,dt)
        s.water[:,:-1]-=qx*dt/dx;s.water[:,1:]+=qx*dt/dx
        s.water[:-1]-=qy*dt/dx;s.water[1:]+=qy*dt/dx
        s.water[:,0]-=out[0]*dt/dx;s.water[:,-1]-=out[1]*dt/dx
        s.water[0]-=out[2]*dt/dx;s.water[-1]-=out[3]*dt/dx
        if s.water.min()<-1e-10:raise RuntimeError("Negative water after face limiter")
        s.water=np.maximum(s.water,0);s.qx=qx;s.qy=qy
        self.ledger.outflow+=float(out.sum()*dt*dx)

    def _advect_sediment(self,water,qx,qy,out,dt):
        s=self.state;dx=self.config.dx;concentration=s.sediment/np.maximum(water,1e-30)
        tx=np.where(qx[None]>=0,concentration[:,:,:-1],concentration[:,:,1:])*qx*dt/dx
        ty=np.where(qy[None]>=0,concentration[:,:-1],concentration[:,1:])*qy*dt/dx
        boundary=np.array([concentration[:,:,0],concentration[:,:,-1],concentration[:,0],concentration[:,-1]])
        boundary=boundary*out[:,None,:]*dt/dx
        s.sediment[:,:,:-1]-=tx;s.sediment[:,:,1:]+=tx
        s.sediment[:,:-1]-=ty;s.sediment[:,1:]+=ty
        s.sediment[:,:,0]-=boundary[0];s.sediment[:,:,-1]-=boundary[1]
        s.sediment[:,0]-=boundary[2];s.sediment[:,-1]-=boundary[3]
        if s.sediment.min()<-1e-10:raise RuntimeError("Negative sediment after advection")
        s.sediment=np.maximum(s.sediment,0)
        self.exported_solids+=boundary.sum(axis=(0,2))*dx**2

    def _properties(self,name):
        s=self.state;indices=s.exposed;values=np.array([getattr(layer,name) for layer in s.layers])
        result=values[np.maximum(indices,0)].copy()
        result[indices<0]=0
        return result

    def _exchange(self,dt):
        s=self.state;c=self.config;h=s.water
        ux=np.zeros_like(h);uy=np.zeros_like(h)
        ux[:,:-1]+=s.qx;ux[:,1:]+=s.qx;uy[:-1]+=s.qy;uy[1:]+=s.qy
        velocity=np.hypot(ux,uy)/(2*np.maximum(h,1e-4))
        tau=1000*G*c.manning**2*velocity**2/np.maximum(h,1e-4)**(1/3)
        tau=np.where(h>1e-5,tau,0)
        # Three mobility thresholds, with rapid gravel settling and slow fines.
        critical=np.array([8.0,1.2,.3])[:,None,None]
        capacity=.075*np.maximum(0,1-critical/np.maximum(tau,1e-8))*np.minimum(velocity/1.6,1)
        capacity*=np.array([.30,.46,.24])[:,None,None]
        target=capacity*h;deficit=np.maximum(target-s.sediment,0).sum(axis=0)
        resistance=self._properties("critical_shear");k=self._properties("erodibility")
        loose=s.loose.sum(axis=0)>1e-9;k=np.where(loose,6e-5,k);resistance=np.where(loose,.6,resistance)
        wanted=np.minimum(deficit,dt*c.morphological_factor*k*np.maximum(tau-resistance,0))
        wanted=np.minimum(wanted,.015*c.dx)
        parcel=self._remove(wanted);s.sediment+=parcel
        self.ledger.eroded+=float(parcel.sum()*c.dx**2)
        settling=np.array([.055,.013,.0014])[:,None,None]
        fraction=1-np.exp(-settling*dt*c.morphological_factor/np.maximum(h,1e-4))
        deposit=np.maximum(s.sediment-target,0)*fraction
        deposit=np.where(h[None]<1e-5,s.sediment,deposit)
        s.sediment-=deposit;s.loose+=deposit
        self.ledger.deposited+=float(deposit.sum()*c.dx**2)

    def _thermal(self,dt):
        s=self.state;c=self.config;z=s.height
        angle=self._properties("repose_degrees")
        angle=np.where(s.loose.sum(axis=0)>1e-7,35,angle)
        angle=np.where(s.exposed<0,89,angle)
        threshold=np.tan(np.radians(np.maximum(angle-4*s.saturation,5)))*c.dx
        dx=z[:,:-1]-z[:,1:];dy=z[:-1]-z[1:]
        tx=np.sign(dx)*np.maximum(np.abs(dx)-np.where(dx>=0,threshold[:,:-1],threshold[:,1:]),0)*c.thermal_rate*dt
        ty=np.sign(dy)*np.maximum(np.abs(dy)-np.where(dy>=0,threshold[:-1],threshold[1:]),0)*c.thermal_rate*dt
        # Solid-volume transfer; porosity changes may change bulk bed height.
        tx*=1-LOOSE_POROSITY;ty*=1-LOOSE_POROSITY
        wanted=np.zeros_like(z)
        wanted[:,:-1]+=np.maximum(tx,0);wanted[:,1:]+=np.maximum(-tx,0)
        wanted[:-1]+=np.maximum(ty,0);wanted[1:]+=np.maximum(-ty,0)
        limiter=np.minimum(1,.04*c.dx/np.maximum(wanted,1e-30))
        tx*=np.where(tx>=0,limiter[:,:-1],limiter[:,1:]);ty*=np.where(ty>=0,limiter[:-1],limiter[1:])
        wanted[:]=0
        wanted[:,:-1]+=np.maximum(tx,0);wanted[:,1:]+=np.maximum(-tx,0)
        wanted[:-1]+=np.maximum(ty,0);wanted[1:]+=np.maximum(-ty,0)
        parcel=self._remove(wanted)
        fractions=parcel/np.maximum(wanted,1e-30)
        fx=np.where(tx[None]>=0,fractions[:,:,:-1],fractions[:,:,1:])*tx
        fy=np.where(ty[None]>=0,fractions[:,:-1],fractions[:,1:])*ty
        # Donor has already lost parcel: add only recipient amounts.
        s.loose[:,:,1:]+=np.maximum(fx,0);s.loose[:,:,:-1]+=np.maximum(-fx,0)
        s.loose[:,1:]+=np.maximum(fy,0);s.loose[:,:-1]+=np.maximum(-fy,0)
        self.ledger.thermal_transferred+=float(parcel.sum()*c.dx**2)

    def step(self,dt=None):
        s=self.state;c=self.config
        dt=self.timestep() if dt is None else min(float(dt),self.timestep())
        if not np.isfinite(dt) or dt<=0:raise ValueError("Timestep must be positive and finite")
        # A storm wanes in the last quarter. The upstream inlet remains explicit.
        storm=1 if s.time<.75*c.duration else .15
        rain=c.rain_mm_hour/3600000*s.rain_pattern*storm*dt
        inlet=c.inlet_m3_second*s.inlet_pattern*dt/c.dx**2
        s.water+=rain+inlet
        self.ledger.rain+=float(rain.sum()*c.dx**2);self.ledger.inlet+=float(inlet.sum()*c.dx**2)
        self._flow(dt);self._exchange(dt);self._thermal(dt)
        capacity=s.pore_capacity
        spill=np.maximum(s.soil_water-capacity,0);s.soil_water-=spill;s.water+=spill
        conductivity=self._properties("conductivity")
        conductivity=np.where(s.loose.sum(axis=0)>1e-7,2e-5,conductivity)
        infiltrate=np.minimum(s.water,np.minimum(np.maximum(capacity-s.soil_water,0),conductivity*(1+1.5*(1-s.saturation))*dt))
        s.water-=infiltrate;s.soil_water+=infiltrate
        self.ledger.infiltration+=float(infiltrate.sum()*c.dx**2)
        evap=np.minimum(s.water,c.evaporation_mm_hour/3600000*dt);s.water-=evap
        soil_evap=np.minimum(s.soil_water,c.evaporation_mm_hour/3600000*dt*.35);s.soil_water-=soil_evap
        self.ledger.evaporation+=float((evap+soil_evap).sum()*c.dx**2)
        s.time+=dt;s.steps+=1;self.last_dt=dt
        if s.steps%100==0:s.validate()
        if s.steps>=c.max_steps:raise RuntimeError("Step budget exhausted; reduce duration or increase max_steps")
        return dt

    def run(self,callback=None):
        next_record=self.state.time
        while self.state.time<self.config.duration-1e-10:
            self.step(min(self.timestep(),self.config.duration-self.state.time))
            if self.state.time>=next_record:
                report=self.report();self.history.append(report)
                if callback:callback(report)
                next_record=self.state.time+max(1,self.config.duration/16)
        self.state.validate();report=self.report()
        if not report["passed"]:raise RuntimeError("Conservation check failed: "+str(report))
        return report

    def report(self):
        s=self.state;l=self.ledger
        expected_water=self.initial_water+l.rain+l.inlet-l.outflow-l.evaporation
        water_error=s.water_volume()-expected_water
        solid_error=s.solid_volumes()+self.exported_solids-self.initial_solids
        wr=abs(water_error)/max(1,self.initial_water+l.rain+l.inlet)
        sr=float(np.max(np.abs(solid_error)/np.maximum(1,self.initial_solids)))
        return {"passed":wr<1e-9 and sr<1e-9,"time_seconds":s.time,"steps":s.steps,
                "water_relative_error":wr,"solid_relative_error":sr,"water_error_m3":water_error,
                "solid_error_m3":dict(zip(GRAINS,solid_error.tolist())),
                "exported_solid_m3":dict(zip(GRAINS,self.exported_solids.tolist())),
                "surface_water_m3":float(s.water.sum()*self.config.dx**2),
                "soil_water_m3":float(s.soil_water.sum()*self.config.dx**2),
                "max_erosion_depth_m":float(np.max(s.initial_height-s.height)),
                "max_deposition_depth_m":float(np.max(s.height-s.initial_height)),
                "ledger_m3":asdict(l),"morphological_factor":self.config.morphological_factor,
                "model":"local-inertial finite-volume runoff; conservative multiclass bed exchange; shallow soil bucket"}
