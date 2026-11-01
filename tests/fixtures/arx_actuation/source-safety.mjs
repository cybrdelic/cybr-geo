/* P2 simulation/control reference. This is NOT a safety-rated controller. */
export const clamp=(x,a,b)=>Math.max(a,Math.min(b,x));
export function springState(q,spec,strength=1,jointIndex=1){
  const s=jointIndex===2?spec.elbow_counterbalance:spec.counterbalance, A=s.parent_anchor_m, B=s.child_anchor_m;
  // Joint 2 rotates about -Z. The two springs have identical planar mechanics.
  const c=Math.cos(q),sn=Math.sin(q),bx=c*B[0]+sn*B[1],by=-sn*B[0]+c*B[1];
  const dx=A[0]-bx,dy=A[1]-by,dz=A[2]-B[2],L=Math.hypot(dx,dy,dz);
  const extension=Math.max(0,L-s.free_eye_length_m);
  const F=L>=s.free_eye_length_m ? strength*(s.initial_tension_N+s.stiffness_N_m*extension):0;
  const torque=-s.spring_count*(bx*F*dy/L-by*F*dx/L);
  const energy=s.spring_count*strength*(s.initial_tension_N*extension+.5*s.stiffness_N_m*extension*extension);
  return {length_m:L,force_each_N:F,torque_Nm:torque,potential_energy_J:energy};
}
export class SafetyState {
  constructor(spec){this.spec=spec;this.mode='RUN';this.reason='';this.faultTime=null;this.stopTime=null;this.lastCommandTime=0;this.stopSnapshot=null;this.events=[];}
  trip(reason,t){if(this.mode==='EMERGENCY')return;this.mode='EMERGENCY';this.reason=reason;this.faultTime=t;this.events.push({t,mode:this.mode,reason});}
  command(t,valid=true){if(valid)this.lastCommandTime=t;}
  normalStop(t,q,qd,gap){
    if(this.mode!=='RUN')return;
    const T=.35,end=q.map((x,i)=>x+.5*T*qd[i]);
    if(end.some((x,i)=>x<this.spec.joint_soft_limits_rad[i][0]||x>this.spec.joint_soft_limits_rad[i][1])){this.trip('controlled-stop-envelope',t);return;}
    this.mode='CONTROLLED_STOP';this.stopTime=t;this.stopSnapshot={q:[...q],qd:[...qd],gap,T};
    this.events.push({t,mode:this.mode,reason:'operator normal stop'});
  }
  update(t,q,qd,{estop=false,power=true,feedbackValid=true,watchdogSeconds=.1,limits=true}={}){
    if(estop)this.trip('emergency-stop',t);
    else if(!power)this.trip('power-loss',t);
    else if(!feedbackValid||q.some(x=>!Number.isFinite(x)))this.trip('feedback-invalid',t);
    else if(t-this.lastCommandTime>watchdogSeconds+1e-9)this.trip('command-watchdog',t);
    else if(limits&&q.some((x,i)=>x<this.spec.joint_soft_limits_rad[i][0]-.002||x>this.spec.joint_soft_limits_rad[i][1]+.002))this.trip('software-travel-limit',t);
    if(this.mode==='CONTROLLED_STOP'&&t-this.stopTime>=this.stopSnapshot.T){this.mode='HOLD';this.events.push({t,mode:this.mode,reason:'controlled deceleration complete'});}
  }
  reference(t,normal){
    if(!this.stopSnapshot||!['CONTROLLED_STOP','HOLD'].includes(this.mode))return normal;
    const s=this.stopSnapshot,u=clamp((t-this.stopTime)/s.T,0,1);
    const f=u-2.5*u**4+3*u**5-u**6,v=1-10*u**3+15*u**4-6*u**5,a=(-30*u*u+60*u**3-30*u**4)/s.T;
    return {q:s.q.map((x,i)=>x+s.qd[i]*s.T*f),qd:s.qd.map(x=>x*v),qdd:s.qd.map(x=>x*a),gap:s.gap,stage:8};
  }
}
export function brakeFraction(t,faultTime,b,delayScale=1){
  if(faultTime==null)return 0;
  return clamp((t-faultTime-b.delay_s*delayScale)/(b.ramp_s*delayScale),0,1);
}
