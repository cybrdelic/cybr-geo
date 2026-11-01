// Backend-neutral counterpart of mechanism_lab.motion.MotionTask.
// Units remain the generated joint's degrees or millimetres; time is seconds.
const object=value=>value!==null&&typeof value==='object'&&!Array.isArray(value);
const keysEqual=(a,b)=>Object.keys(a).sort().join('\0')===Object.keys(b).sort().join('\0');
export class MotionTask {
  constructor(document){
    if(!object(document)||!keysEqual(document,{schema:0,period_s:0,interpolation:0,waypoints:0,actuator_bindings:0})||document.schema!=='cybrgeo-motion/1'||document.interpolation!=='cosine_out_and_back')throw Error('Unsupported lowered motion task');
    if(!Number.isFinite(document.period_s)||document.period_s<.1||document.period_s>3600)throw Error('Motion period must be within [0.1, 3600] seconds');
    const bindings=document.actuator_bindings;
    if(!object(bindings)||Object.values(bindings).some(value=>typeof value!=='string')||new Set(Object.values(bindings)).size!==Object.keys(bindings).length)throw Error('Motion actuator bindings must map unique joint names');
    const expected=Object.fromEntries(Object.values(bindings).map(name=>[name,0]));
    if(!Array.isArray(document.waypoints)||document.waypoints.length!==2||document.waypoints.some(point=>!object(point)||!keysEqual(point,expected)||Object.values(point).some(value=>!Number.isFinite(value))))throw Error('Motion requires two finite waypoints covering every independent control');
    this.period_s=document.period_s;this.waypoints=document.waypoints.map(point=>({...point}));this.actuator_bindings={...bindings};
  }
  sample(seconds){
    if(!Number.isFinite(seconds))throw Error('Motion time must be finite');
    const omega=2*Math.PI/this.period_s,phase=omega*seconds,blend=.5*(1-Math.cos(phase)),speed=.5*omega*Math.sin(phase),acceleration=.5*omega*omega*Math.cos(phase);
    const values={},velocities={},accelerations={};
    for(const [name,start] of Object.entries(this.waypoints[0])){const delta=this.waypoints[1][name]-start;values[name]=start+delta*blend;velocities[name]=delta*speed;accelerations[name]=delta*acceleration;}
    return {values,velocities,accelerations};
  }
  peakRates(){
    const omega=2*Math.PI/this.period_s;
    return Object.fromEntries(Object.entries(this.waypoints[0]).map(([name,start])=>{const delta=Math.abs(this.waypoints[1][name]-start);return [name,{speed:delta*omega/2,acceleration:delta*omega*omega/2}];}));
  }
}
