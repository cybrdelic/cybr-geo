// Test-only execution harness around byte-exact original ARX source extracts.
// It does not implement a competing simulator or numerical approximation.
import fs from 'node:fs';
import {SafetyState,springState,brakeFraction} from './source-safety.mjs';
const input=JSON.parse(fs.readFileSync(0,'utf8'));
const clip=(x,a,b)=>Math.max(a,Math.min(b,x));
const block=name=>fs.readFileSync(new URL(name,import.meta.url),'utf8');
let result;
if(input.kind==='motor'){
  const kernel=new Function('par','o','i','qd','dt','tau','ctrl','clip',block('source-motor-block.js'));
  const state={current:[input.current??0],temperature:[input.temperature??25],voltage:[0],torque:[0],
    electricalEnergy:[0],copperHeat:[0],mechanicalEnergy:[0],gearHeat:[0],
    monitor:{beltDemandLimitSeconds:[0],gearDemandLimitSeconds:[0]}};
  const par=Object.fromEntries(Object.entries(input.config).map(([k,v])=>[k,[v]]));
  result=input.steps.map(s=>{state.motorPower=s.power??true;kernel.call(state,par,{torqueScale:s.scale??1},0,[s.speed],s.dt,s.demand,[0],clip);return JSON.parse(JSON.stringify(state));});
}else if(input.kind==='gripper'){
  const kernel=new Function('gr','o','grossRequest','rotorSpeed','gripTransition','etaRequest','dt','ctrl','clip',block('source-gripper-block.js'));
  const gr=input.config,state={gripperCurrent:input.current??0,gripperTemperature:input.temperature??25,
    gripperI2Integral:0,gripperVoltage:0,gripperTorque:0};
  result=input.steps.map(s=>{state.gripperPower=s.power??true;state.gripperCurrentCeiling=s.ceiling;
    const rotorSpeed=gr.gear_ratio*s.speed,transition=Math.tanh(rotorSpeed/gr.gear_efficiency_transition_motor_rad_s);
    const eta=Math.exp(Math.log(gr.gear_efficiency)*transition*Math.sign(s.demand));
    kernel.call(state,gr,{gripScale:s.scale??1},s.demand,rotorSpeed,transition,eta,s.dt,new Array(7),clip);
    return {...state};});
}else if(input.kind==='spring'){
  result=input.cases.map(c=>springState(c.q,input.spec,c.strength,c.index));
}else if(input.kind==='brake'){
  result=input.cases.map(c=>brakeFraction(c.time,c.fault,input.config,c.scale));
}else if(input.kind==='budget'){
  const kernel=new Function('dt',block('source-current-budget-block.js'));
  const state={peakCredit:input.config.peak_budget_s,spec:{gripper_driver:input.config}};
  result=input.steps.map(s=>{state.gripperCurrent=s.current;kernel.call(state,s.dt);return {credit:state.peakCredit,ceiling:state.gripperCurrentCeiling};});
}else if(input.kind==='safety'){
  const state=new SafetyState({joint_soft_limits_rad:input.limits});
  result=input.operations.map(o=>{
    if(o.action==='command')state.command(o.time,o.valid??true);
    if(o.action==='stop')state.normalStop(o.time,o.q,o.qd,o.gap);
    if(o.action==='update')state.update(o.time,o.q,o.qd,o.options??{});
    return {mode:state.mode,reason:state.reason,fault_time_s:state.faultTime,events:structuredClone(state.events),reference:state.reference(o.time,input.normal)};
  });
}else throw new Error('Unknown fixed test case');
process.stdout.write(JSON.stringify(result));
