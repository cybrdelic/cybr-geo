const clip=(x,a,b)=>Math.max(a,Math.min(b,x));
const dot=(a,b)=>a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
const norm=a=>Math.hypot(...a);
const scale=(a,s)=>a.map(x=>x*s);
const add=(a,b)=>a.map((x,i)=>x+b[i]);
const sub=(a,b)=>a.map((x,i)=>x-b[i]);
const mix=(a,b,t)=>a.map((x,i)=>x+(b[i]-x)*t);
const ease=t=>{t=clip(t,0,1);return t*t*t*(t*(t*6-15)+10);};
const phase=(t,a,b)=>ease((t-a)/(b-a));
function ry(a){const c=Math.cos(a),s=Math.sin(a);return [c,0,s,0,1,0,-s,0,c];}
function rz(a){const c=Math.cos(a),s=Math.sin(a);return [c,-s,0,s,c,0,0,0,1];}
function mm(a,b){const o=new Array(9);for(let i=0;i<3;i++)for(let j=0;j<3;j++){o[3*i+j]=0;for(let k=0;k<3;k++)o[3*i+j]+=a[3*i+k]*b[3*k+j];}return o;}
function transposeMul(a,v){return [a[0]*v[0]+a[3]*v[1]+a[6]*v[2],a[1]*v[0]+a[4]*v[1]+a[7]*v[2],a[2]*v[0]+a[5]*v[1]+a[8]*v[2]];}

export function reference(time,p){
  let t=time%10;if(t<0)t+=10;
  const returning=t>=5,h=returning?t-5:t,dir=returning?-1:1,s=p.scene_units_to_meters;
  const a=p.workpiece_targets_m.A,b=p.workpiece_targets_m.B;
  const start=returning?b:a,end=returning?a:b,hs=[start[0],.92*s,start[2]],he=[end[0],.92*s,end[2]];
  const travel=phase(h,1.65,2.75),hump=16*travel*travel*(1-travel)*(1-travel);
  let tcp,stage;
  if(h<.65){tcp=mix(hs,start,phase(h,0,.65));stage=0;}
  else if(h<1){tcp=start.slice();stage=1;}
  else if(h<1.65){tcp=mix(start,hs,phase(h,1,1.65));stage=2;}
  else if(h<2.75){tcp=mix(hs,he,travel);tcp[1]+=.16*s*hump;stage=3;}
  else if(h<3.4){tcp=mix(he,end,phase(h,2.75,3.4));stage=4;}
  else if(h<3.75){tcp=end.slice();stage=5;}
  else if(h<4.45){tcp=mix(end,he,phase(h,3.75,4.45));stage=6;}
  else{tcp=he.slice();stage=7;}
  const cfg=p.controller,opened=cfg.jaw_open_half_gap_m,closed=cfg.jaw_closed_target_half_gap_m;
  const gap=h>=.40&&h<3.4?closed:opened;
  let direction=[.15*hump,-1,dir*.34*hump];direction=scale(direction,1/norm(direction));
  const wrist=sub(tcp,scale(direction,p.tool_length_m)),delta=sub(wrist,[0,p.shoulder_height_m,0]);
  const radial=Math.hypot(delta[0],delta[2]),q1=Math.atan2(delta[2],delta[0]),base=ry(-q1);
  const l1=p.upper_length_m,l2=p.fore_length_m;
  const q3=Math.acos(clip((dot(delta,delta)-l1*l1-l2*l2)/(2*l1*l2),-1,1));
  const elevation=Math.atan2(delta[1],radial)+Math.atan2(l2*Math.sin(q3),l1+l2*Math.cos(q3));
  const q2=Math.PI/2-elevation,fore=mm(mm(base,rz(-q2)),rz(-q3)),local=transposeMul(fore,direction);
  const q4=Math.atan2(-local[2],local[0]),q5=Math.acos(clip(local[1],-1,1)),bend=mm(mm(fore,ry(q4)),rz(-q5));
  let refx=sub([1,0,0],scale(direction,direction[0]));refx=scale(refx,1/norm(refx));
  const refLocal=transposeMul(bend,refx);
  let q6=Math.atan2(-refLocal[2],refLocal[0]);if(q6<0)q6+=2*Math.PI;q6+=dir*.5*hump;
  return {q:[q1,q2,q3,q4,q5,q6],gap,stage};
}
export function referenceDerivatives(t,p){
  const e=.00025,r=reference(t,p),a=reference(t-e,p).q,b=reference(t+e,p).q;
  r.qd=r.q.map((_,i)=>(b[i]-a[i])/(2*e));r.qdd=r.q.map((x,i)=>(b[i]-2*x+a[i])/(e*e));return r;
}

