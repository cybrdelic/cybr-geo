// This validates a trace's representation, not its physical fidelity or provenance.
export function simulationSamples(simulation,partNames){
  if(simulation?.schema!=='cybrgeo.simulation/1'||simulation.backend!=='MuJoCo'||!Array.isArray(simulation.samples)||simulation.samples.length<2)throw Error('No recorded MuJoCo body-state trace');
  let previous=-Infinity;
  for(const sample of simulation.samples){
    if(!Number.isFinite(sample.time_s)||sample.time_s<0||sample.time_s<=previous)throw Error('Simulation times must increase');
    previous=sample.time_s;
    for(const name of partNames){
      const matrix=sample.body_poses_mm?.[name];
      if(!Array.isArray(matrix)||matrix.length!==4||matrix.some(row=>!Array.isArray(row)||row.length!==4||row.some(value=>!Number.isFinite(value))))throw Error('Missing or invalid simulated body pose: '+name);
      if(matrix[3].some((value,index)=>Math.abs(value-(index===3?1:0))>1e-8))throw Error('Invalid homogeneous body pose: '+name);
      // Only rigid solved poses can move the immutable CAD part geometry.
      for(let a=0;a<3;a++)for(let b=0;b<3;b++){
        const dot=matrix[0][a]*matrix[0][b]+matrix[1][a]*matrix[1][b]+matrix[2][a]*matrix[2][b];
        if(Math.abs(dot-(a===b?1:0))>1e-5)throw Error('Non-rigid simulated body pose: '+name);
      }
      const determinant=matrix[0][0]*(matrix[1][1]*matrix[2][2]-matrix[1][2]*matrix[2][1])-matrix[0][1]*(matrix[1][0]*matrix[2][2]-matrix[1][2]*matrix[2][0])+matrix[0][2]*(matrix[1][0]*matrix[2][1]-matrix[1][1]*matrix[2][0]);
      if(Math.abs(determinant-1)>1e-5)throw Error('Reflected simulated body pose: '+name);
    }
  }
  return simulation.samples;
}

export function sampleIndexAt(samples,seconds){
  let lo=0,hi=samples.length-1;
  while(lo<hi){const mid=Math.ceil((lo+hi)/2);if(samples[mid].time_s<=seconds)lo=mid;else hi=mid-1;}
  return lo;
}

export function engineeringAccess(response,plan,fromServer=false){
  const identity=plan?.construction?.model_sha256;
  const bound=fromServer&&!!identity&&response?.plan?.construction?.model_sha256===identity;
  const simulation=bound?response.simulation:null,clearance=bound?response.clearance:null;
  const simulationBound=simulation&&['model_sha256','mechanism_sha256','recipe_sha256'].every(key=>simulation[key]===plan.construction[key]);
  const clearanceBound=clearance&&clearance.compiled_geometry_sha256===plan?.construction?.recipe_sha256&&['model_sha256','mechanism_sha256'].every(key=>clearance[key]===plan.construction[key]);
  return {bound,accepted:bound&&response.passed===true&&response.motion_authorization?.allowed===true&&clearanceBound&&clearance.passed===true&&simulationBound&&simulation.tracking_passed===true,
    simulation:simulationBound?simulation:null,clearance:clearanceBound?clearance:null,
    identityError:bound&&((simulation&&!simulationBound)||(clearance&&!clearanceBound))?'A returned engineering record belongs to a different model or recipe.':null};
}

export function acceptedPoseAt(certificate,motion,seconds){
  const points=certificate?.waypoints;
  if(certificate?.passed!==true||certificate.interpolation!=='piecewise_linear_in_independent_joint_coordinates'||!Array.isArray(points)||points.length<2)throw Error('No accepted piecewise-linear motion record');
  const requested=motion.sample(seconds).values,names=Object.keys(requested).sort();
  if(points.some(point=>!point||Object.keys(point).sort().join('\0')!==names.join('\0')||Object.values(point).some(value=>!Number.isFinite(value))))throw Error('Accepted path controls differ from the motion task');
  // Same projection as native authorized_command: the comparison tolerance
  // does not enlarge the envelope. Execute the canonical projection itself.
  for(let index=0;index<points.length-1;index++){
    const start=points[index],end=points[index+1],delta=names.map(name=>end[name]-start[name]);
    const length=delta.reduce((sum,value)=>sum+value*value,0);
    const u=length?delta.reduce((sum,value,i)=>sum+(requested[names[i]]-start[names[i]])*value,0)/length:0;
    if(u>=-1e-10&&u<=1+1e-10){
      const canonical=Object.fromEntries(names.map((name,i)=>[name,start[name]+Math.max(0,Math.min(1,u))*delta[i]]));
      if(names.every(name=>Math.abs(requested[name]-canonical[name])<=1e-9))return canonical;
    }
  }
  throw Error('Motion command is outside the certified path');
}
