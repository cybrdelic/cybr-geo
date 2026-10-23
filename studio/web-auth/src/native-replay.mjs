// Native project replay is a display of recorded body states, not a solver.
export function validateNativeReplay(replay){
  if(replay?.schema!=='cybrgeo-native-replay/1'||!Array.isArray(replay.times_s)||!replay.times_s.length)throw Error('No native recorded-state replay');
  if(replay.units!=='mm'||replay.up_axis!=='Z'||replay.interpolation!=='previous_recorded_state')throw Error('Native replay must use mm/Z-up and held recorded states');
  if(!replay.part_bindings||!replay.transforms_mm||!Object.keys(replay.part_bindings).length)throw Error('Native replay has no body bindings');
  let previous=-Infinity;
  for(const time of replay.times_s){if(!Number.isFinite(time)||time<0||time<=previous)throw Error('Native replay times must increase');previous=time;}
  const bodies=new Set(Object.values(replay.part_bindings));
  for(const body of bodies){
    const poses=replay.transforms_mm[body];
    if(!Array.isArray(poses)||poses.length!==replay.times_s.length)throw Error('Missing recorded body: '+body);
    for(const m of poses){
      if(!Array.isArray(m)||m.length!==4||m.some(row=>!Array.isArray(row)||row.length!==4||row.some(x=>!Number.isFinite(x))))throw Error('Invalid recorded body matrix: '+body);
      if(m[3].some((x,i)=>Math.abs(x-(i===3?1:0))>1e-9))throw Error('Invalid homogeneous body matrix');
      for(let a=0;a<3;a++)for(let b=0;b<3;b++){const dot=m[0][a]*m[0][b]+m[1][a]*m[1][b]+m[2][a]*m[2][b];if(Math.abs(dot-(a===b?1:0))>1e-7)throw Error('Non-rigid native body matrix');}
      const det=m[0][0]*(m[1][1]*m[2][2]-m[1][2]*m[2][1])-m[0][1]*(m[1][0]*m[2][2]-m[1][2]*m[2][0])+m[0][2]*(m[1][0]*m[2][1]-m[1][1]*m[2][0]);
      if(Math.abs(det-1)>1e-7)throw Error('Reflected native body matrix');
    }
  }
  return replay;
}

export function nativeReplayAccess(report,replay,fromServer=false){
  const bound=fromServer&&report?.schema==='cybrgeo-engineering-result/2'
    &&!!report.model_sha256&&report.model_sha256===replay?.model_sha256
    &&!!report.control_sha256&&report.control_sha256===replay?.control_sha256;
  const reference=report?.simulation?.reference_sha256;
  if(!bound||(reference&&reference!==replay.reference_sha256))throw Error('Native geometry/replay receipt is not bound to this server run');
  validateNativeReplay(replay);
  return {bound:true,passed:report.passed===true&&report.requirements?.passed===true};
}

export function nativeFrameAt(replay,time){
  if(!Number.isFinite(time))throw Error('Replay time must be finite');
  let lo=0,hi=replay.times_s.length-1;
  while(lo<hi){const mid=Math.ceil((lo+hi)/2);if(replay.times_s[mid]<=time)lo=mid;else hi=mid-1;}
  return lo;
}

export function nativeMeshes(T,gltfScene,replay){
  validateNativeReplay(replay);
  const group=new T.Group(),meshes=new Map();
  // The native exporter leaves each mesh's vertices in body-local mm and puts
  // the reference world pose/unit conversion in its node matrix. Discard that
  // node matrix, flatten the hierarchy, then apply the recorded core mm/Z-up
  // matrix exactly once. No joint pivots, original parents or FK are reapplied.
  gltfScene.traverse(node=>{
    if(!node.isMesh)return;
    if(!Object.hasOwn(replay.part_bindings,node.name)||meshes.has(node.name))throw Error('Unexpected/duplicate native GLB part: '+node.name);
    const mesh=new T.Mesh(node.geometry,node.material);mesh.name=node.name;mesh.matrixAutoUpdate=false;group.add(mesh);meshes.set(mesh.name,mesh);
  });
  if(meshes.size!==Object.keys(replay.part_bindings).length)throw Error('Native GLB is missing bound parts');
  applyNativeFrame(meshes,replay,0);group.updateMatrixWorld(true);
  return {group,meshes};
}

export function applyNativeFrame(meshes,replay,index){
  if(!Number.isInteger(index)||index<0||index>=replay.times_s.length)throw Error('Invalid recorded frame index');
  for(const [name,mesh] of meshes){mesh.matrix.set(...replay.transforms_mm[replay.part_bindings[name]][index].flat());mesh.matrixWorldNeedsUpdate=true;}
}
