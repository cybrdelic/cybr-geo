export function validateJoints(parts,joints=[]){
  if(!Array.isArray(joints)||joints.length>256)throw Error('Use at most 256 joints');
  const names=new Set(parts.map(p=>p.name)),ids=new Set(),children=new Set(),parent=new Map();
  const vec=v=>Array.isArray(v)&&v.length===3&&v.every(n=>Number.isFinite(n)&&Math.abs(n)<=10000);
  for(const j of joints){
    if(typeof j.name!=='string'||!j.name.length||j.name.length>80||ids.has(j.name)||!names.has(j.child)||children.has(j.child)||j.parent!==null&&!names.has(j.parent)||j.parent===j.child)throw Error('Joint names and child parts must be unique, with valid parent parts');
    if(!['fixed','revolute','prismatic'].includes(j.type)||!vec(j.origin)||!vec(j.axis)||Math.hypot(...j.axis)<1e-8)throw Error('Invalid joint type, pivot or axis');
    if(![j.min,j.max,j.value].every(Number.isFinite)||j.min>j.max||j.value<j.min||j.value>j.max||Math.max(Math.abs(j.min),Math.abs(j.max))>(j.type==='revolute'?36000:10000))throw Error('Invalid joint limits or initial value');
    if(j.type==='fixed'&&(j.min!==0||j.max!==0||j.value!==0||j.drive))throw Error('A fixed attachment cannot have motion or a drive');
    ids.add(j.name);children.add(j.child);parent.set(j.child,j.parent);
  }
  for(const child of children){const seen=new Set();let node=child;while(node!==null&&parent.has(node)){if(seen.has(node))throw Error('Joint hierarchy contains a cycle');seen.add(node);node=parent.get(node);}}
  const byName=new Map(joints.map(j=>[j.name,j]));
  for(const j of joints){if(!j.drive)continue;const d=j.drive,source=byName.get(d.joint);if(!source||source===j||source.type==='fixed'||!Number.isFinite(d.ratio)||d.ratio===0||!Number.isFinite(d.offset))throw Error('Invalid mechanical drive relation');const range=[source.min*d.ratio+d.offset,source.max*d.ratio+d.offset];if(Math.min(...range)<j.min-1e-7||Math.max(...range)>j.max+1e-7)throw Error('Driven joint limits do not cover its input range');const seen=new Set([j.name]);let node=source;while(node){if(seen.has(node.name))throw Error('Mechanical drive contains a cycle');seen.add(node.name);node=node.drive?byName.get(node.drive.joint):null;}}
  return joints.map(j=>({...j,origin:[...j.origin],axis:j.axis.map(n=>n/Math.hypot(...j.axis))}));
}

export function demoValues(definitions,timeSeconds){
  if(!Number.isFinite(timeSeconds))throw Error('Demo time must be finite');
  // Same reference-starting eight-second trajectory as KinematicGraph.demo_values.
  const blend=.5*(1-Math.cos(2*Math.PI*timeSeconds/8)),values={};
  definitions.filter(j=>j.type!=='fixed'&&!j.drive).forEach((joint,index)=>{
    const reference=joint.value,left=reference-joint.min,right=joint.max-reference;
    const direction=left>right||(left===right&&index%2)?-1:1,available=direction<0?left:right;
    const amplitude=Math.min(available*.7,joint.type==='revolute'?22:available*.7);
    values[joint.name]=reference+direction*amplitude*blend;
  });
  return values;
}

export function attachJoints(T,root,parts,joints=[]){
  const definitions=validateJoints(parts,joints),byChild=new Map(definitions.map(j=>[j.child,j])),groups=new Map(),meshes=new Map(parts.map(p=>[p.name,p])),pose={};
  for(const mesh of parts){const j=byChild.get(mesh.name),pivot=j?.origin||[0,0,0],group=new T.Group();group.name='joint_'+mesh.name;group.position.fromArray(pivot);groups.set(mesh.name,group);root.add(group);group.add(mesh);mesh.position.sub(new T.Vector3(...pivot));}
  for(const j of definitions){if(j.parent!==null){const parent=groups.get(j.parent),parentOrigin=byChild.get(j.parent)?.origin||[0,0,0],group=groups.get(j.child);parent.add(group);group.position.fromArray(j.origin).sub(new T.Vector3(...parentOrigin));}}
  function update(){const resolved=new Set();function value(j){if(resolved.has(j.name))return pose[j.name];if(j.drive){const source=definitions.find(x=>x.name===j.drive.joint);pose[j.name]=value(source)*j.drive.ratio+j.drive.offset;}resolved.add(j.name);return pose[j.name]??j.value;}for(const j of definitions){const v=value(j),group=groups.get(j.child);group.position.fromArray(j.origin).sub(new T.Vector3(...(j.parent===null?[0,0,0]:byChild.get(j.parent)?.origin||[0,0,0])));group.quaternion.identity();if(j.type==='revolute')group.quaternion.setFromAxisAngle(new T.Vector3(...j.axis),v*Math.PI/180);else if(j.type==='prismatic')group.position.addScaledVector(new T.Vector3(...j.axis),v);}root.updateMatrixWorld(true);}
  function setJoint(name,value){const j=definitions.find(j=>j.name===name);if(!j||j.drive||!Number.isFinite(value))return;pose[name]=Math.max(j.min,Math.min(j.max,value));update();}
  function reset(){definitions.forEach(j=>pose[j.name]=j.value);update();}
  reset();root.userData.cybrJoints=definitions;
  return {definitions,groups,meshes,pose,setJoint,setPose(values){Object.entries(values).forEach(([name,value])=>setJoint(name,value));},reset};
}
