import test from 'node:test';
import assert from 'node:assert/strict';
import * as T from 'three';
import {preflightPlan,recipeDigest,objectDigest,modelDigest} from './src/preflight.mjs';
import {simulationSamples,sampleIndexAt,engineeringAccess,acceptedPoseAt} from './src/engineering.mjs';
import {attachJoints,demoValues} from './src/joints.mjs';
import {MotionTask} from './src/motion.mjs';
import {meshCheck} from './src/cad-kernel.mjs';

const part=name=>({name,body:{kind:'box',size:[4,6,8],position:[0,0,0],rotation:[0,0,0]},unions:[],cuts:[],fillet:0,color:[.5,.5,.5],metalness:.7,roughness:.3});
const joint=(name,child,parent=null)=>({name,child,parent,type:'revolute',origin:[0,0,0],axis:[0,0,1],min:-10,max:10,value:0,drive:null});
const fixture=()=>({name:'Example',units:'mm',parts:[part('base'),part('arm')],joints:[{...joint('mount','base'),type:'fixed',min:0,max:0},joint('hinge','arm','base')]});
async function constructed(){const plan=fixture();plan.construction={schema:'cybrgeo-construction/1',recipe_sha256:await recipeDigest(plan)};return plan;}

test('static recipes remain supported',async()=>{const plan=fixture();delete plan.joints;const check=await preflightPlan(plan);assert.equal(check.joints.length,0);assert.equal(check.construction,null);});
test('cycles are rejected before invalid CAD dimensions',async()=>{const plan=fixture();plan.joints[0].parent='arm';plan.parts[0].body.size=[-4,6,8];await assert.rejects(preflightPlan(plan),/cycle/);});
test('drive ranges must cover the entire source interval',async()=>{const plan=fixture();plan.joints[0]={...joint('mount','base'),min:-10,max:10};plan.joints[1].drive={joint:'mount',ratio:1,offset:12};await assert.rejects(preflightPlan(plan),/input range/);});
test('fixed joints cannot pretend to be actuators',async()=>{const plan=fixture();plan.joints[1].drive={joint:'mount',ratio:1,offset:0};await assert.rejects(preflightPlan(plan),/Invalid mechanical drive/);plan.joints[1].drive=null;plan.joints[0].max=1;await assert.rejects(preflightPlan(plan),/fixed attachment/);});
test('modified solids cannot retain their construction identity',async()=>{const plan=await constructed();assert.ok((await preflightPlan(plan)).construction);plan.parts[1].body.size[0]+=.1;await assert.rejects(preflightPlan(plan),/no longer matches/);});
test('modified joint definitions invalidate the same identity',async()=>{const plan=await constructed();plan.joints[1].axis=[0,1,0];await assert.rejects(preflightPlan(plan),/no longer matches/);});
test('large hand-authored collections cannot claim expanded compiler budget',async()=>{const plan=fixture();plan.joints=[];plan.parts=Array.from({length:65},(_,i)=>part('part_'+i));await assert.rejects(preflightPlan(plan),/budget/);});
test('numeric normalization is stable for integer floats and signed zero',async()=>{const a=fixture(),b=fixture();b.parts[0].body.position[0]=-0;assert.equal(await recipeDigest(a),await recipeDigest(b));});
test('hierarchy and a driven follower move from their shared rest pivot',()=>{
  const root=new T.Group(),meshes=['base','arm','tool'].map(name=>{const mesh=new T.Mesh(new T.BoxGeometry(1,1,1));mesh.name=name;return mesh;});
  const joints=[{...joint('base_axis','base'),min:-90,max:90}, {...joint('elbow','arm','base'),origin:[10,0,0],min:-90,max:90}, {...joint('follower','tool','arm'),origin:[20,0,0],drive:{joint:'elbow',ratio:-1,offset:0},min:-90,max:90}];
  const rig=attachJoints(T,root,meshes,joints);rig.setJoint('base_axis',90);rig.setJoint('elbow',30);
  assert.equal(rig.pose.follower,-30);assert.ok(Math.abs(rig.groups.get('arm').getWorldPosition(new T.Vector3()).x)<1e-8);
  const rotation=rig.groups.get('tool').getWorldQuaternion(new T.Quaternion());const axis=new T.Vector3(1,0,0).applyQuaternion(rotation);assert.ok(axis.distanceTo(new T.Vector3(0,1,0))<1e-8);
});
test('CAD pole collapse is removed as a whole face without hiding actual holes',()=>{
  const mesh={vertices:[0,0,0,1,0,0,0,1,0,0,0,1,0,0,0],normals:Array(15).fill(0),triangles:[0,2,1,0,1,3,0,3,2,1,2,3,0,4,1]};
  const check=meshCheck(mesh,'Tetrahedron');assert.equal(check.removed_collapsed_triangles,1);assert.equal(mesh.triangles.length,12);
  mesh.triangles.splice(0,3);assert.throws(()=>meshCheck(mesh,'Open tetrahedron'),/not closed/);
});
test('default demo starts at reference, alternates symmetric controls, and caps rotation',()=>{
  const definitions=[joint('a','a'),joint('b','b'),{...joint('slide','slide'),type:'prismatic',min:0,max:10},{...joint('large','large'),min:-90,max:90}];
  assert.deepEqual(demoValues(definitions,0),{a:0,b:0,slide:0,large:0});
  assert.deepEqual(demoValues(definitions,4),{a:7,b:-7,slide:7,large:-22});
  assert.deepEqual(demoValues(definitions,8),{a:0,b:0,slide:0,large:0});
});

test('load edits invalidate the source identity before CAD',async()=>{
  const plan=await constructed();plan.construction.mechanism={schema:'cybrgeo-mechanism/2',loads:{payload_kg:1}};
  plan.construction.mechanism_sha256=await objectDigest(plan.construction.mechanism);plan.construction.model_sha256=await modelDigest(plan);
  await preflightPlan(plan);plan.construction.mechanism.loads.payload_kg=2;
  await assert.rejects(preflightPlan(plan),/Mechanism source no longer matches/);
});
test('derived physical assumptions cannot retain a model identity',async()=>{
  const plan=await constructed();plan.construction.mechanism={schema:'cybrgeo-mechanism/2'};plan.construction.dynamics={mass_kg:1};
  plan.construction.mechanism_sha256=await objectDigest(plan.construction.mechanism);plan.construction.model_sha256=await modelDigest(plan);
  await preflightPlan(plan);plan.construction.dynamics.mass_kg=3;
  await assert.rejects(preflightPlan(plan),/Engineering inputs no longer match/);
});
test('imported passed flags cannot enable engineering playback',()=>{
  const plan={construction:{model_sha256:'example',mechanism_sha256:'source',recipe_sha256:'geometry'}},response={plan,passed:true,motion_authorization:{allowed:true},clearance:{passed:true,compiled_geometry_sha256:'geometry',...plan.construction},simulation:{backend:'MuJoCo',tracking_passed:true,...plan.construction}};
  assert.equal(engineeringAccess(response,plan).accepted,false);
  assert.equal(engineeringAccess(response,plan,true).accepted,true);
  assert.equal(engineeringAccess(response,{construction:{model_sha256:'different'}},true).bound,false);
  response.clearance.model_sha256='old-motion-source';assert.equal(engineeringAccess(response,plan,true).accepted,false);assert.equal(engineeringAccess(response,plan,true).clearance,null);
  response.simulation.model_sha256='stale';assert.equal(engineeringAccess(response,plan,true).simulation,null);
});
test('simulation replay preserves actual recorded body motion and rejects non-rigid poses',()=>{
  const pose=x=>[[1,0,0,x],[0,1,0,0],[0,0,1,0],[0,0,0,1]];
  const simulation={schema:'cybrgeo.simulation/1',backend:'MuJoCo',samples:[{time_s:0,body_poses_mm:{arm:pose(0)}},{time_s:.05,body_poses_mm:{arm:pose(1.23)}}]};
  const samples=simulationSamples(simulation,['arm']);assert.equal(samples[1].body_poses_mm.arm[0][3],1.23);
  assert.equal(sampleIndexAt(samples,.025),0);assert.equal(sampleIndexAt(samples,.05),1);
  simulation.samples[1].body_poses_mm.arm[0][0]=2;assert.throws(()=>simulationSamples(simulation,['arm']),/Non-rigid/);
});
const motionDocument=()=>({schema:'cybrgeo-motion/1',period_s:12,interpolation:'cosine_out_and_back',waypoints:[{j:2,slide:1},{j:8,slide:3}],actuator_bindings:{motor:'j',jaw:'slide'}});
test('accepted playback evaluates source timing and cannot leave the certified polyline',()=>{
  const motion=new MotionTask(motionDocument()),certificate={passed:true,interpolation:'piecewise_linear_in_independent_joint_coordinates',waypoints:motion.waypoints};
  assert.deepEqual(acceptedPoseAt(certificate,motion,0),{j:2,slide:1});assert.deepEqual(acceptedPoseAt(certificate,motion,6),{j:8,slide:3});
  assert.deepEqual(acceptedPoseAt(certificate,motion,12),{j:2,slide:1});
  assert.throws(()=>acceptedPoseAt({...certificate,passed:false},motion,0),/No accepted/);
  const outside=new MotionTask({...motionDocument(),waypoints:[{j:2,slide:1},{j:8,slide:4}]});
  assert.throws(()=>acceptedPoseAt(certificate,outside,6),/outside the certified/);
});
test('canonical motion retains nonzero starts, analytic rates and requested period',()=>{
  const motion=new MotionTask(motionDocument()),sample=motion.sample(3),omega=2*Math.PI/12;
  assert.ok(Math.abs(sample.values.j-5)<1e-12);assert.equal(sample.velocities.j,3*omega);assert.ok(Math.abs(sample.accelerations.j)<1e-12);
  assert.equal(motion.sample(0).accelerations.j,3*omega*omega);assert.equal(motion.peakRates().j.speed,3*omega);
  const slower=new MotionTask({...motionDocument(),period_s:24});assert.equal(slower.peakRates().j.speed,motion.peakRates().j.speed/2);assert.equal(slower.peakRates().j.acceleration,motion.peakRates().j.acceleration/4);
  assert.throws(()=>new MotionTask({...motionDocument(),period_s:true}),/period/);assert.throws(()=>new MotionTask({...motionDocument(),waypoints:[{j:0},{j:1}]}),/every independent/);
});
test('malformed lowered motion is rejected before CAD geometry or identities',async()=>{
  const plan=fixture();plan.construction={motion:{...motionDocument(),period_s:0}};plan.parts[0].body.size=[-1,1,1];
  await assert.rejects(preflightPlan(plan),/Motion period/);
});
