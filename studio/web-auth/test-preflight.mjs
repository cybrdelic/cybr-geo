import test from 'node:test';
import assert from 'node:assert/strict';
import * as T from 'three';
import {preflightPlan,recipeDigest} from './src/preflight.mjs';
import {attachJoints,demoValues} from './src/joints.mjs';
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
