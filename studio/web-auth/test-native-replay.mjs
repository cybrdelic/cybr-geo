import test from 'node:test';
import assert from 'node:assert/strict';
import * as T from 'three';
import {validateNativeReplay,nativeReplayAccess,nativeFrameAt,nativeMeshes,applyNativeFrame} from './src/native-replay.mjs';
const matrix=x=>[[0,-1,0,x],[1,0,0,200],[0,0,1,300],[0,0,0,1]];
const fixture=()=>({schema:'cybrgeo-native-replay/1',units:'mm',up_axis:'Z',interpolation:'previous_recorded_state',model_sha256:'model',control_sha256:'control',reference_sha256:'reference',times_s:[0,.5,1],part_bindings:{arm:'a',free:'puck'},transforms_mm:{a:[matrix(0),matrix(100),matrix(200)],puck:[matrix(700),matrix(650),matrix(600)]}});
const report=()=>({schema:'cybrgeo-engineering-result/2',model_sha256:'model',control_sha256:'control',simulation:{reference_sha256:'reference'},passed:true,requirements:{passed:true}});
test('held native state and strict rigid matrices',()=>{const r=fixture();validateNativeReplay(r);assert.equal(nativeFrameAt(r,.4999),0);assert.equal(nativeFrameAt(r,.5),1);assert.equal(nativeFrameAt(r,100),2);r.transforms_mm.puck[1][0][0]=2;assert.throws(()=>validateNativeReplay(r),/rigid/);});
test('no imported or stale receipt can supply a passing run',()=>{assert.throws(()=>nativeReplayAccess(report(),fixture()),/bound/);assert.equal(nativeReplayAccess(report(),fixture(),true).passed,true);const bad=report();bad.model_sha256='other';assert.throws(()=>nativeReplayAccess(bad,fixture(),true),/bound/);const failed=report();failed.passed=false;assert.equal(nativeReplayAccess(failed,fixture(),true).passed,false);});
test('flattened native GLB applies actual pose once and leaves free body independent',()=>{
  const r=fixture(),source=new T.Group(),parent=new T.Group();parent.position.set(1000,2000,3000);parent.scale.setScalar(.001);source.add(parent);
  for(const name of ['arm','free']){const mesh=new T.Mesh(new T.BoxGeometry(2,2,2),new T.MeshStandardMaterial());mesh.name=name;mesh.position.set(999,999,999);parent.add(mesh);}
  const {group,meshes}=nativeMeshes(T,source,r);applyNativeFrame(meshes,r,1);group.updateMatrixWorld(true);
  assert.deepEqual(new T.Vector3(1,0,0).applyMatrix4(meshes.get('arm').matrixWorld).toArray(),[100,201,300]);
  assert.deepEqual(new T.Vector3(1,0,0).applyMatrix4(meshes.get('free').matrixWorld).toArray(),[650,201,300]);
});
test('missing native parts and reflected/transposed poses rejected',()=>{const r=fixture();assert.throws(()=>nativeMeshes(T,new T.Group(),r),/missing/);r.transforms_mm.a[0]=[[1,0,0,0],[0,1,0,0],[0,0,-1,0],[0,0,0,1]];assert.throws(()=>validateNativeReplay(r),/Reflected/);});
