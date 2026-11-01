import {readFile,writeFile} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import assert from 'node:assert/strict';
import * as T from 'three';
import {GLTFLoader} from 'three/examples/jsm/loaders/GLTFLoader.js';
import {nativeReplayAccess,nativeMeshes,applyNativeFrame,nativeFrameAt} from './src/native-replay.mjs';

const [fixturePath,glbPath,outputPath]=process.argv.slice(2);
if(!outputPath)throw Error('Usage: node probe-native-replay.mjs fixture.json assembly.glb receipt.json');
const fixture=JSON.parse(await readFile(fixturePath,'utf8')),bytes=await readFile(glbPath),replay=fixture.replay;
const buffer=bytes.buffer.slice(bytes.byteOffset,bytes.byteOffset+bytes.byteLength);
const parsed=await new GLTFLoader().parseAsync(buffer,'');
const gate=nativeReplayAccess(fixture.report,replay,true);
assert.throws(()=>nativeReplayAccess(fixture.report,replay,false));
const failed=structuredClone(fixture.report);failed.passed=false;
assert.equal(nativeReplayAccess(failed,replay,true).passed,false);
const stale=structuredClone(replay);stale.model_sha256='stale';assert.throws(()=>nativeReplayAccess(fixture.report,stale,true));
const {group,meshes}=nativeMeshes(T,parsed.scene,replay);
let maximum=0,comparisons=0;
for(const sample of fixture.expected){
  applyNativeFrame(meshes,replay,sample.frame_index);group.updateMatrixWorld(true);
  for(const part of sample.parts){
    const mesh=meshes.get(part.name);assert.ok(mesh,'missing '+part.name);
    for(let i=0;i<part.vertex_indices.length;i++){
      const actual=new T.Vector3().fromBufferAttribute(mesh.geometry.attributes.position,part.vertex_indices[i]).applyMatrix4(mesh.matrixWorld).toArray();
      for(let axis=0;axis<3;axis++)maximum=Math.max(maximum,Math.abs(actual[axis]-part.world_vertices_mm[i][axis]));
      comparisons++;
    }
  }
}
assert.ok(maximum<1e-4,'world vertex error '+maximum+' mm');
assert.equal(nativeFrameAt(replay,(replay.times_s[0]+replay.times_s[1])/2),0);
const receipt={schema:'cybrgeo-native-three-proof/1',passed:true,task_criteria_passed:gate.passed,
  model_sha256:replay.model_sha256,control_sha256:replay.control_sha256,reference_sha256:replay.reference_sha256,
  trace_file_sha256:fixture.trace_file_sha256,glb_sha256:createHash('sha256').update(bytes).digest('hex'),
  part_count:meshes.size,sampled_times_s:fixture.expected.map(s=>s.time_s),compared_vertices:comparisons,
  max_vertex_error_mm:maximum,tolerance_mm:1e-4,held_state_policy:true,
  forged_import_rejected:true,stale_model_rejected:true,failed_run_not_passing:true,
  scope:'Programmatic Three.js GLTFLoader and world-vertex parity against actual integrated SI body poses. No live browser or hardware verification.'};
await writeFile(outputPath,JSON.stringify(receipt,null,2)+'\n');console.log(JSON.stringify(receipt,null,2));
