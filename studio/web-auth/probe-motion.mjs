// Compare the browser rig on exported CAD meshes with native solver transforms.
// Generate the input with make-motion-fixture.py in this directory.
import {readFile,writeFile} from 'node:fs/promises';
import {resolve,join} from 'node:path';
import * as T from 'three';
import {attachJoints} from './src/joints.mjs';
import {MotionTask} from './src/motion.mjs';
const [fixturePath,artifactRoot,reportPath]=process.argv.slice(2);
if(!fixturePath||!artifactRoot||!reportPath){console.error('Usage: node probe-motion.mjs FIXTURE.json ARTIFACT_ROOT REPORT.json');process.exit(2);}
const fixture=JSON.parse(await readFile(resolve(fixturePath),'utf8')),results=[];
for(const model of fixture.models){
  const assembly=JSON.parse(await readFile(join(resolve(artifactRoot),'browser-'+model.architecture,'assembly.json'),'utf8'));
  if(!model.recipe_sha256||assembly.construction?.recipe_sha256!==model.recipe_sha256)throw Error(`${model.architecture}: native fixture and CAD artifact recipe identities differ`);
  for(const key of ['mechanism_sha256','model_sha256'])if((model[key]||null)!==(assembly.construction?.[key]||null))throw Error(`${model.architecture}: native fixture and CAD artifact ${key} differ`);
  const root=new T.Group(),meshes=assembly.nodes.map(node=>{const source=assembly.geometries[node.shape],geometry=new T.BufferGeometry();geometry.setAttribute('position',new T.Float32BufferAttribute(source.positions,3));const mesh=new T.Mesh(geometry);mesh.name=node.name;return mesh;});
  const rig=attachJoints(T,root,meshes,assembly.joints),motion=new MotionTask(assembly.construction?.motion||model.motion);let maxError=0,maxMotion=0,maxCommandError=0;
  for(const [name,rates] of Object.entries(motion.peakRates()))for(const [key,value] of Object.entries(rates))if(Math.abs(value-model.peak_rates[name][key])>1e-10)throw Error(`${model.architecture}: native/browser peak ${key} differs`);
  for(const sample of model.samples){
    const command=motion.sample(sample.time_seconds),commands=command.values;
    for(const quantity of ['values','velocities','accelerations'])for(const [name,value] of Object.entries(command[quantity]))maxCommandError=Math.max(maxCommandError,Math.abs(value-sample[quantity][name]));
    if(maxCommandError>1e-10)throw Error(`${model.architecture}: native/browser motion task differs at ${sample.time_seconds}s`);
    rig.reset();rig.setPose(commands);
    for(const mesh of meshes){
      const matrix=sample.transforms[mesh.name],expected=new T.Matrix4().set(...matrix.flat()),actual=mesh.matrixWorld;
      maxError=Math.max(maxError,...actual.elements.map((value,i)=>Math.abs(value-expected.elements[i])));
      // Check actual artifact vertices too, not only empty Three groups.
      const points=mesh.geometry.attributes.position;
      for(const index of [0,Math.floor(points.count/2),points.count-1]){
        const rest=new T.Vector3().fromBufferAttribute(points,index),native=rest.clone().applyMatrix4(expected),browser=rest.clone().applyMatrix4(actual);
        maxError=Math.max(maxError,native.distanceTo(browser));maxMotion=Math.max(maxMotion,rest.distanceTo(browser));
      }
    }
  }
  if(maxError>1e-8||maxMotion<1)throw Error(`${model.architecture}: native/browser mismatch ${maxError}mm or no observable motion (${maxMotion}mm)`);
  results.push({architecture:model.architecture,recipe_sha256:model.recipe_sha256,mechanism_sha256:model.mechanism_sha256,model_sha256:model.model_sha256,parts:meshes.length,samples:model.samples.length,motion_period_s:motion.period_s,maximum_command_derivative_error:maxCommandError,independent_controls:rig.definitions.filter(j=>j.type!=='fixed'&&!j.drive).length,maximum_transform_or_vertex_error:maxError,maximum_sampled_vertex_motion_mm:maxMotion,passed:true});
}
const report={scope:'Cross-language prescribed-kinematic parity on vertices from actual WASM CAD exports; no live browser UI, contact dynamics or hardware qualification.',results};
await writeFile(resolve(reportPath),JSON.stringify(report,null,2));console.log(JSON.stringify(report,null,2));
