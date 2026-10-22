// Compare the browser rig on exported CAD meshes with native solver transforms.
// Generate the input with make-motion-fixture.py in this directory.
import {readFile,writeFile} from 'node:fs/promises';
import {resolve,join} from 'node:path';
import * as T from 'three';
import {attachJoints,demoValues} from './src/joints.mjs';
const [fixturePath,artifactRoot,reportPath]=process.argv.slice(2);
if(!fixturePath||!artifactRoot||!reportPath){console.error('Usage: node probe-motion.mjs FIXTURE.json ARTIFACT_ROOT REPORT.json');process.exit(2);}
const fixture=JSON.parse(await readFile(resolve(fixturePath),'utf8')),results=[];
for(const model of fixture.models){
  const assembly=JSON.parse(await readFile(join(resolve(artifactRoot),'browser-'+model.architecture,'assembly.json'),'utf8'));
  if(!model.recipe_sha256||assembly.construction?.recipe_sha256!==model.recipe_sha256)throw Error(`${model.architecture}: native fixture and CAD artifact recipe identities differ`);
  const root=new T.Group(),meshes=assembly.nodes.map(node=>{const source=assembly.geometries[node.shape],geometry=new T.BufferGeometry();geometry.setAttribute('position',new T.Float32BufferAttribute(source.positions,3));const mesh=new T.Mesh(geometry);mesh.name=node.name;return mesh;});
  const rig=attachJoints(T,root,meshes,assembly.joints);let maxError=0,maxMotion=0;
  for(const sample of model.samples){
    const commands=demoValues(rig.definitions,sample.time_seconds);
    for(const [name,value] of Object.entries(commands))if(Math.abs(value-sample.values[name])>1e-12)throw Error(`${model.architecture}: native/browser demo trajectory differs at ${sample.time_seconds}s for ${name}`);
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
  results.push({architecture:model.architecture,recipe_sha256:model.recipe_sha256,parts:meshes.length,samples:model.samples.length,independent_controls:rig.definitions.filter(j=>j.type!=='fixed'&&!j.drive).length,maximum_transform_or_vertex_error:maxError,maximum_sampled_vertex_motion_mm:maxMotion,passed:true});
}
const report={scope:'Cross-language prescribed-kinematic parity on vertices from actual WASM CAD exports; no live browser UI, contact dynamics or hardware qualification.',results};
await writeFile(resolve(reportPath),JSON.stringify(report,null,2));console.log(JSON.stringify(report,null,2));
