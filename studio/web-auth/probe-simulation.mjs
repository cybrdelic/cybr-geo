// Programmatic actual Three.js body-state replay. This does not drive a browser.
import {readFile,writeFile} from 'node:fs/promises';
import {resolve} from 'node:path';
import {createHash} from 'node:crypto';
import * as T from 'three';
import {simulationSamples} from './src/engineering.mjs';
const [fixturePath,simulationPath,assemblyPath,outputPath]=process.argv.slice(2);
if(!outputPath){console.error('Usage: node probe-simulation.mjs NATIVE_FIXTURE.json SIMULATION.json ASSEMBLY.json REPORT.json');process.exit(2);}
const read=async path=>await readFile(resolve(path)),sha=bytes=>createHash('sha256').update(bytes).digest('hex');
const fixture=JSON.parse(await read(fixturePath)),simulationBytes=await read(simulationPath),assemblyBytes=await read(assemblyPath);
if(sha(simulationBytes)!==fixture.simulation_sha256||sha(assemblyBytes)!==fixture.browser_assembly_sha256)throw Error('Replay artifacts changed after native fixture generation');
const simulation=JSON.parse(simulationBytes),assembly=JSON.parse(assemblyBytes),root=new T.Group();
const meshes=new Map(assembly.nodes.map(node=>{
  const geometry=new T.BufferGeometry();geometry.setAttribute('position',new T.Float32BufferAttribute(assembly.geometries[node.shape].positions,3));
  const mesh=new T.Mesh(geometry);mesh.name=node.name;mesh.matrixAutoUpdate=false;root.add(mesh);return [node.name,mesh];
}));
const samples=simulationSamples(simulation,[...meshes.keys()]);
for(const key of ['recipe_sha256','mechanism_sha256','model_sha256'])if(fixture[key]!==simulation[key]||fixture[key]!==assembly.construction[key])throw Error('Replay model identity differs: '+key);
let maximumMatrixError=0,maximumVertexError=0,maximumMotion=0;
for(const expected of fixture.samples){
  const sample=samples[expected.simulation_sample_index];if(sample.time_s!==expected.time_s)throw Error('Replay sample time differs');
  for(const [name,mesh] of meshes)mesh.matrix.set(...sample.body_poses_mm[name].flat());root.updateMatrixWorld(true);
  for(const [name,mesh] of meshes){
    const target=expected.parts[name],matrix=new T.Matrix4().set(...target.matrix_mm.flat());
    maximumMatrixError=Math.max(maximumMatrixError,...matrix.elements.map((value,index)=>Math.abs(value-mesh.matrixWorld.elements[index])));
    target.vertex_indices.forEach((vertexIndex,index)=>{
      const point=new T.Vector3().fromBufferAttribute(mesh.geometry.attributes.position,vertexIndex),world=point.clone().applyMatrix4(mesh.matrixWorld);
      maximumVertexError=Math.max(maximumVertexError,world.distanceTo(new T.Vector3(...target.world_vertices_mm[index])));
      maximumMotion=Math.max(maximumMotion,point.distanceTo(world));
    });
  }
}
if(maximumMatrixError>1e-8||maximumVertexError>1e-7||maximumMotion<.01)throw Error('Three replay differs from native simulated_assembly or has no measurable motion');
const report={schema:'cybrgeo-simulation-replay-evidence/1',backend:simulation.backend,backend_version:simulation.backend_version,
  recipe_sha256:fixture.recipe_sha256,mechanism_sha256:fixture.mechanism_sha256,model_sha256:fixture.model_sha256,
  simulation_sha256:fixture.simulation_sha256,browser_assembly_sha256:fixture.browser_assembly_sha256,
  parts:meshes.size,samples:fixture.samples.length,maximum_matrix_error:maximumMatrixError,maximum_vertex_error_mm:maximumVertexError,maximum_sampled_motion_mm:maximumMotion,passed:true,
  scope:'Actual integrated MuJoCo body matrices and WASM CAD vertices replayed in a flat Three.js group, compared with native simulated_assembly. No duplicate joint pivots, no unit conversion, no live browser UI claim. This checks replay fidelity, not simulation accuracy or hardware qualification.'};
await writeFile(resolve(outputPath),JSON.stringify(report,null,2)+'\n');console.log(JSON.stringify(report,null,2));
