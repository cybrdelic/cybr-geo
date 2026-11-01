// Real WebAssembly CAD acceptance. Inputs are emitted by lab mechanism.
import {readFile,mkdir,writeFile} from 'node:fs/promises';
import {resolve,join,dirname} from 'node:path';
import {createHash} from 'node:crypto';
import {initialize,buildCad} from './src/cad-kernel.mjs';
import {importSTEP,measureVolume} from 'replicad';
const [input,output,...flags]=process.argv.slice(2);
if(!input||!output){console.error('Usage: node probe-cad.mjs RECIPE.json OUTPUT_DIRECTORY [--drawings]');process.exit(2);}
const plan=JSON.parse(await readFile(resolve(input),'utf8'));
await initialize({wasmBinary:await readFile(new URL('./node_modules/replicad-opencascadejs/dist/replicad_single.wasm',import.meta.url))});
const started=performance.now();
const result=await buildCad(plan,message=>{if(flags.includes('--verbose'))console.error(message);},{drawings:flags.includes('--drawings')});
// Reopen the delivered STEP bytes instead of relying only on the source B-reps.
const reopened=await importSTEP(new Blob([result.files['assembly.step']]));
let stepReadback;
try{
  const solids=reopened.solids,solidCount=solids.length;solids.forEach(shape=>shape.delete());
  const volume=measureVolume(reopened),expected=result.report.checks.reduce((n,c)=>n+c.volume_mm3,0),relativeError=Math.abs(volume-expected)/expected;
  // STEP roundtrips change analytic surface integration at sub-ppm scale even
  // when topology is identical. Record the actual error and a 10-ppm bound.
  const volumeTolerance=1e-5;
  if(solidCount!==plan.parts.length||!Number.isFinite(volume)||volume<=0||relativeError>volumeTolerance)throw Error(`Exported STEP readback mismatch: solids=${solidCount}/${plan.parts.length}, volume=${volume}/${expected}mm3, relative_error=${relativeError}`);
  stepReadback={solid_count:solidCount,volume_mm3:volume,relative_volume_error:relativeError,relative_volume_tolerance:volumeTolerance};
}finally{reopened.delete();}
let stlFiles=0,stlTriangles=0;
for(const [name,bytes] of Object.entries(result.files).filter(([name])=>name.endsWith('.stl'))){
  if(bytes.length<84)throw Error(`${name}: incomplete binary STL`);
  const data=new DataView(bytes.buffer,bytes.byteOffset,bytes.byteLength),count=data.getUint32(80,true);
  if(!count||bytes.length!==84+count*50)throw Error(`${name}: STL triangle record count differs from bytes`);
  for(let face=0;face<count;face++)for(let coordinate=0;coordinate<12;coordinate++)if(!Number.isFinite(data.getFloat32(84+face*50+coordinate*4,true)))throw Error(`${name}: non-finite STL record`);
  stlFiles++;stlTriangles+=count;
}
if(stlFiles!==plan.parts.length)throw Error('Missing exported STL parts');
await mkdir(resolve(output),{recursive:true});
for(const [name,bytes] of Object.entries(result.files)){const target=join(resolve(output),name);await mkdir(dirname(target),{recursive:true});await writeFile(target,bytes);}
const report={...result.report,triangles:result.assembly.geometries.reduce((n,g)=>n+g.indices.length/3,0),step_readback:stepReadback,stl_binary_records:{files:stlFiles,triangles:stlTriangles,finite:true,declared_lengths_match:true},files_sha256:Object.fromEntries(Object.entries(result.files).map(([name,bytes])=>[name,createHash('sha256').update(bytes).digest('hex')])),elapsed_seconds:(performance.now()-started)/1000,drawings:flags.includes('--drawings')};
await writeFile(join(resolve(output),'browser-acceptance.json'),JSON.stringify(report,null,2));
console.log(JSON.stringify({model:plan.name,parts:report.parts,triangles:report.triangles,construction_bound:report.construction_bound,all_solids_valid:report.checks.every(c=>c.analytic_valid&&c.watertight),seconds:report.elapsed_seconds},null,2));
