// Compare the example port directly with the pinned original JavaScript.
import {readFile,writeFile,mkdir} from 'node:fs/promises';
import {resolve,join,dirname} from 'node:path';
import {fileURLToPath,pathToFileURL} from 'node:url';
import {spawnSync} from 'node:child_process';
import {createHash} from 'node:crypto';
const [sourceDirectory,python,outputPath]=process.argv.slice(2);
if(!sourceDirectory||!python||!outputPath)throw Error('Usage: node verify_task.mjs PINNED_SOURCE_CHECKOUT PYTHON REPORT.json');
const source=resolve(sourceDirectory),integration=join(source,'ARX-6-P5-Engineering','integration');
const git=spawnSync('git',['-C',source,'rev-parse','HEAD'],{encoding:'utf8'});
const commit='aabdf8c57a4ee64f77b8a246ad9aca7b51d9cba2';
if(git.status!==0||git.stdout.trim()!==commit)throw Error('Source checkout is not the pinned ARX belt commit');
const controllerPath=join(integration,'controller.mjs'),parametersPath=join(integration,'parameters.json');
const [controllerBytes,parametersBytes,safetyBytes]=await Promise.all([controllerPath,parametersPath,join(integration,'safety.mjs')].map(path=>readFile(path)));
const parameters=JSON.parse(parametersBytes),{reference,referenceDerivatives}=await import(pathToFileURL(controllerPath));
const times=new Set(Array.from({length:24001},(_,index)=>index/2400));
const boundaries=[];
for(const cycle of [-10,0,10])for(const half of [0,5])for(const boundary of [0,.40,.65,1,1.65,2.75,3.4,3.75,4.45,5]){
  const time=cycle+half+boundary;boundaries.push(time);
  for(const offset of [-.00025,-1e-9,0,1e-9,.00025])times.add(time+offset);
}
for(const time of [-25.125,-10,-.125,10.123456789,27.123456789,100.375])times.add(time);
const sampleTimes=[...times].sort((a,b)=>a-b),taskPath=fileURLToPath(new URL('task.py',import.meta.url));
const runner=`import importlib.util,json,sys\nspec=importlib.util.spec_from_file_location('arx_belt_example_task',sys.argv[1])\nmodule=importlib.util.module_from_spec(spec)\nspec.loader.exec_module(module)\ndata=json.load(sys.stdin)\njson.dump([module.reference(t,data['parameters']) for t in data['times']],sys.stdout,allow_nan=False)\n`;
const pythonRun=spawnSync(python,['-c',runner,taskPath],{input:JSON.stringify({parameters,times:sampleTimes}),encoding:'utf8',maxBuffer:64*1024*1024,env:{...process.env,PYTHONDONTWRITEBYTECODE:'1'}});
if(pythonRun.status!==0)throw Error('Python reference failed: '+pythonRun.stderr);
const actual=JSON.parse(pythonRun.stdout),maxima={q:0,qd:0,qdd:0,gap:0},worst={};
const tolerances={q:2e-13,qd:1e-9,qdd:5e-6,gap:0};
if(actual.length!==sampleTimes.length)throw Error('Python reference sample count differs');
for(let index=0;index<sampleTimes.length;index++){
  const time=sampleTimes[index],expected=referenceDerivatives(time,parameters),position=reference(time,parameters),got=actual[index];
  if(got.stage!==expected.stage||got.stage!==position.stage)throw Error('Stage mismatch at '+time);
  for(const key of ['q','qd','qdd','gap']){
    const left=key==='gap'?[got[key]]:got[key],right=key==='gap'?[expected[key]]:expected[key];
    if(left.length!==right.length)throw Error('Output dimensionality mismatch');
    for(let joint=0;joint<left.length;joint++){
      const error=Math.abs(left[joint]-right[joint]);
      if(!Number.isFinite(error))throw Error('Nonfinite comparison at '+time);
      if(error>maxima[key]){maxima[key]=error;worst[key]={time_s:time,joint_index:joint,python:left[joint],javascript:right[joint]};}
      if(error>tolerances[key])throw Error(`${key} mismatch at ${time}s joint ${joint}: ${error} > ${tolerances[key]}`);
    }
  }
}
const sha=bytes=>createHash('sha256').update(bytes).digest('hex');
const report={schema:'cybrgeo-arx-reference-parity/1',source_commit:commit,source_files:{controller_sha256:sha(controllerBytes),parameters_sha256:sha(parametersBytes),safety_sha256:sha(safetyBytes)},task_sha256:sha(await readFile(taskPath)),samples:sampleTimes.length,integration_grid:{start_s:0,end_s:10,hz:2400,samples:24001},boundary_checks:{unique_boundaries:[...new Set(boundaries)].sort((a,b)=>a-b),offsets_s:[-.00025,-1e-9,0,1e-9,.00025]},maximum_absolute_error:maxima,absolute_tolerance:tolerances,worst,units:{q:'rad',qd:'rad/s',qdd:'rad/s^2',gap:'m (half-gap)',time:'s'},passed:true,scope:'Source-math parity against the original pinned controller at every nominal integration-grid time, phase/gap boundaries and arbitrary negative/later-cycle times. Finite-difference angle branches are preserved; numerical tolerances accommodate cross-language libm rounding amplified by the original second-difference stencil. This is reference-task parity, not integrated motion, contact success or hardware qualification.'};
await mkdir(dirname(resolve(outputPath)),{recursive:true});await writeFile(outputPath,JSON.stringify(report,null,2)+'\n');console.log(JSON.stringify(report,null,2));
