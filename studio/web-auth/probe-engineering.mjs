// Verify the UI gate with completed native records, without rerunning a solver.
import {readFile,writeFile} from 'node:fs/promises';
import {join} from 'node:path';
import {createHash} from 'node:crypto';
import {engineeringAccess,simulationSamples,acceptedPoseAt} from './src/engineering.mjs';
import {preflightPlan} from './src/preflight.mjs';
import {MotionTask} from './src/motion.mjs';
const [engineeringPath,simulationPath,recipePath,outputPath,beltDirectory]=process.argv.slice(2);
if(!engineeringPath||!simulationPath||!recipePath||!outputPath)throw Error('Usage: node probe-engineering.mjs ENGINEERING.json SIMULATION.json RECIPE.json REPORT.json [BELT_DIAGNOSTIC_DIRECTORY]');
const sha=bytes=>createHash('sha256').update(bytes).digest('hex');
const [engineeringBytes,simulationBytes,recipeBytes]=await Promise.all([engineeringPath,simulationPath,recipePath].map(path=>readFile(path)));
const response={...JSON.parse(engineeringBytes),simulation:JSON.parse(simulationBytes)},plan=JSON.parse(recipeBytes);
await preflightPlan(plan);
const access=engineeringAccess(response,plan,true);
if(!access.accepted||access.identityError)throw Error('Completed current engineering records did not enable accepted playback');
const samples=simulationSamples(access.simulation,plan.parts.map(part=>part.name)),motion=new MotionTask(plan.construction.motion);
let maximumCommandError=0;
for(const sample of samples){
  const canonical=acceptedPoseAt(access.clearance,motion,sample.time_s);
  for(const [name,value] of Object.entries(canonical))maximumCommandError=Math.max(maximumCommandError,Math.abs(value-sample.requested_values[name]));
}
if(!Number.isFinite(maximumCommandError)||maximumCommandError>1e-9)throw Error('Recorded requests differ from canonical source motion');
const checks={current_record_accepted:true,all_recorded_body_poses_valid:true,
  imported_passed_flags_rejected:!engineeringAccess(response,plan).accepted,
  failed_run_rejected:!engineeringAccess({...response,passed:false},plan,true).accepted,
  denied_motion_authorization_rejected:!engineeringAccess({...response,motion_authorization:{allowed:false}},plan,true).accepted,
  failed_tracking_rejected:!engineeringAccess({...response,simulation:{...response.simulation,tracking_passed:false}},plan,true).accepted,
  stale_clearance_source_rejected:!engineeringAccess({...response,clearance:{...response.clearance,model_sha256:'stale'}},plan,true).accepted};
if(Object.values(checks).some(value=>!value))throw Error('An invalid engineering state enabled accepted playback');
let beltDiagnostic=null;
if(beltDirectory){
  const [beltBytes,beltRecipeBytes]=await Promise.all(['simulation.json','recipe.json'].map(name=>readFile(join(beltDirectory,name))));
  const simulation=JSON.parse(beltBytes),beltPlan=JSON.parse(beltRecipeBytes);
  await preflightPlan(beltPlan);
  // A real solver trace alone has no complete engineering authorization,
  // whether or not its tracking criteria passed.
  const diagnostic=engineeringAccess({plan:beltPlan,simulation,clearance:null,passed:false,motion_authorization:{allowed:false}},beltPlan,true);
  const diagnosticSamples=simulationSamples(diagnostic.simulation,beltPlan.parts.map(part=>part.name));
  if(diagnostic.accepted)throw Error('A belt trace without completed engineering evidence enabled accepted motion');
  beltDiagnostic={model_sha256:beltPlan.construction.model_sha256,simulation_sha256:sha(beltBytes),recipe_sha256_file:sha(beltRecipeBytes),parts:beltPlan.parts.length,samples:diagnosticSamples.length,accepted:false,diagnostic_replay_available:true,tracking_passed:simulation.tracking_passed,operating_limits_passed:simulation.operating_limits_passed,missing_evidence:'No continuous-clearance certificate or complete engineering authorization supplied to this diagnostic case'};
}
const report={schema:'cybrgeo-engineering-gate-evidence/1',model_sha256:plan.construction.model_sha256,mechanism_sha256:plan.construction.mechanism_sha256,recipe_sha256:plan.construction.recipe_sha256,
  engineering_sha256:sha(engineeringBytes),simulation_sha256:sha(simulationBytes),recipe_sha256_file:sha(recipeBytes),parts:plan.parts.length,samples:samples.length,maximum_canonical_command_error:maximumCommandError,checks,belt_diagnostic:beltDiagnostic,passed:true,
  scope:'Programmatic execution of the actual browser engineering gate on completed source-bound native pipeline records and the same compiled browser recipe. Includes canonical source commands at every exported simulation time, failed-state and imported-flag rejection. No browser UI execution or solver rerun.'};
await writeFile(outputPath,JSON.stringify(report,null,2)+'\n');console.log(JSON.stringify(report,null,2));
