// Summarize executed original-source runs; do not replace their acceptance flags.
import {readFile,writeFile,mkdir} from 'node:fs/promises';
import {resolve,join,dirname} from 'node:path';
import {spawnSync} from 'node:child_process';
import {createHash} from 'node:crypto';
const [sourceDirectory,outputRoot,reportPath]=process.argv.slice(2);
if(!sourceDirectory||!outputRoot||!reportPath)throw Error('Usage: node summarize_baseline.mjs PINNED_SOURCE_CHECKOUT BASELINE_OUTPUT_ROOT REPORT.json');
const source=resolve(sourceDirectory),commit='aabdf8c57a4ee64f77b8a246ad9aca7b51d9cba2';
const git=spawnSync('git',['-C',source,'rev-parse','HEAD'],{encoding:'utf8'}),status=spawnSync('git',['-C',source,'status','--porcelain'],{encoding:'utf8'});
if(git.status!==0||git.stdout.trim()!==commit||status.status!==0||status.stdout.trim())throw Error('The pinned source checkout must remain unmodified');
const sha=bytes=>createHash('sha256').update(bytes).digest('hex'),runs={};
for(const name of ['nominal','friction_loss','no_counterbalance']){
  const relative=`${name}/results/${name}.json`,bytes=await readFile(join(outputRoot,relative));
  runs[name]={report:JSON.parse(bytes),relative,sha256:sha(bytes)};
}
const nominal=runs.nominal.report,loss=runs.friction_loss.report,layout=nominal.interface.flat_frame_layout;
if(JSON.stringify(layout)!==JSON.stringify(loss.interface.flat_frame_layout)||!nominal.interface.units.includes('Y up'))throw Error('Unexpected source frame schema');
for(const run of Object.values(runs))if(JSON.stringify(run.report.metrics.provenance)!==JSON.stringify(nominal.metrics.provenance))throw Error('Scenario source fingerprints differ');
const field=(frame,name)=>frame.slice(...layout[name]);
const nearest=(report,time)=>report.frames.reduce((a,b)=>Math.abs(b[0]-time)<Math.abs(a[0]-time)?b:a);
const distance=(a,b)=>Math.hypot(...a.map((x,index)=>x-b[index]));
let beforeError=0,maximumHeightDifference=0,maximumPuckDivergence=0;
for(const frame of loss.frames){
  const other=nearest(nominal,frame[0]);
  if(Math.abs(frame[0]-other[0])>1e-12)throw Error('Scenario sampling times differ');
  if(frame[0]<=2)beforeError=Math.max(beforeError,...frame.map((value,index)=>Math.abs(value-other[index])));
  if(frame[0]>2&&frame[0]<3.4){
    const a=field(other,'puck_position'),b=field(frame,'puck_position');
    maximumHeightDifference=Math.max(maximumHeightDifference,a[1]-b[1]);
    maximumPuckDivergence=Math.max(maximumPuckDivergence,distance(a,b));
  }
}
const snapshotTimes=[1.9,2,2.2,2.5,2.75,3.4,4.9];
const snapshots=snapshotTimes.map(time=>{const a=nearest(nominal,time),b=nearest(loss,time);return {time_s:time,nominal_puck_position_m:field(a,'puck_position'),friction_loss_puck_position_m:field(b,'puck_position'),nominal_pad_normal_N:field(a,'pad_normal_force'),friction_loss_pad_normal_N:field(b,'pad_normal_force')};});
const effectObserved=beforeError===0&&maximumHeightDifference>.01&&nominal.metrics.placement_B_error_m<.0015&&loss.metrics.placement_B_error_m>.0015;
if(!nominal.metrics.all_checks_pass||!effectObserved||Object.values(runs).some(run=>run.report.metrics.maximum_imposed_workpiece_force!==0))throw Error('The expected original-source nominal/contact-loss comparison was not observed');
const runtimeRoot=join(source,'ARX-6-P5-Engineering','runtime','node_modules','@mujoco','mujoco'),runtime={};
for(const name of ['mujoco.js','mujoco.wasm','package.json'])runtime[name]=sha(await readFile(join(runtimeRoot,name)));
const validator=await readFile(join(source,'ARX-6-P5-Engineering','integration','validate_p5.mjs'));
const report={schema:'cybrgeo-arx-source-baseline/1',source_commit:commit,source_checkout_clean:true,validator_sha256:sha(validator),source_input_sha256:nominal.metrics.provenance,runtime_sha256:runtime,
  scenarios:Object.fromEntries(Object.entries(runs).map(([name,run])=>{const m=run.report.metrics;return [name,{raw_report:run.relative,raw_report_sha256:run.sha256,duration_s:m.duration_s,timestep_s:m.dt_s,options:m.options,solver:m.solver,samples:run.report.frames.length,original_validator_passed:m.all_checks_pass,original_failed_checks:m.checks.filter(check=>!check.passed).map(check=>check.name),placement_B_error_m:m.placement_B_error_m,return_A_error_m:m.return_A_error_m,maximum_imposed_workpiece_force:m.maximum_imposed_workpiece_force}];})),
  contact_loss_comparison:{change_time_s:2,friction_after_scale:.01,maximum_pre_change_state_difference:beforeError,maximum_before_release_puck_height_difference_m:maximumHeightDifference,maximum_before_release_puck_position_divergence_m:maximumPuckDivergence,nominal_B_error_m:nominal.metrics.placement_B_error_m,friction_loss_B_error_m:loss.metrics.placement_B_error_m,effect_observed:effectObserved,snapshots,
    interpretation:'The runs match exactly through 2 s. Reducing configured pair friction to 1% causes the independently simulated puck to leave the pads and fall before the commanded 3.4 s release. Source nominal placement tolerance 1.5 mm is met by nominal and missed by friction_loss. The original friction_loss validator still reports pass because that scenario omits nominal placement checks; its pass flag is preserved above.'},
  scope:'Fresh execution of the original pinned validate_p5.mjs with ARX_VALIDATION_OUT redirected to separate task outputs. These are source-baseline simulations for comparison with a later shared runtime, not validation of that runtime, the separate compact ARX source, or hardware. No workpiece weld, pose command, actuator or applied force was added.'};
await mkdir(dirname(resolve(reportPath)),{recursive:true});await writeFile(reportPath,JSON.stringify(report,null,2)+'\n');console.log(JSON.stringify({nominal_passed:nominal.metrics.all_checks_pass,friction_loss_original_passed:loss.metrics.all_checks_pass,friction_loss_B_error_m:loss.metrics.placement_B_error_m,maximum_before_release_height_difference_m:maximumHeightDifference,pre_change_error:beforeError,effectObserved},null,2));
