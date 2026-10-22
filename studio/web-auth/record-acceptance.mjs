// Compact reproducible receipt for the actual CAD and motion probes.
import {readFile,writeFile,mkdir} from 'node:fs/promises';
import {resolve,join,dirname} from 'node:path';
import {fileURLToPath} from 'node:url';
import {createHash} from 'node:crypto';
const [inputRoot,artifactRoot,motionReportPath,outputPath]=process.argv.slice(2);
if(!inputRoot||!artifactRoot||!motionReportPath||!outputPath){
  console.error('Usage: node record-acceptance.mjs RECIPE_DIRECTORY ARTIFACT_ROOT MOTION_REPORT.json OUTPUT.json');process.exit(2);
}
const sha=bytes=>createHash('sha256').update(bytes).digest('hex');
const readJSON=async path=>JSON.parse(await readFile(path,'utf8'));
const motion=await readJSON(resolve(motionReportPath)),models=[];
for(const architecture of ['compact_arm','belt_arm','inspection_head']){
  const recipe=await readJSON(join(resolve(inputRoot),architecture+'.json'));
  const reportPath=join(resolve(artifactRoot),'browser-'+architecture,'browser-acceptance.json');
  const report=await readJSON(reportPath),sample=motion.results.find(item=>item.architecture===architecture);
  const exportedRecipe=await readJSON(join(resolve(artifactRoot),'browser-'+architecture,'recipe.json'));
  if(recipe.construction.recipe_sha256!==exportedRecipe.construction.recipe_sha256)throw Error('Stale CAD artifacts for '+architecture);
  if(!sample?.passed||sample.recipe_sha256!==recipe.construction.recipe_sha256||sample.parts!==recipe.parts.length||report.parts!==recipe.parts.length)throw Error('Mismatched or failed evidence for '+architecture);
  if(!report.checks.every(c=>c.analytic_valid&&c.finite&&c.valid_indices&&c.watertight&&c.consistent_winding))throw Error('Failed CAD check for '+architecture);
  models.push({architecture,recipe_sha256:recipe.construction.recipe_sha256,parts:report.parts,triangles:report.triangles,
    all_parts_valid_closed_solids:true,construction_bound:report.construction_bound,drawings:report.drawings,
    step_readback:report.step_readback,stl_binary_records:report.stl_binary_records,motion:sample,
    full_cad_report_sha256:sha(await readFile(reportPath)),export_sha256:Object.fromEntries(
      Object.entries(report.files_sha256).filter(([name])=>['assembly.step','assembly.json','recipe.json','construction.json','validation.json','whiteprint.svg'].includes(name)))});
}
const repoRoot=fileURLToPath(new URL('../../',import.meta.url));
const sources=['src/mechanism_lab/construction.py','src/mechanism_lab/constructed.py',
  'studio/web-auth/src/cad-kernel.mjs','studio/web-auth/src/preflight.mjs','studio/web-auth/src/joints.mjs',
  'studio/web-auth/src/inspector.mjs','studio/web-auth/src/cad-worker.mjs','studio/web-auth/build.mjs',
  'studio/web-auth/probe-cad.mjs','studio/web-auth/probe-motion.mjs','studio/web-auth/make-motion-fixture.py',
  'studio/web-auth/record-acceptance.mjs','studio/web-auth/package-lock.json'];
const sourceHashes={};for(const source of sources)sourceHashes[source]=sha(await readFile(join(repoRoot,source)));
const evidence={schema:'cybrgeo-browser-evidence/1',recorded_at:new Date().toISOString(),runtime:{node:process.version,platform:process.platform},
  scope:'Actual Replicad/OpenCascade WebAssembly construction and export readback in Node.js; actual exported mesh vertices and Three.js joint transforms compared with native prescribed kinematics.',
  limits:['No live browser UI verification: computer-use browser inventory was empty.','No contact dynamics, strength, fatigue, thermal or hardware qualification.','These original mechanism families are not reproductions or validation of IRIS-09 or either ARX-6 source.','Only recorded recipes and trajectory samples are covered.'],
  source_sha256:sourceHashes,motion_report_sha256:sha(await readFile(resolve(motionReportPath))),models};
await mkdir(dirname(resolve(outputPath)),{recursive:true});await writeFile(resolve(outputPath),JSON.stringify(evidence,null,2)+'\n');
console.log('Recorded '+models.length+' actual CAD and motion probe results in '+outputPath);
