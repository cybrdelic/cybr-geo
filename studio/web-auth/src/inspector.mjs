import * as T from 'three';
import {OrbitControls} from 'three/examples/jsm/controls/OrbitControls.js';
import {GLTFExporter} from 'three/examples/jsm/exporters/GLTFExporter.js';
import {zipSync,strToU8} from 'fflate';
import {attachJoints} from './joints.mjs';
import {MotionTask} from './motion.mjs';
import {engineeringAccess,simulationSamples,sampleIndexAt,acceptedPoseAt} from './engineering.mjs';

const $=id=>document.getElementById(id),host=$('canvas');
let renderer=null,renderFailure='';
try{renderer=new T.WebGLRenderer({antialias:true,preserveDrawingBuffer:true});renderer.setPixelRatio(Math.min(devicePixelRatio,2));renderer.setClearColor('#e5eae3');renderer.outputColorSpace=T.SRGBColorSpace;host.append(renderer.domElement);}
catch(e){renderFailure='WebGL viewport unavailable: '+e.message;}
const scene=new T.Scene(),camera=new T.PerspectiveCamera(35,1,.1,10000);camera.up.set(0,0,1);
camera.position.set(600,-750,550);
const orbit=renderer?new OrbitControls(camera,renderer.domElement):{target:new T.Vector3(),update(){}};
orbit.enableDamping=true;scene.add(new T.HemisphereLight(0xffffff,0x77806e,2.4));
for(const [position,intensity] of [[[300,-500,700],3],[[-400,200,400],1.7]]){const light=new T.DirectionalLight(0xffffff,intensity);light.position.fromArray(position);scene.add(light);}
const grid=new T.GridHelper(1400,28,0xaabaaa,0xd1dad0);grid.rotation.x=Math.PI/2;grid.position.z=-.2;scene.add(grid);
let root=null,rigRoot=null,simulationRoot=null,rig=null,result=null,plan=null,session=null,worker=null;
let engineering=null,access=engineeringAccess(null,null),samples=[],examples=[],motion=null;
let playMode='inspect',playing=false,playStart=0,playOffset=0,currentSample=0;
const jointInputs=new Map(),simulationMeshes=new Map();
new ResizeObserver(()=>{const w=host.clientWidth,h=host.clientHeight;if(w&&h){renderer?.setSize(w,h,false);camera.aspect=w/h;camera.updateProjectionMatrix();}}).observe(host);
function error(message){$('error').hidden=!message;$('error').textContent=message||'';}
function busy(value){for(const id of ['engineer','compile','plan','import','document','examples','applyLoads','payload','gravity'])$(id).disabled=value;}
function save(name,data,type='application/octet-stream'){const url=URL.createObjectURL(new Blob([data],{type})),link=document.createElement('a');link.href=url;link.download=name;link.click();setTimeout(()=>URL.revokeObjectURL(url),2000);}
function stopMotion(){playing=false;$('replay').textContent='Replay MuJoCo';$('demo').textContent='Accepted motion';}
function inspect(){stopMotion();playMode='inspect';if(rigRoot)rigRoot.visible=true;if(simulationRoot)simulationRoot.visible=false;}
function sourceDocument(){const source=JSON.parse($('document').value);if(source.schema!=='cybrgeo-mechanism/2')throw Error('Use a cybrgeo-mechanism/2 source document');return source;}
function syncLoads(){try{const source=sourceDocument();$('payload').value=source.loads?.payload?.mass_kg??'';$('gravity').value=(source.loads?.gravity_mm_s2||[]).join(', ');}catch{}}
function invalidate(){inspect();engineering=null;access=engineeringAccess(null,plan);samples=[];showEvidence();$('scope').textContent='Document changed; run engineering again';$('state').textContent='Displaying the last constructed geometry';}
function setDocument(source,changed=true){$('document').value=JSON.stringify(source,null,2);syncLoads();if(changed)invalidate();}
function updateControls(){if(!rig)return;for(const [name,c] of jointInputs){c.input.value=rig.pose[name];c.output.textContent=Number(rig.pose[name]).toFixed(1)+c.unit;}}
function controls(){
  $('joints').replaceChildren();jointInputs.clear();
  for(const j of rig.definitions.filter(j=>j.type!=='fixed'&&!j.drive)){
    const div=document.createElement('div');div.className='joint';const label=document.createElement('label');label.textContent=j.name.replaceAll('_',' ');
    const output=document.createElement('output'),input=document.createElement('input');input.type='range';input.min=j.min;input.max=j.max;input.step=(j.max-j.min)/300||1;input.value=rig.pose[j.name];label.append(output);
    input.oninput=()=>{inspect();rig.setJoint(j.name,Number(input.value));updateControls();$('motionScope').textContent='Manual inspection pose. This command is not authorized by the motion record.';};
    div.append(label,input);$('joints').append(div);jointInputs.set(j.name,{input,output,unit:j.type==='revolute'?'°':' mm'});
  }updateControls();
}
function textList(element,items){element.replaceChildren();for(const item of items){const li=document.createElement('li');li.textContent=String(item);element.append(li);}}
function showEvidence(){
  const simulation=access.simulation,clearance=access.clearance;
  $('scope').textContent=access.accepted?'Declared engineering criteria passed':access.bound?'Engineering incomplete or criteria failed':'Geometry inspection; no current engineering receipt';
  $('scope').dataset.state=access.accepted?'pass':access.bound?'fail':'none';
  $('clearanceState').textContent='Motion envelope: '+(clearance?(clearance.passed===true?'recorded path and error tube passed.':'unresolved or failed; see diagnostics.'):'not computed.');
  $('simulationState').textContent=simulation?'Dynamics: '+simulation.backend+' '+simulation.backend_version+' · tracking criteria '+(simulation.tracking_passed?'passed.':'failed.'): 'Dynamics: not computed.';
  textList($('runErrors'),[...(engineering?.errors||[]).map(item=>item.stage+': '+item.message),...(access.identityError?[access.identityError]:[])]);
  $('metrics').replaceChildren();
  for(const [name,metric] of Object.entries(simulation?.metrics||{})){
    const row=document.createElement('tr');
    for(const value of [name,Number(metric.max_tracking_error).toPrecision(3)+' '+metric.unit,Number(metric.peak_applied_effort_SI).toPrecision(3)+(metric.unit==='deg'?' N·m':' N'),(100*metric.saturation_fraction).toFixed(1)+'%']){const cell=document.createElement('td');cell.textContent=value;row.append(cell);}
    $('metrics').append(row);
  }
  $('replay').disabled=!samples.length;$('timeline').disabled=!samples.length;
  $('replay').textContent=simulation&&!simulation.tracking_passed?'Replay failed run':'Replay MuJoCo';
  let acceptedPath=false;try{if(access.accepted&&motion){for(const time of [0,motion.period_s/4,motion.period_s/2])acceptedPoseAt(clearance,motion,time);acceptedPath=true;}}catch{}
  $('demo').disabled=!acceptedPath;
  $('motionScope').textContent=samples.length?'MuJoCo replay shows recorded body poses, including tracking and coupling errors. A sampled solver trace does not prove continuous clearance.': 'Manual joint inspection is available. A fresh server run is required for accepted motion or simulated playback.';
  textList($('assumptions'),[...(plan?.assumptions||[]),...(simulation?.assumptions||[]),...(clearance?.limitations||[])]);
}
function display(next,nextPlan,analysis){
  if(root){scene.remove(root);const geometries=new Set(),materials=new Set();root.traverse(o=>{if(o.geometry)geometries.add(o.geometry);if(o.material)(Array.isArray(o.material)?o.material:[o.material]).forEach(m=>materials.add(m));});geometries.forEach(g=>g.dispose());materials.forEach(m=>m.dispose());}
  result=next;plan=nextPlan;root=new T.Group();root.name=result.assembly.name;rigRoot=new T.Group();simulationRoot=new T.Group();simulationRoot.visible=false;root.add(rigRoot,simulationRoot);scene.add(root);simulationMeshes.clear();
  const meshes=result.assembly.nodes.map(node=>{
    const source=result.assembly.geometries[node.shape],style=result.assembly.materials[node.material],geometry=new T.BufferGeometry();
    geometry.setAttribute('position',new T.Float32BufferAttribute(source.positions,3));geometry.setAttribute('normal',new T.Float32BufferAttribute(source.normals,3));geometry.setIndex(source.indices);
    const material=new T.MeshStandardMaterial({color:new T.Color(...style.color),metalness:style.metalness,roughness:Math.max(.25,style.roughness)}),mesh=new T.Mesh(geometry,material);mesh.name=node.name;
    const solved=new T.Mesh(geometry,material);solved.name=node.name;solved.matrixAutoUpdate=false;simulationRoot.add(solved);simulationMeshes.set(node.name,solved);return mesh;
  });
  rig=attachJoints(T,rigRoot,meshes,result.assembly.joints);controls();inspect();
  engineering=analysis;access=engineeringAccess(analysis,plan,!!analysis);samples=[];motion=plan.construction?.motion?new MotionTask(plan.construction.motion):null;
  if(access.simulation){try{samples=simulationSamples(access.simulation,meshes.map(mesh=>mesh.name));}catch(e){access.accepted=false;engineering.errors=[...(engineering.errors||[]),{stage:'playback',message:e.message}];}}
  if(samples.length){$('timeline').max=samples.at(-1).time_s;$('timeline').value=0;currentSample=0;}
  const box=new T.Box3().setFromObject(root),center=box.getCenter(new T.Vector3()),size=box.getSize(new T.Vector3()),span=Math.max(...size.toArray());
  const vertical=camera.fov*Math.PI/180,fitAngle=Math.min(vertical,2*Math.atan(Math.tan(vertical/2)*camera.aspect)),distance=box.getBoundingSphere(new T.Sphere()).radius/Math.sin(fitAngle/2)*1.12;
  orbit.target.copy(center);camera.position.copy(center).add(new T.Vector3(1.1,-1.5,1.05).normalize().multiplyScalar(distance));camera.near=Math.max(.05,span/2000);camera.far=span*30;camera.updateProjectionMatrix();orbit.update();grid.scale.setScalar(Math.max(1,span/700));
  $('model').textContent=result.assembly.name.replaceAll('_',' ').toUpperCase();$('counts').textContent=meshes.length+' analytic parts · '+jointInputs.size+' independent controls · '+result.assembly.joints.length+' joint bindings';
  $('state').textContent='Constructed with OpenCascade';$('interpretation').textContent=plan.generation?.interpretation||plan.construction?.mechanism?.provenance||plan.summary||'Compiled geometry';
  if(plan.construction?.mechanism)setDocument(plan.construction.mechanism,false);
  $('download').disabled=false;$('capture').disabled=!renderer;$('reset').disabled=false;showEvidence();busy(false);error(renderFailure);
  window.cybrInspection={model:result.assembly.name,report:result.report,rig,root,renderer,scene,camera,engineering,samples};
}
function construct(nextPlan,analysis=null){
  busy(true);error(null);inspect();$('state').textContent='Preparing browser CAD from the compiled recipe.';
  worker?.terminate();worker=new Worker(new URL('./cad-worker.mjs',import.meta.url),{type:'module'});
  worker.onmessage=({data})=>{if(data.progress)$('state').textContent=data.progress;if(data.result){try{display(data.result,nextPlan,analysis);}catch(e){busy(false);error(e.message);}}if(data.error){busy(false);error(data.error);$('state').textContent='Build stopped; last completed assembly retained';}};
  worker.onerror=event=>{busy(false);error(event.message);};worker.postMessage({plan:nextPlan,drawings:$('drawings').checked});
}
async function request(body,engineeringRun=false){
  busy(true);error(null);inspect();$('state').textContent=engineeringRun?'Computing native geometry, motion clearance and physics. This can take a few minutes.':'Compiling the mechanism document.';
  try{
    session ||= await (await fetch('/api/session')).json();
    const response=await fetch(engineeringRun?'/api/engineer':'/api/plan',{method:'POST',headers:{'Content-Type':'application/json','X-Studio-Token':session.csrf},body:JSON.stringify(body)}),value=await response.json();
    if(!response.ok)throw Error(value.error||'Engineering request failed');
    if(!value.plan)throw Error('The server returned no compiled plan');
    construct(value.plan,engineeringRun?value:null);
  }catch(e){busy(false);error(e.message);$('state').textContent='Request failed; last completed geometry retained';}
}
function showSimulation(index){
  currentSample=index;const sample=samples[index];rigRoot.visible=false;simulationRoot.visible=true;playMode='simulation';
  for(const [name,mesh] of simulationMeshes)mesh.matrix.set(...sample.body_poses_mm[name].flat());
  root.updateMatrixWorld(true);$('timeline').value=sample.time_s;$('time').textContent=sample.time_s.toFixed(2)+' s';
  $('state').textContent='Recorded MuJoCo state · '+sample.time_s.toFixed(2)+' s';
}
$('document').oninput=()=>{syncLoads();invalidate();};
$('examples').onchange=()=>{const example=examples.find(item=>item.id===$('examples').value);if(example)setDocument(example.source);};
$('applyLoads').onclick=()=>{try{const source=sourceDocument(),tokens=$('gravity').value.split(','),gravity=tokens.map(Number),mass=Number($('payload').value);if(tokens.some(token=>!token.trim())||!$('payload').value.trim()||gravity.length!==3||gravity.some(x=>!Number.isFinite(x))||!Number.isFinite(mass))throw Error('Use a finite payload mass and three comma-separated gravity components');source.loads.payload.mass_kg=mass;source.loads.gravity_mm_s2=gravity;setDocument(source);error(null);}catch(e){error(e.message);}};
$('saveSource').onclick=()=>{try{save('mechanism.json',JSON.stringify(sourceDocument(),null,2),'application/json');}catch(e){error(e.message);}};
$('engineer').onclick=()=>{try{request({mechanism_source:sourceDocument()},true);}catch(e){error(e.message);}};
$('compile').onclick=()=>{try{request({mechanism_source:sourceDocument()});}catch(e){error(e.message);}};
$('plan').onclick=()=>request({prompt:$('prompt').value});
$('import').onchange=async()=>{try{const file=$('import').files[0];if(!file)return;if(file.size>2e6)throw Error('Source exceeds the 2 MB limit');const value=JSON.parse(await file.text());invalidate();if(value.schema==='cybrgeo-mechanism/2'){setDocument(value);$('state').textContent='Source loaded; run engineering to compute evidence';}else if(Array.isArray(value.parts)){construct(value);}else throw Error('Open a mechanism source or exported CAD recipe');}catch(e){error(e.message);}};
$('replay').onclick=()=>{if(playing&&playMode==='simulation'){stopMotion();return;}const time=playMode==='simulation'?samples[currentSample].time_s:0;playOffset=time>=samples.at(-1).time_s?0:time;playStart=performance.now();playing=true;playMode='simulation';$('replay').textContent='Pause replay';showSimulation(sampleIndexAt(samples,playOffset));};
$('demo').onclick=()=>{if(playing&&playMode==='accepted'){stopMotion();return;}inspect();playMode='accepted';playOffset=0;playStart=performance.now();playing=true;$('demo').textContent='Pause motion';$('motionScope').textContent='Following the server-accepted command polyline. This is prescribed kinematics, separate from simulated body motion.';};
$('reset').onclick=()=>{inspect();rig.reset();updateControls();$('timeline').value=0;$('time').textContent='0.00 s';$('state').textContent='Reference geometry pose';};
$('timeline').oninput=()=>{stopMotion();showSimulation(sampleIndexAt(samples,Number($('timeline').value)));};
$('capture').onclick=()=>{renderer.render(scene,camera);renderer.domElement.toBlob(blob=>save(result.assembly.name+'.png',blob,'image/png'));};
$('download').onclick=async()=>{
  stopMotion();$('download').disabled=true;
  try{
    // Capture every input before the asynchronous GLB writer yields to edits.
    const name=result.assembly.name,clone=root.clone(true),standard=new T.Group();standard.name=name;standard.rotation.x=-Math.PI/2;standard.scale.setScalar(.001);standard.add(clone);
    const pose=playMode==='simulation'?{mode:'MuJoCo recorded body state',sample:samples[currentSample]}:{mode:playMode,controls:{...rig.pose}};
    const files={...result.files,'pose.json':strToU8(JSON.stringify(pose,null,2)),'EXPORT-SCOPE.txt':strToU8('STEP/STL/drawings use reference pose. GLB uses the viewed pose in metres/Y-up. Any engineering record applies only to its bound model, load/contact assumptions and tested trajectory. No hardware qualification.\n')};
    if(plan.construction?.mechanism)files['mechanism.json']=strToU8(JSON.stringify(plan.construction.mechanism,null,2));
    if(access.bound&&!access.identityError)files['engineering.json']=strToU8(JSON.stringify(engineering,null,2));
    const glb=await new GLTFExporter().parseAsync(standard,{binary:true});files['selected-pose.glb']=new Uint8Array(glb);
    save(name+'.zip',zipSync(files,{level:3}),'application/zip');
  }catch(e){error(e.message);}finally{$('download').disabled=false;}
};
function frame(now){
  requestAnimationFrame(frame);
  if(playing&&rig){
    const seconds=playOffset+(now-playStart)/1000;
    if(playMode==='simulation'){showSimulation(sampleIndexAt(samples,seconds));if(seconds>=samples.at(-1).time_s)stopMotion();}
    else if(playMode==='accepted'){try{rig.setPose(acceptedPoseAt(access.clearance,motion,seconds));updateControls();$('state').textContent='Accepted source motion · prescribed kinematics';}catch(e){stopMotion();error(e.message);}}
  }
  orbit.update();renderer?.render(scene,camera);
}requestAnimationFrame(frame);
async function start(){
  busy(true);
  try{
    const response=await fetch('/api/examples');if(!response.ok)throw Error('Local example service unavailable');const value=await response.json();examples=value.examples;
    if(!Array.isArray(examples)||!examples.length)throw Error('The server returned no mechanism examples');
    $('examples').replaceChildren();for(const item of examples){const option=document.createElement('option');option.value=item.id;option.textContent=item.source.name||item.id;$('examples').append(option);}
    const selected=examples.find(item=>item.id===value.default_example)||examples[0];$('examples').value=selected.id;setDocument(selected.source,false);
    await request({mechanism_source:selected.source},true);
  }catch(e){busy(false);error(e.message);$('state').textContent='Open a source document or start the local Studio server';}
}start();
