import * as T from 'three';
import {OrbitControls} from 'three/examples/jsm/controls/OrbitControls.js';
import {GLTFExporter} from 'three/examples/jsm/exporters/GLTFExporter.js';
import {zipSync,strToU8} from 'fflate';
import {attachJoints,demoValues} from './joints.mjs';

const $=id=>document.getElementById(id), host=$('canvas');
const renderer=new T.WebGLRenderer({antialias:true,preserveDrawingBuffer:true});
renderer.setPixelRatio(Math.min(devicePixelRatio,2));renderer.setClearColor('#e5eae3');renderer.outputColorSpace=T.SRGBColorSpace;
host.append(renderer.domElement);
const scene=new T.Scene(),camera=new T.PerspectiveCamera(35,1,.1,10000);camera.up.set(0,0,1);
camera.position.set(600,-750,550);const orbit=new OrbitControls(camera,renderer.domElement);orbit.enableDamping=true;
scene.add(new T.HemisphereLight(0xffffff,0x77806e,2.4));
for(const [position,intensity] of [[[300,-500,700],3], [[-400,200,400],1.7]]){const light=new T.DirectionalLight(0xffffff,intensity);light.position.fromArray(position);scene.add(light);}
const grid=new T.GridHelper(1400,28,0xaabaaa,0xd1dad0);grid.rotation.x=Math.PI/2;grid.position.z=-.2;scene.add(grid);
let root=null,rig=null,result=null,plan=null,session=null,worker=null,animating=false,animationStart=0;
const jointInputs=new Map();
new ResizeObserver(()=>{const w=host.clientWidth,h=host.clientHeight;renderer.setSize(w,h,false);camera.aspect=w/h;camera.updateProjectionMatrix();}).observe(host);
function error(message){$('error').hidden=!message;$('error').textContent=message||'';}
function busy(value){['generate','plan','import'].forEach(id=>$(id).disabled=value);}
function save(name,data,type='application/octet-stream'){const url=URL.createObjectURL(new Blob([data],{type}));const link=document.createElement('a');link.href=url;link.download=name;link.click();setTimeout(()=>URL.revokeObjectURL(url),2000);}
function setAnimating(value){animating=value;animationStart=performance.now();$('animate').textContent=value?'Pause':'Animate';}
function controls(){
  $('joints').replaceChildren();jointInputs.clear();
  for(const j of rig.definitions.filter(j=>j.type!=='fixed'&&!j.drive)){
    const div=document.createElement('div');div.className='joint';const label=document.createElement('label');label.textContent=j.name.replaceAll('_',' ');
    const output=document.createElement('output'),input=document.createElement('input');input.type='range';input.min=j.min;input.max=j.max;input.step=(j.max-j.min)/300;input.value=rig.pose[j.name];label.append(output);
    input.oninput=()=>{setAnimating(false);rig.setJoint(j.name,Number(input.value));updateControls();};div.append(label,input);$('joints').append(div);jointInputs.set(j.name,{input,output,unit:j.type==='revolute'?'°':' mm'});
  }
  updateControls();
}
function updateControls(){for(const [name,c] of jointInputs){c.input.value=rig.pose[name];c.output.textContent=Number(rig.pose[name]).toFixed(1)+c.unit;}}
function display(next){
  if(root){scene.remove(root);root.traverse(o=>{o.geometry?.dispose();if(o.material){if(Array.isArray(o.material))o.material.forEach(m=>m.dispose());else o.material.dispose();}});}
  result=next;root=new T.Group();root.name=result.assembly.name;scene.add(root);
  const meshes=result.assembly.nodes.map(node=>{
    const source=result.assembly.geometries[node.shape],style=result.assembly.materials[node.material];
    const geometry=new T.BufferGeometry();geometry.setAttribute('position',new T.Float32BufferAttribute(source.positions,3));geometry.setAttribute('normal',new T.Float32BufferAttribute(source.normals,3));geometry.setIndex(source.indices);
    const mesh=new T.Mesh(geometry,new T.MeshStandardMaterial({color:new T.Color(...style.color),metalness:style.metalness,roughness:Math.max(.25,style.roughness)}));mesh.name=node.name;return mesh;
  });
  rig=attachJoints(T,root,meshes,result.assembly.joints);controls();
  const box=new T.Box3().setFromObject(root),center=box.getCenter(new T.Vector3()),size=box.getSize(new T.Vector3()),span=Math.max(...size.toArray());
  const vertical=camera.fov*Math.PI/180,fitAngle=Math.min(vertical,2*Math.atan(Math.tan(vertical/2)*camera.aspect)),distance=box.getBoundingSphere(new T.Sphere()).radius/Math.sin(fitAngle/2)*1.12;
  orbit.target.copy(center);camera.position.copy(center).add(new T.Vector3(1.1,-1.5,1.05).normalize().multiplyScalar(distance));camera.near=Math.max(.05,span/2000);camera.far=span*30;camera.updateProjectionMatrix();orbit.update();grid.scale.setScalar(Math.max(1,span/700));
  $('model').textContent=result.assembly.name.replaceAll('_',' ').toUpperCase();$('counts').textContent=`${meshes.length} analytic parts · ${jointInputs.size} independent controls · ${result.assembly.joints.length} joint bindings`;
  $('state').textContent='Constructed · real OpenCascade solids';$('scope').textContent=result.report.construction_bound?'Bound construction plan + prescribed kinematics':'Geometry + authored joint graph';
  $('interpretation').textContent=plan.generation?.interpretation || (plan.construction ? `Selected ${plan.construction.architecture.replaceAll('_',' ')} construction family. Additional requested features are not synthesized.` : plan.summary || 'Custom static CAD recipe.');
  const assumptions=$('assumptions');assumptions.replaceChildren();for(const statement of plan.assumptions||[]){const item=document.createElement('li');item.textContent=statement;assumptions.append(item);}
  ['download','capture'].forEach(id=>$(id).disabled=false);['animate','reset'].forEach(id=>$(id).disabled=jointInputs.size===0);busy(false);error(null);
  // Explicit diagnostic surface for reproducible browser acceptance; no private state.
  window.cybrInspection={model:result.assembly.name,report:result.report,rig,root,renderer,scene,camera};
}
async function construct(nextPlan){
  busy(true);error(null);setAnimating(false);$('state').textContent='Preparing analytic CAD…';plan=nextPlan;
  worker?.terminate();worker=new Worker(new URL('./cad-worker.mjs',import.meta.url),{type:'module'});
  worker.onmessage=({data})=>{if(data.progress)$('state').textContent=data.progress;if(data.result)display(data.result);if(data.error){busy(false);error(data.error);$('state').textContent='Build stopped; last completed assembly retained';}};
  worker.onerror=event=>{busy(false);error(event.message);};worker.postMessage({plan,drawings:$('drawings').checked});
}
async function request(body){
  busy(true);error(null);$('state').textContent='Compiling mechanical intent…';
  try{session ||= await (await fetch('/api/session')).json();const response=await fetch('/api/plan',{method:'POST',headers:{'Content-Type':'application/json','X-Studio-Token':session.csrf},body:JSON.stringify(body)});const value=await response.json();if(!response.ok)throw Error(value.error||'Planning failed');await construct(value.plan);}
  catch(e){busy(false);error(e.message);$('state').textContent='Open a recipe or run the local Studio server';}
}
$('generate').onclick=()=>request({mechanism:{architecture:$('architecture').value,parameters:{scale:Number($('scale').value),radial_clearance_mm:Number($('clearance').value)}}});
$('plan').onclick=()=>request({prompt:$('prompt').value});
$('import').onchange=async()=>{try{const file=$('import').files[0];if(file.size>2e6)throw Error('Recipe exceeds the 2 MB limit');await construct(JSON.parse(await file.text()));}catch(e){error(e.message);}};
$('animate').onclick=()=>setAnimating(!animating);$('reset').onclick=()=>{setAnimating(false);rig.reset();updateControls();};
$('capture').onclick=()=>{renderer.render(scene,camera);renderer.domElement.toBlob(blob=>save(`${result.assembly.name}.png`,blob,'image/png'));};
$('download').onclick=async()=>{
  setAnimating(false);$('download').disabled=true;
  try{
    const clone=root.clone(true),standard=new T.Group();standard.name=result.assembly.name;standard.rotation.x=-Math.PI/2;standard.scale.setScalar(.001);standard.add(clone);
    const glb=await new GLTFExporter().parseAsync(standard,{binary:true});
    const files={...result.files,'selected-pose.glb':new Uint8Array(glb),'pose.json':strToU8(JSON.stringify(rig.pose,null,2)),'EXPORT-SCOPE.txt':strToU8('Original conceptual geometry. STEP/STL/drawings use reference pose. selected-pose.glb uses selected controls in metres/Y-up. Motion is prescribed kinematics; no contact, dynamics, torque or hardware qualification.\n')};
    save(`${result.assembly.name}.zip`,zipSync(files,{level:3}),'application/zip');
  }catch(e){error(e.message);}finally{$('download').disabled=false;}
};
function frame(now){requestAnimationFrame(frame);if(animating&&rig){rig.setPose(demoValues(rig.definitions,(now-animationStart)/1000));updateControls();}orbit.update();renderer.render(scene,camera);}requestAnimationFrame(frame);
// The first run is constructive without requiring a prompt or model inference.
request({mechanism:{architecture:'compact_arm',parameters:{}}});
