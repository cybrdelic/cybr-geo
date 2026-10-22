// Browser implementation of Studio's existing millimetre JSON CAD contract.
// Geometry remains analytic OpenCascade B-rep, compiled to WebAssembly.
import initOpenCascade from 'replicad-opencascadejs';
import {setOC,makeBox,makeCylinder,makeSphere,makeCompound,measureVolume,exportSTEP,drawProjection,ProjectionCamera} from 'replicad';
import {preflightPlan} from './preflight.mjs';
let oc;
export async function initialize(options={}){if(!oc){oc=await initOpenCascade(options);setOC(oc);}return oc;}
const vector=(value,positive=false)=>{if(!Array.isArray(value)||value.length!==3||value.some(n=>!Number.isFinite(n)||Math.abs(n)>10000||(positive&&n<=0)))throw Error('Expected three finite dimensions in millimetres, within 10,000 mm.');return value;};
function primitive(spec){
  const size=vector(spec.size,true),position=vector(spec.position),rotation=vector(spec.rotation);let shape;
  if(spec.kind==='box')shape=makeBox(size.map(n=>-n/2),size.map(n=>n/2));
  else if(spec.kind==='cylinder')shape=makeCylinder(size[0]/2,size[2],[0,0,-size[2]/2],[0,0,1]);
  else if(spec.kind==='sphere')shape=makeSphere(size[0]/2);
  else throw Error('Unsupported CAD primitive: '+spec.kind);
  rotation.forEach((angle,index)=>{if(angle)shape=shape.rotate(angle,[0,0,0],[Number(index===0),Number(index===1),Number(index===2)]);});
  return shape.translate(position);
}
const escape=text=>String(text).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&apos;'}[c]));
const encoder=new TextEncoder();
async function bytes(blob){return new Uint8Array(await blob.arrayBuffer());}
export function meshCheck(mesh,name='Part'){
  if(!mesh.vertices.length||!mesh.triangles.length||mesh.vertices.some(n=>!Number.isFinite(n))||mesh.normals.some(n=>!Number.isFinite(n)))throw Error('Non-finite or empty mesh');
  const vertices=mesh.vertices.length/3;if(mesh.triangles.some(i=>!Number.isInteger(i)||i<0||i>=vertices))throw Error('Invalid triangle indices');
  const edges=new Map();let collapsed=0;const triangles=[];
  const key=i=>mesh.vertices.slice(i*3,i*3+3).map(n=>Math.round(n*1e5)).join(',');
  // OCCT emits collapsed triangles at periodic seams and sphere poles. Remove
  // the entire collapsed face, not just its zero-length edge (which otherwise
  // counts the same surviving edge twice and falsely reports a nonmanifold mesh).
  for(let i=0;i<mesh.triangles.length;i+=3){const face=mesh.triangles.slice(i,i+3),v=face.map(key);if(new Set(v).size<3){collapsed++;continue;}triangles.push(...face);for(let j=0;j<3;j++){const a=v[j],b=v[(j+1)%3];const k=a<b?a+'|'+b:b+'|'+a;const row=edges.get(k)||[0,0];row[0]++;row[1]+=a<b?1:-1;edges.set(k,row);}}
  if(!triangles.length)throw Error(`${name}: tessellation contains no non-collapsed faces`);
  mesh.triangles=triangles;
  const watertight=[...edges.values()].every(([count,winding])=>count===2&&winding===0);
  if(!watertight){const bad=[...edges.values()].filter(([count,winding])=>count!==2||winding!==0);throw Error(`${name}: tessellation is not closed with consistent winding (${bad.length} edges; ${JSON.stringify(bad.slice(0,8))})`);}
  return {finite:true,valid_indices:true,watertight:true,consistent_winding:true,removed_collapsed_triangles:collapsed,topology_weld_mm:1e-5};
}
function technicalSheet(compound,plan,bounds){
  const panels=[{name:'TOP / +Z',projection:'XY',x:15,y:27},{name:'ISOMETRIC',projection:new ProjectionCamera([0,0,0],[1,-1,1],[1,1,0]),x:220,y:27},{name:'SIDE / -Y',projection:'XZ',x:15,y:139},{name:'END / +X',projection:'YZ',x:220,y:139}];
  const items=[];
  for(const panel of panels){
    const {visible,hidden}=drawProjection(compound,panel.projection);
    const outer=visible.toSVG(3);const viewBox=visible.toSVGViewBox(3);
    const inside=svg=>svg.slice(svg.indexOf('>')+1,svg.lastIndexOf('</svg>'));
    const [left,top,width,height]=viewBox.split(/\s+/).map(Number),scale=Math.min(182/width,92/height),tx=panel.x+(182-width*scale)/2-left*scale,ty=panel.y+(92-height*scale)/2-top*scale;
    items.push(`<text x="${panel.x}" y="${panel.y-5}" font-size="2.6">${panel.name}</text><g transform="translate(${tx} ${ty}) scale(${scale})"><g fill="none" stroke="#a0a6a2" stroke-width=".18" stroke-dasharray="1.2 .8">${inside(hidden.toSVG(3))}</g><g fill="none" stroke="#26352d" stroke-width=".25">${inside(outer)}</g></g>`);
  }
  const dims=bounds[1].map((n,i)=>(n-bounds[0][i]).toFixed(2));
  return `<svg xmlns="http://www.w3.org/2000/svg" width="420mm" height="297mm" viewBox="0 0 420 297"><rect width="420" height="297" fill="white"/><g font-family="Arial,sans-serif" fill="#26352d"><rect x="7" y="7" width="406" height="283" fill="none" stroke="#526359" stroke-width=".3"/><text x="14" y="16" font-size="3.4">CYBR GEO / WHITEPRINT</text>${items.join('')}<path d="M7 252H413M216 252V290" stroke="#526359" fill="none" stroke-width=".25"/><text x="14" y="261" font-size="2.8">NOMINAL ENVELOPE: ${dims.join(' × ')} mm</text><text x="14" y="268" font-size="2.4">Analytic hidden-line removal. Each view fitted independently.</text><text x="14" y="275" font-size="2.4">No tolerance, strength, contact or manufacturing qualification.</text><text x="14" y="282" font-size="2.4">Use STEP for model dimensions. This sheet is a concept drawing.</text><text x="223" y="264" font-size="3.8">${escape(plan.name.slice(0,45))}</text><text x="223" y="274" font-size="2.7">UNITS: mm | SCALE: FITTED / DO NOT SCALE | REV: A</text><text x="223" y="284" font-size="2.7">CONCEPT / NOT RELEASED FOR FABRICATION</text></g></svg>`;
}
export async function buildCad(plan,progress=()=>{},options={}){
  if(!oc)throw Error('CAD kernel is not initialized');
  const preflight=await preflightPlan(plan);
  const shapes=[],geometries=[],materials=[],nodes=[],checks=[],files={},names=new Set();
  const bounds=[[Infinity,Infinity,Infinity],[-Infinity,-Infinity,-Infinity]];
  try{
    for(const [index,spec] of plan.parts.entries()){
      if(!/^[A-Za-z0-9 _-]{1,60}$/.test(spec.name)||names.has(spec.name)||/^(con|prn|aux|nul|com[1-9]|lpt[1-9])$/i.test(spec.name))throw Error('Use unique portable part names');names.add(spec.name);
      if(!Array.isArray(spec.cuts)||spec.cuts.length>48||!Array.isArray(spec.unions)||spec.unions.length>24)throw Error('CAD operation budget exceeded');
      progress(`Constructing ${spec.name} / ${index+1} of ${plan.parts.length}`);
      let shape=primitive(spec.body);shapes.push(shape);
      for(const item of spec.unions){const tool=primitive(item);try{const next=shape.fuse(tool);shape.delete();shape=next;shapes[index]=shape;}finally{tool.delete();}}
      for(const item of spec.cuts){const tool=primitive(item);try{const next=shape.cut(tool);shape.delete();shape=next;shapes[index]=shape;}finally{tool.delete();}}
      if(!Number.isFinite(spec.fillet)||spec.fillet<0||spec.fillet>25)throw Error('Invalid fillet radius');
      if(spec.fillet){const next=shape.fillet(spec.fillet);shape.delete();shape=next;shapes[index]=shape;}
      const analyzer=new oc.BRepCheck_Analyzer(shape.wrapped,true,false,false);const valid=analyzer.IsValid();analyzer.delete();
      const volume=measureVolume(shape);const solids=shape.solids;const count=solids.length;solids.forEach(s=>s.delete());
      if(!valid||!Number.isFinite(volume)||volume<=.001||count!==1)throw Error(`${spec.name}: expected one valid connected positive-volume solid (valid=${valid}, solids=${count}, volume_mm3=${volume})`);
      const box=shape.boundingBox;box.bounds.forEach((point,side)=>point.forEach((n,axis)=>bounds[side][axis]=side?Math.max(bounds[side][axis],n):Math.min(bounds[side][axis],n)));box.delete();
      const mesh=shape.mesh({tolerance:.06,angularTolerance:.15});const integrity=meshCheck(mesh,spec.name);
      const lines=shape.meshEdges({tolerance:.06,angularTolerance:.15}).lines;const edges=[];for(let i=0;i<lines.length;i+=6)edges.push([lines.slice(i,i+3),lines.slice(i+3,i+6)]);
      vector(spec.color);if(spec.color.some(n=>n<0||n>1)||![spec.metalness,spec.roughness].every(n=>Number.isFinite(n)&&n>=0&&n<=1))throw Error('Invalid material');
      geometries.push({positions:mesh.vertices,normals:mesh.normals,indices:mesh.triangles,edges,valid,volume_mm3:volume,provenance:'OpenCascade B-rep'});
      materials.push({color:spec.color,metalness:spec.metalness,roughness:spec.roughness,ior:1.5,transmission:0});
      nodes.push({id:index,parent:null,name:spec.name,shape:index,material:index,matrix:[1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1]});
      checks.push({name:spec.name,analytic_valid:valid,volume_mm3:volume,...integrity});
      files[`step_parts/${spec.name}.step`]=await bytes(shape.blobSTEP());files[`stl/${spec.name}.stl`]=await bytes(shape.blobSTL({binary:true,tolerance:.06,angularTolerance:.15}));
    }
    if(bounds[1].some((n,i)=>n-bounds[0][i]>10000))throw Error('Assembly exceeds the studio size limit');
    progress('Exporting named STEP assembly');files['assembly.step']=await bytes(exportSTEP(shapes.map((shape,i)=>({shape,name:plan.parts[i].name,color:'#'+plan.parts[i].color.map(n=>Math.round(Math.pow(n,1/2.2)*255).toString(16).padStart(2,'0')).join('')})),{unit:'MM',modelUnit:'MM'}));
    let svg='';if(options.drawings!==false){const compound=makeCompound(shapes.map(s=>s.clone()));try{progress('Drawing analytic whiteprints');svg=technicalSheet(compound,plan,bounds);}finally{compound.delete();}}
    const joints=preflight.joints;
    const assembly={schema:'cybrgeo-browser/1',units:'mm',name:plan.name,geometries,materials,nodes,joints,construction:preflight.construction};
    const report={model:plan.name,parts:plan.parts.length,units:'mm',bounds_mm:bounds,checks,recipe_sha256:preflight.digest,construction_bound:!!preflight.construction,kinematics:'prescribed joint graph; not contact dynamics',dynamics:'not simulated',hardware:'unqualified',backend:'OpenCascade WebAssembly / CYBR GEO recipe contract',disclaimer:'Geometry checks only; no strength, fit, contact or manufacturing qualification.'};
    files['assembly.json']=encoder.encode(JSON.stringify(assembly));files['recipe.json']=encoder.encode(JSON.stringify(plan,null,2));files['validation.json']=encoder.encode(JSON.stringify(report,null,2));if(svg)files['whiteprint.svg']=encoder.encode(svg);
    files['parts.csv']=encoder.encode('id,name,analytic_cad,volume_mm3\n'+checks.map((r,i)=>`${i+1},${r.name},true,${r.volume_mm3}`).join('\n'));
    files['joints.json']=encoder.encode(JSON.stringify({units:'mm',angle_units:'degrees',joints},null,2));
    if(preflight.construction)files['construction.json']=encoder.encode(JSON.stringify(preflight.construction,null,2));
    return {assembly,files,svg,report,bounds};
  }finally{shapes.forEach(shape=>{try{shape.delete();}catch{}});}
}
