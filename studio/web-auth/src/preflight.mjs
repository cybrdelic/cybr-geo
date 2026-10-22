// Construction metadata records design intent. It never certifies dynamics or hardware.
import {validateJoints} from './joints.mjs';

function canonical(value) {
  if (typeof value === 'number') {
    if (!Number.isFinite(value)) throw Error('Recipe numbers must be finite');
    return Math.abs(value) < 0.5e-9 ? '0.000000000' : value.toFixed(9);
  }
  if (Array.isArray(value)) return value.map(canonical);
  if (value && typeof value === 'object') return Object.fromEntries(Object.keys(value).sort().map(key => [key,canonical(value[key])]));
  return value;
}

export async function recipeDigest(plan) {
  const payload = JSON.stringify(canonical({parts:plan.parts,joints:plan.joints || []}));
  const digest = await crypto.subtle.digest('SHA-256',new TextEncoder().encode(payload));
  return [...new Uint8Array(digest)].map(n => n.toString(16).padStart(2,'0')).join('');
}

export async function preflightPlan(plan) {
  if (!plan || plan.units !== 'mm' || !Array.isArray(plan.parts) || !plan.parts.length) throw Error('A millimetre recipe with named parts is required');
  if (typeof plan.name !== 'string' || !plan.name.trim()) throw Error('A model name is required');
  if(plan.parts.some(part=>!part||typeof part.name!=='string'))throw Error('Every part needs a string name');
  // Run graph validation before creating even the first OpenCascade shape.
  const joints = validateJoints(plan.parts,plan.joints || []);
  const expanded = !!plan.construction;
  if (plan.parts.length > (expanded ? 256 : 64)) throw Error('Recipe part budget exceeded');
  const names = new Set();
  const vector = (value,positive=false) => Array.isArray(value) && value.length===3 && value.every(n => Number.isFinite(n) && Math.abs(n)<=10000 && (!positive || n>0));
  for (const part of plan.parts) {
    if (!/^[A-Za-z0-9 _-]{1,60}$/.test(part.name) || names.has(part.name) || /^(con|prn|aux|nul|com[1-9]|lpt[1-9])$/i.test(part.name)) throw Error('Use unique portable part names');
    names.add(part.name);
    if (!Array.isArray(part.cuts) || part.cuts.length>48 || !Array.isArray(part.unions) || part.unions.length>24) throw Error('CAD operation budget exceeded');
    for (const primitive of [part.body,...part.unions,...part.cuts]) {
      if (!primitive || !['box','cylinder','sphere'].includes(primitive.kind) || !vector(primitive.size,true) || !vector(primitive.position) || !vector(primitive.rotation)) throw Error(`${part.name}: invalid primitive dimensions or transform`);
    }
    if (!Number.isFinite(part.fillet) || part.fillet<0 || part.fillet>25) throw Error('Invalid fillet radius');
    if (!vector(part.color) || part.color.some(n => n<0 || n>1) || ![part.metalness,part.roughness].every(n => Number.isFinite(n) && n>=0 && n<=1)) throw Error('Invalid material');
  }
  const digest = await recipeDigest(plan);
  if (expanded) {
    if(plan.construction.schema!=='cybrgeo-construction/1')throw Error('Unsupported construction schema');
    if (!/^[a-f0-9]{64}$/.test(plan.construction.recipe_sha256 || '') || plan.construction.recipe_sha256 !== digest) throw Error('Construction no longer matches the geometry or joints. Regenerate from the mechanism plan before building.');
    if (joints.length !== plan.parts.length) throw Error('Every constructed part must belong to the load and joint hierarchy');
  }
  return {joints,digest,construction:expanded ? structuredClone(plan.construction) : null};
}
