import {readFile,writeFile} from 'node:fs/promises';
import {join} from 'node:path';
import {preflightPlan} from './src/preflight.mjs';
const [address,inputRoot,reportPath]=process.argv.slice(2);
if(!address||!inputRoot||!reportPath)throw Error('Usage: node probe-http.mjs LOOPBACK_URL RECIPE_DIRECTORY REPORT.json');
const target=new URL(address);if(target.protocol!=='http:'||!['127.0.0.1','localhost'].includes(target.hostname))throw Error('Use the local Studio loopback URL');
const base=target.origin;
const [sessionResponse,examplesResponse,indexResponse,scriptResponse,workerResponse,wasmResponse]=await Promise.all(['/api/session','/api/examples','/','/inspector.mjs','/cad-worker.mjs','/vendor/cad.wasm'].map(path=>fetch(base+path)));
for(const response of [sessionResponse,examplesResponse,indexResponse,scriptResponse,workerResponse,wasmResponse])if(!response.ok)throw Error('Failed route '+new URL(response.url).pathname+':'+response.status);
const session=await sessionResponse.json(),examples=await examplesResponse.json(),html=await indexResponse.text(),script=await scriptResponse.text(),worker=await workerResponse.text(),wasm=new Uint8Array(await wasmResponse.arrayBuffer());
if(!html.includes('id="document"')||!html.includes('Run engineering')||!script.includes('/api/engineer')||!worker.includes('buildCad')||wasm.length<1e6||wasm[0]!==0||wasm[1]!==97||wasm[2]!==115||wasm[3]!==109)throw Error('Built Studio source closure was not served');
const cases=[...examples.examples.map(example=>({id:example.id,source:example.source})),{id:'four_axis',source:JSON.parse(await readFile('evidence/four-axis-source.json','utf8'))}];
const results=[];
for(const item of cases){
  const response=await fetch(base+'/api/plan',{method:'POST',headers:{'Content-Type':'application/json','Origin':base,'X-Studio-Token':session.csrf},body:JSON.stringify({mechanism_source:item.source})});
  const result=await response.json();if(!response.ok||result.provider!=='constructive')throw Error('Source route failed for '+item.id+': '+JSON.stringify(result));
  const checked=await preflightPlan(result.plan),saved=JSON.parse(await readFile(join(inputRoot,item.id+'.json'),'utf8'));
  for(const key of ['recipe_sha256','mechanism_sha256','model_sha256'])if(checked.construction[key]!==saved.construction[key])throw Error('Server/browser artifact mismatch '+item.id+' '+key);
  results.push({example:item.id,status:response.status,provider:result.provider,parts:result.plan.parts.length,model_sha256:checked.construction.model_sha256});
}
const rejected=await fetch(base+'/api/plan',{method:'POST',headers:{'Content-Type':'application/json','Origin':base},body:JSON.stringify({mechanism_source:cases[0].source})});
if(rejected.status!==403)throw Error('Missing CSRF token should be rejected');
const report={schema:'cybrgeo-studio-http-evidence/1',default_example:examples.default_example,constructive_available:session.constructiveAvailable,source_requests:results,built_assets:{index:true,inspector:true,worker:true,wasm_bytes:wasm.length,wasm_content_type:wasmResponse.headers.get('content-type')},missing_token_status:rejected.status,scope:'Real loopback HTTP requests against local Studio; source compilation matched actual WASM artifact identities. No duplicated engineering solve and no live browser UI execution.'};
await writeFile(reportPath,JSON.stringify(report,null,2)+'\n');console.log(JSON.stringify(report,null,2));
