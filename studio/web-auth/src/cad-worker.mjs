import {initialize,buildCad} from './cad-kernel.mjs';
self.onmessage=async({data})=>{
  try {
    await initialize({locateFile:()=>new URL('./vendor/cad.wasm',self.location.href).href});
    const result=await buildCad(data.plan,message=>self.postMessage({progress:message}),{drawings:!!data.drawings});
    self.postMessage({result});
  } catch(error) {self.postMessage({error:String(error.message || error)});}
};
