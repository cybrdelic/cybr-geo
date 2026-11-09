"use strict";
/* Dependency-free stereo and inverse optical mapping. No inferred position. */
(() => {
  const canvas=document.getElementById("view"),status=document.getElementById("status");
  const gl=canvas.getContext("webgl2",{antialias:false,preserveDrawingBuffer:true});
  if(!gl){status.textContent="WebGL2 is unavailable in this browser.";return;}
  const PI=Math.PI;
  const spec={ipd:64,focus:0,relief:15,screenW:159.1,screenH:73.4,
              screenY:43.5,n:1.49,thickness:8.5,efl:45,diameter:34};
  spec.radius=(spec.n-1)*spec.efl*(1+Math.sqrt(1-spec.thickness/(spec.n*spec.efl)));
  const qmul=(a,b)=>[
    a[3]*b[0]+a[0]*b[3]+a[1]*b[2]-a[2]*b[1],
    a[3]*b[1]-a[0]*b[2]+a[1]*b[3]+a[2]*b[0],
    a[3]*b[2]+a[0]*b[1]-a[1]*b[0]+a[2]*b[3],
    a[3]*b[3]-a[0]*b[0]-a[1]*b[1]-a[2]*b[2]];
  const axis=(v,a)=>[v[0]*Math.sin(a/2),v[1]*Math.sin(a/2),v[2]*Math.sin(a/2),Math.cos(a/2)];
  const qrot=(q,v)=>qmul(qmul(q,[...v,0]),[-q[0],-q[1],-q[2],q[3]]).slice(0,3);
  let sensor=[0,0,0,1],center=[0,0,0,1],yaw=0,pitch=0,tracking=false;
  let lastSensor=0,frames=0,grid=false,drag=null,sensorEnabled=false;
  const normalize=v=>{const n=Math.hypot(...v);return v.map(x=>x/n);};
  const dot=(a,b)=>a.reduce((s,x,i)=>s+x*b[i],0);
  const sub=(a,b)=>a.map((x,i)=>x-b[i]);
  const plus=(a,b)=>a.map((x,i)=>x+b[i]);
  const mul=(a,k)=>a.map(x=>x*k);
  function hit(o,d,c,r){
    const oc=sub(o,c),b=dot(oc,d),disc=b*b-dot(oc,oc)+r*r;
    if(disc<0)return null;
    const root=Math.sqrt(disc),t=[-b-root,-b+root].find(x=>x>1e-6);
    return t===undefined?null:plus(o,mul(d,t));
  }
  function refract(d,normal,n1,n2){
    let N=normal;
    if(dot(d,N)>0)N=mul(N,-1);
    const eta=n1/n2,c=-dot(N,d),k=1-eta*eta*(1-c*c);
    return k<0?null:normalize(plus(mul(d,eta),mul(N,eta*c-Math.sqrt(k))));
  }
  function ray(angle){
    const R=spec.radius,c=R-spec.thickness/2,y=spec.focus;
    let o=[0,y-spec.thickness/2-spec.relief,0],d=[Math.sin(angle),Math.cos(angle),0];
    const c1=[0,y+c,0],c2=[0,y-c,0];
    let p=hit(o,d,c1,R);
    if(!p||Math.hypot(p[0],p[2])>spec.diameter/2)return null;
    d=refract(d,normalize(sub(p,c1)),1,spec.n);
    let q=hit(plus(p,mul(d,1e-5)),d,c2,R);
    if(!q||Math.hypot(q[0],q[2])>spec.diameter/2)return null;
    d=refract(d,normalize(sub(q,c2)),spec.n,1);
    if(!d||d[1]<=0)return null;
    return q[0]+d[0]*(spec.screenY-q[1])/d[1];
  }
  function buildLut(){
    const samples=[];
    for(let i=0;i<=600;i++){
      const theta=i*PI/2400,x=ray(theta);
      if(x===null)break;
      if(samples.length && x<=samples[samples.length-1][0])break;
      samples.push([x,theta]);
    }
    const lut=new Float32Array(128);let j=0;
    for(let i=0;i<128;i++){
      const x=i*100/127;
      while(j+1<samples.length && samples[j+1][0]<x)j++;
      if(j+1>=samples.length){lut[i]=-1;continue;}
      const [x0,t0]=samples[j],[x1,t1]=samples[j+1];
      lut[i]=t0+(t1-t0)*(x-x0)/(x1-x0);
    }
    return lut;
  }
  const vs="#version 300 es\nprecision highp float;\nvoid main(){vec2 p=vec2((gl_VertexID<<1)&2,gl_VertexID&2);gl_Position=vec4(p*2.-1.,0,1);}";
  const fs="#version 300 es\nprecision highp float;\nout vec4 frag;\nuniform vec2 resolution;\nuniform vec4 orientation;\nuniform float ipd,screenW,screenH;\nuniform float rays[128];\nuniform int calibration;\nvec3 rotateQ(vec4 q,vec3 v){return v+2.*cross(q.xyz,cross(q.xyz,v)+q.w*v);}\nfloat sphere(vec3 ro,vec3 rd,vec3 center,float radius){\n  vec3 o=ro-center;float b=dot(o,rd),h=b*b-dot(o,o)+radius*radius;\n  if(h<0.)return 1e5;\n  float t=-b-sqrt(h);return t>0.?t:1e5;\n}\nvec3 shade(vec3 ro,vec3 rd){\n  vec3 sky=mix(vec3(.025,.05,.073),vec3(.31,.41,.43),pow(max(0.,rd.y),.6));\n  float best=1e4;vec3 n=vec3(0,1,0),color=sky;\n  if(rd.y<-.001){\n    float t=-ro.y/rd.y;\n    if(t>0.){\n      best=t;vec3 p=ro+rd*t;\n      vec2 g=abs(fract(p.xz*.5-.5)-.5)/max(fwidth(p.xz*.5),vec2(.0001));\n      float line=1.-min(min(g.x,g.y),1.);\n      color=mix(vec3(.17,.21,.22),vec3(.045,.085,.09),line);\n    }\n  }\n  for(int i=0;i<4;i++){\n    vec3 center=i==0?vec3(-.45,1.6,-1.7):i==1?vec3(.65,1.45,-3.2):i==2?vec3(-1.3,1.15,-5.0):vec3(2.,2.,-8.);\n    float radius=i==0?.18:i==1?.32:i==2?.58:.8;\n    float t=sphere(ro,rd,center,radius);\n    if(t<best){best=t;n=normalize(ro+rd*t-center);\n      color=i==0?vec3(.92,.33,.13):i==1?vec3(.04,.57,.49):i==2?vec3(.72,.69,.55):vec3(.24,.42,.76);\n    }\n  }\n  if(best<1e4){\n    float light=.24+.76*max(0.,dot(n,normalize(vec3(-.4,.8,.35))));\n    color*=light;\n    color=mix(color,sky,1.-exp(-best*.018));\n  }\n  return pow(max(color,vec3(0)),vec3(1./2.2));\n}\nvoid main(){\n  vec2 uv=gl_FragCoord.xy/resolution;\n  float eye=uv.x<.5?-1.:1.;\n  vec2 mm=vec2((uv.x-.5)*screenW-eye*ipd*.5,(uv.y-.5)*screenH);\n  float rho=length(mm),index=rho*127./100.;\n  int lo=int(floor(index));\n  if(lo<0||lo>=127){frag=vec4(0,0,0,1);return;}\n  float a=rays[lo],b=rays[lo+1];\n  if(a<0.||b<0.){frag=vec4(0,0,0,1);return;}\n  float theta=mix(a,b,fract(index));\n  vec2 radial=rho>1e-7?mm/rho:vec2(0);\n  vec3 local=vec3(radial*sin(theta),-cos(theta));\n  vec3 rd=rotateQ(orientation,local);\n  vec3 ro=vec3(0,1.65,0)+rotateQ(orientation,vec3(eye*ipd*.0005,0,0));\n  vec3 color=shade(ro,rd);\n  if(calibration==1){\n    vec2 plane=local.xy/-local.z;\n    vec2 g=abs(fract(plane*8.-.5)-.5)/max(fwidth(plane*8.),vec2(.001));\n    float l=1.-min(min(g.x,g.y),1.);\n    color=mix(vec3(.018),vec3(.74,.87,.84),l);\n    if(length(plane)<.006)color=vec3(1.,.25,.1);\n  }\n  if(abs(uv.x-.5)<1./resolution.x)color=vec3(0);\n  frag=vec4(color,1);\n}";
  function shader(type,source){
    const sh=gl.createShader(type);gl.shaderSource(sh,source);gl.compileShader(sh);
    if(!gl.getShaderParameter(sh,gl.COMPILE_STATUS))throw Error(gl.getShaderInfoLog(sh));
    return sh;
  }
  let program;
  try{
    program=gl.createProgram();gl.attachShader(program,shader(gl.VERTEX_SHADER,vs));
    gl.attachShader(program,shader(gl.FRAGMENT_SHADER,fs));gl.linkProgram(program);
    if(!gl.getProgramParameter(program,gl.LINK_STATUS))throw Error(gl.getProgramInfoLog(program));
  }catch(e){status.textContent="Graphics failed: "+e.message;throw e;}
  gl.useProgram(program);gl.bindVertexArray(gl.createVertexArray());
  const loc=Object.fromEntries(["resolution","orientation","ipd","screenW","screenH","rays","calibration"]
       .map(k=>[k,gl.getUniformLocation(program,k)]));
  function update(){
    for(const k of ["ipd","focus","relief"]){
      spec[k]=Number(document.getElementById(k).value);
      document.getElementById(k+"Value").textContent=String(spec[k]);
    }
    gl.uniform1fv(loc.rays,buildLut());
    gl.uniform1f(loc.ipd,spec.ipd);gl.uniform1f(loc.screenW,spec.screenW);
    gl.uniform1f(loc.screenH,spec.screenH);
  }
  ["ipd","focus","relief"].forEach(k=>document.getElementById(k).addEventListener("input",update));
  document.getElementById("grid").onchange=e=>{grid=e.target.checked;};
  function onSensor(e){
    if(!sensorEnabled||e.alpha===null||e.beta===null||e.gamma===null)return;
    const rad=PI/180,angle=(screen.orientation?.angle??window.orientation??0)*rad;
    sensor=qmul(qmul(qmul(axis([0,1,0],e.alpha*rad),
      axis([1,0,0],e.beta*rad)),axis([0,0,1],-e.gamma*rad)),
      qmul(axis([1,0,0],-PI/2),axis([0,0,1],-angle)));
    if(!tracking){
      tracking=true;
      const forward=qrot(sensor,[0,0,-1]);
      center=axis([0,1,0],-Math.atan2(-forward[0],-forward[2]));
    }
    lastSensor=performance.now();
  }
  window.addEventListener("deviceorientation",onSensor);
  function recenter(){
    if(tracking){
      const forward=qrot(sensor,[0,0,-1]);
      center=axis([0,1,0],-Math.atan2(-forward[0],-forward[2]));
    }else{yaw=0;pitch=0;}
  }
  document.getElementById("recenter").onclick=recenter;
  document.getElementById("start").onclick=async()=>{
    try{
      if(typeof DeviceOrientationEvent!=="undefined"&&typeof DeviceOrientationEvent.requestPermission==="function"){
        const permission=await DeviceOrientationEvent.requestPermission();
        if(permission!=="granted"){status.textContent="Orientation permission was denied.";return;}
      }
      sensorEnabled=true;
      await document.documentElement.requestFullscreen?.();
      try{await screen.orientation.lock("landscape");}catch(_){}
      document.body.classList.add("immersive");
      status.textContent="Waiting for phone orientation events.";
    }catch(e){status.textContent=e.message;}
  };
  document.getElementById("inspect").onclick=()=>document.body.classList.add("immersive");
  document.getElementById("exit").onclick=async()=>{
    document.body.classList.remove("immersive");
    if(document.fullscreenElement)await document.exitFullscreen();
  };
  canvas.addEventListener("pointerdown",e=>{drag=[e.clientX,e.clientY];canvas.setPointerCapture(e.pointerId);});
  canvas.addEventListener("pointermove",e=>{
    if(!drag||tracking)return;
    yaw-=(e.clientX-drag[0])*.003;pitch=Math.max(-1.3,Math.min(1.3,pitch-(e.clientY-drag[1])*.003));
    drag=[e.clientX,e.clientY];
  });
  canvas.addEventListener("pointerup",()=>{drag=null;});
  window.addEventListener("keydown",e=>{
    if(e.key==="Escape")document.body.classList.remove("immersive");
    if(e.key.toLowerCase()==="r")recenter();
  });
  update();
  function frame(){
    const width=Math.min(1600,Math.round(innerWidth*devicePixelRatio/2)*2);
    const height=Math.round(width*innerHeight/innerWidth/2)*2;
    if(canvas.width!==width||canvas.height!==height){canvas.width=width;canvas.height=height;}
    gl.viewport(0,0,width,height);gl.uniform2f(loc.resolution,width,height);
    const q=tracking?qmul(center,sensor):qmul(axis([0,1,0],yaw),axis([1,0,0],pitch));
    gl.uniform4fv(loc.orientation,q);gl.uniform1i(loc.calibration,grid?1:0);
    gl.drawArrays(gl.TRIANGLES,0,3);frames++;
    if(frames%60===0)status.textContent=tracking?
      (performance.now()-lastSensor<1000?"Phone orientation active · 3DoF":"Orientation events paused"):
      "Desktop preview · drag to look. On phone: use HTTPS or localhost, then Enter VR.";
    requestAnimationFrame(frame);
  }
  window.visor={
    spec,ray,buildLut,
    stats:()=>({frames,tracking,width:canvas.width,height:canvas.height,error:gl.getError()}),
    setPose:(y,p)=>{yaw=y;pitch=p;},
    opticalSamples:()=>[0,5,10,20,25].map(a=>[a,ray(a*PI/180)]),
  };
  frame();
})();
