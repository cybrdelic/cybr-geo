// Photographic offline renderer built on the verified Mechanism Lab transport core.
// Reuses the BVH, triangle intersection, GGX BSDF, area-light sampling and MIS
// from pathtrace.cpp, but replaces the presentation layer: physical sensor/lens
// rays, thin-lens depth of field, camera-relative softboxes, subdued controllable
// environment transport, an automatically placed matte studio floor, and richer
// *explicit* material microfinish. No image generation or post-hoc fake geometry.
#define main mechanism_pathtrace_legacy_main
#include "pathtrace.cpp"
#undef main

struct PhotoOptions {
 std::string mesh,out,materials;
 int w=1280,h=800,spp=128,depth=12,threads=4,seed=2026;
 float az=220.f,el=18.f,tx=0,ty=0,tz=0;
 float focal=65.f,sensorWidth=36.f,cameraDistance=420.f,fstop=5.6f,focusDistance=0.f;
 float exposure=1.f,envStrength=.24f,backgroundStrength=1.f,lightSize=1.35f,lightIntensity=1.f;
 float floorGap=2.f,floorRoughness=.82f,orthoScale=120.f;
 bool ortho=false,noFloor=false;
};

PhotoOptions parsePhoto(int argc,char**argv){
 PhotoOptions o;
 if(argc<3){
  std::cerr<<"usage: mechanism_photoreal scene.meshbin output.ppm [options]\n"
           <<"  --materials FILE --w 1920 --h 1080 --spp 512 --depth 14\n"
           <<"  --az 220 --el 18 --tx 100 --ty 0 --tz 0\n"
           <<"  --focal-length 70 --sensor-width 36 --camera-distance 360\n"
           <<"  --fstop 5.6 --focus-distance 360 --env-strength .22\n";
  std::exit(1);
 }
 o.mesh=argv[1];o.out=argv[2];
 for(int i=3;i<argc;i++){
  std::string a=argv[i];
  auto value=[&](){if(i+1>=argc){std::cerr<<"Missing value "<<a<<"\n";std::exit(1);}return std::string(argv[++i]);};
  if(a=="--materials")o.materials=value();
  else if(a=="--w")o.w=std::stoi(value());else if(a=="--h")o.h=std::stoi(value());
  else if(a=="--spp")o.spp=std::stoi(value());else if(a=="--depth")o.depth=std::stoi(value());
  else if(a=="--threads")o.threads=std::stoi(value());else if(a=="--seed")o.seed=std::stoi(value());
  else if(a=="--az")o.az=std::stof(value());else if(a=="--el")o.el=std::stof(value());
  else if(a=="--tx")o.tx=std::stof(value());else if(a=="--ty")o.ty=std::stof(value());else if(a=="--tz")o.tz=std::stof(value());
  else if(a=="--focal-length")o.focal=std::stof(value());else if(a=="--sensor-width")o.sensorWidth=std::stof(value());
  else if(a=="--camera-distance")o.cameraDistance=std::stof(value());else if(a=="--fstop")o.fstop=std::stof(value());
  else if(a=="--focus-distance")o.focusDistance=std::stof(value());else if(a=="--exposure")o.exposure=std::stof(value());
  else if(a=="--env-strength")o.envStrength=std::stof(value());else if(a=="--background-strength")o.backgroundStrength=std::stof(value());
  else if(a=="--light-size")o.lightSize=std::stof(value());else if(a=="--light-intensity")o.lightIntensity=std::stof(value());
  else if(a=="--floor-gap")o.floorGap=std::stof(value());else if(a=="--floor-roughness")o.floorRoughness=std::stof(value());
  else if(a=="--scale")o.orthoScale=std::stof(value());else if(a=="--ortho")o.ortho=true;else if(a=="--no-floor")o.noFloor=true;
  else {std::cerr<<"Unknown option "<<a<<"\n";std::exit(1);}
 }
 if(o.w<1||o.h<1||o.spp<1||o.depth<1||o.threads<1||o.focal<=0||o.sensorWidth<=0||o.cameraDistance<=0||o.fstop<=0){
  std::cerr<<"Invalid photographic render options\n";std::exit(1);
 }
 if(o.focusDistance<=0)o.focusDistance=o.cameraDistance;
 return o;
}

float photoEnvStrength=.24f,photoBackgroundStrength=1.f;
Material photoFloor{V(.019f,.021f,.025f),0.f,.82f,0};

V photoEnvironment(V d,bool camera){
 // Primary background is deliberately much darker than the old inspection room.
 // Indirect environment remains non-zero so metals are never lit by black space.
 float sky=clamp(d.z*.5f+.5f);
 if(camera){
  float horizon=.88f+.12f*sky;
  return V(.0065f,.0075f,.0095f)*(photoBackgroundStrength*horizon);
 }
 V cool=V(.16f,.175f,.195f)*(0.68f+0.32f*sky);
 float horizon=std::exp(-sqr(d.z/.28f));
 V bounce=V(.12f,.105f,.09f)*(.16f*horizon);
 return (cool+bounce)*photoEnvStrength;
}

void photoPerturb(V p,V &n,Material &m){
 // Microfinish exists only when the material sidecar explicitly requests it.
 // Amplitudes are intentionally sub-pixel / highlight-scale, never fake seams.
 Frame fr(n);
 if(m.pattern==1){ // machined / turned
  float r=std::sqrt(p.y*p.y+p.z*p.z);
  float q=std::sin(r*53.1f)*.006f+std::sin(r*233.7f)*.0025f;
  n=unit(n+fr.t*q);m.rough=clamp(m.rough*(1.f+.035f*std::sin(r*31.7f)),.025f,1.f);
 }else if(m.pattern==2){ // bead blasted
  float q1=std::sin(p.x*67.7f+p.y*101.3f+p.z*47.9f)*std::sin(p.x*149.3f-p.z*89.1f);
  float q2=std::sin(p.y*83.9f-p.x*57.1f+p.z*131.7f);
  n=unit(n+fr.t*(.010f*q1)+fr.b*(.010f*q2));m.rough=clamp(m.rough*(1.f+.055f*q1),.04f,1.f);
 }else if(m.pattern==3){ // brushed
  float q=std::sin(p.x*82.1f+p.z*5.7f)*.008f+std::sin(p.x*271.9f)*.002f;
  n=unit(n+fr.t*q);m.rough=clamp(m.rough*(1.f+.05f*std::sin(p.x*41.3f)),.03f,1.f);
 }else if(m.pattern==4){ // polymer
  float q=std::sin(p.x*5.1f+p.y*8.33f)*std::sin(p.y*7.42f-p.x*9.21f);
  m.color*=1.f+.035f*q;m.rough=clamp(m.rough*(1.f+.035f*q),.06f,1.f);
 }else if(m.pattern==5){ // drawn fine wire
  float q=std::sin(p.x*177.1f+p.y*13.1f+p.z*11.7f)*.0045f;
  n=unit(n+fr.t*q);m.rough=clamp(m.rough*(1.f+.03f*std::sin(p.x*97.3f)),.025f,1.f);
 }else if(m.pattern==6){ // copper wire / bus
  float q=std::sin(p.x*121.7f+p.y*17.9f+p.z*23.3f);
  m.rough=clamp(m.rough*(1.f+.045f*q),.025f,1.f);
 }else if(m.pattern==7){ // anodized finish
  float q=std::sin(p.x*73.1f+p.y*109.7f+p.z*139.9f)*std::sin(p.x*31.1f-p.y*47.3f);
  n=unit(n+fr.t*(.005f*q)+fr.b*(.004f*std::sin(p.z*97.1f)));
  m.rough=clamp(m.rough*(1.f+.04f*q),.04f,1.f);
 }
}

V photoTrace(Ray ray,RNG&rng,int maxDepth){
 V L(0),throughput(1),prevP;float lastPDF=0;bool lastSpec=true;
 for(int depth=0;depth<maxDepth;depth++){
  Hit hit;
  if(!sceneHit(ray,hit,depth>0)){L+=throughput*photoEnvironment(ray.d,depth==0);break;}
  V p=ray.o+ray.d*hit.t;
  if(hit.light>=0){
   const auto&light=lights[hit.light];
   if(dot(light.n,-ray.d)>0){
    float w=1;
    if(depth>0&&!lastSpec){
     float dist2=dot(p-prevP,p-prevP);
     float pdf=dist2/(light.area*std::max(1e-8f,dot(light.n,-ray.d))*lights.size());
     w=powerHeuristic(lastPDF,pdf);
    }
    L+=throughput*light.emission*w;
   }
   break;
  }
  V n,gn;Material mat;
  if(hit.floor){n=gn=V(0,0,1);mat=photoFloor;}
  else{
   const auto&t=tris[hit.tri];
   n=unit(t.n0*(1-hit.u-hit.v)+t.n1*hit.u+t.n2*hit.v);gn=unit(cross(t.e1,t.e2));mat=mats[t.mat];
   if(dot(gn,-ray.d)<0)gn=-gn;if(dot(n,gn)<0)n=-n;
   if(dot(n,-ray.d)<.02f)n=unit(n+gn*(.021f-dot(n,-ray.d)));
  }
  photoPerturb(p,n,mat);
  V v=-ray.d;
  int li=std::min(int(lights.size())-1,int(rng.uniform()*lights.size()));const auto&light=lights[li];
  V lp=light.point(rng),delta=lp-p;float dist2=dot(delta,delta),dist=std::sqrt(dist2);V l=delta/dist;
  float nl=dot(n,l),cl=dot(light.n,-l);
  if(nl>0&&cl>0&&dot(gn,l)>0){
   float lightPDF=dist2/(light.area*cl*lights.size());
   if(!occluded(Ray(p+gn*EPS*2,l),dist-EPS*8)){
    float bsdfPDF=pdfBSDF(mat,n,v,l),w=powerHeuristic(lightPDF,bsdfPDF);
    L+=throughput*eval(mat,n,v,l)*light.emission*(nl*w/lightPDF);
   }
  }
  V lnext=sampleBSDF(mat,n,v,rng);float cosine=dot(n,lnext),pdf=pdfBSDF(mat,n,v,lnext);
  if(cosine<=0||pdf<1e-12f||dot(gn,lnext)<=0)break;
  V f=eval(mat,n,v,lnext);throughput*=f*(cosine/pdf);
  if(!std::isfinite(maxc(throughput)))break;
  if(depth>=4){float q=clamp(maxc(throughput),.08f,.94f);if(rng.uniform()>q)break;throughput*=1/q;}
  prevP=p;lastPDF=pdf;lastSpec=false;ray=Ray(p+gn*EPS*2,unit(lnext));
 }
 return L;
}

V diskSample(RNG &rng){
 float r=std::sqrt(rng.uniform()),phi=2*PI*rng.uniform();return V(r*std::cos(phi),r*std::sin(phi),0);
}

int main(int argc,char**argv){
 auto opt=parsePhoto(argc,argv);
 if(!opt.materials.empty()){
  std::ifstream mf(opt.materials);if(!mf){std::cerr<<"Cannot read material table\n";return 4;}
  std::vector<Material> supplied;float r,g,b,metal,rough;int pattern;
  while(mf>>r>>g>>b>>metal>>rough>>pattern){
   if(r<0||g<0||b<0||metal<0||metal>1||rough<=0||rough>1){std::cerr<<"Invalid material\n";return 4;}
   supplied.push_back({V(r,g,b),metal,rough,pattern});
  }
  if(supplied.empty()||!mf.eof()){std::cerr<<"Empty or malformed material table\n";return 4;}
  mats=std::move(supplied);
 }
 omp_set_num_threads(opt.threads);auto start=std::chrono::steady_clock::now();
 std::ifstream in(opt.mesh,std::ios::binary);if(!in){std::cerr<<"Cannot read mesh\n";return 2;}
 uint32_t count=0;in.read(reinterpret_cast<char*>(&count),4);tris.clear();order.clear();nodes.clear();lights.clear();tris.reserve(count);
 Box sceneBounds;
 for(uint32_t i=0;i<count;i++){
  float a[20];in.read(reinterpret_cast<char*>(a),80);if(!in){std::cerr<<"Truncated mesh\n";return 3;}
  int mat=int(a[18]),g=int(a[19]);if(mat<0||mat>=int(mats.size())){std::cerr<<"Invalid triangle material ID\n";return 5;}
  V p0(a[0],a[1],a[2]),p1(a[3],a[4],a[5]),p2(a[6],a[7],a[8]);
  V n0(a[9],a[10],a[11]),n1(a[12],a[13],a[14]),n2(a[15],a[16],a[17]);
  Tri t;t.p=p0;t.e1=p1-p0;t.e2=p2-p0;t.n0=n0;t.n1=n1;t.n2=n2;t.mat=mat;t.group=g;
  if(dot(cross(t.e1,t.e2),cross(t.e1,t.e2))>1e-17f){tris.push_back(t);sceneBounds.grow(t.bounds());}
 }
 if(tris.empty()){std::cerr<<"Empty scene\n";return 6;}
 order.resize(tris.size());std::iota(order.begin(),order.end(),0);nodes.reserve(tris.size()/2);build(0,order.size());
 photoEnvStrength=opt.envStrength;photoBackgroundStrength=opt.backgroundStrength;photoFloor.rough=opt.floorRoughness;
 enableFloor=!opt.noFloor;floorZ=sceneBounds.lo.z-opt.floorGap;
 V target(opt.tx,opt.ty,opt.tz);
 float az=opt.az*PI/180.f,el=opt.el*PI/180.f;
 V camera=target+V(std::cos(az)*std::cos(el),std::sin(az)*std::cos(el),std::sin(el))*opt.cameraDistance;
 V fwd=unit(target-camera),right=unit(cross(fwd,V(0,0,1))),up=cross(right,fwd);

 // Camera-relative softboxes. Sizes change independently from distance so shadow
 // softness is a controllable photographic property rather than a scene-scale accident.
 auto addLight=[&](V center,float width,float height,V emission){
  Light l(center,target,right,width*opt.lightSize,height*opt.lightSize,emission*opt.lightIntensity);lights.push_back(l);
 };
 V towardCamera=unit(camera-target);
 addLight(target+towardCamera*175.f-right*145.f+up*170.f,250.f,175.f,V(2.25f,2.32f,2.42f));
 addLight(target+towardCamera*95.f+right*190.f+up*35.f,210.f,145.f,V(.92f,.88f,.82f));
 addLight(target-towardCamera*175.f+right*115.f+up*145.f,205.f,90.f,V(1.70f,1.82f,2.05f));
 addLight(target-towardCamera*70.f-right*210.f+up*20.f,95.f,235.f,V(.70f,.76f,.84f));

 float aspect=float(opt.w)/opt.h;
 float sensorHeight=opt.sensorWidth/aspect;
 float lensRadius=opt.focal/(2.f*opt.fstop);
 float orthoHalf=opt.orthoScale*.5f;
 std::vector<V> image(size_t(opt.w)*opt.h);std::vector<float> guides(image.size()*9);std::atomic<int> rows{0};
 #pragma omp parallel for schedule(dynamic,1)
 for(int y=0;y<opt.h;y++){
  for(int x=0;x<opt.w;x++){
   RNG rng((uint64_t(y)*opt.w+x)*0x9e3779b97f4a7c15ULL+opt.seed);V c;float lum2=0;
   for(int s=0;s<opt.spp;s++){
    float nx=2.f*(x+rng.uniform())/opt.w-1.f,ny=1.f-2.f*(y+rng.uniform())/opt.h;
    Ray ray(camera,fwd);
    if(opt.ortho){
     ray=Ray(camera+right*(nx*aspect*orthoHalf)+up*(ny*orthoHalf),fwd);
    }else{
     float sx=nx*(opt.sensorWidth*.5f),sy=ny*(sensorHeight*.5f);
     V pinDir=unit(fwd*opt.focal+right*sx+up*sy);
     float focusT=opt.focusDistance/std::max(.05f,dot(pinDir,fwd));V focusPoint=camera+pinDir*focusT;
     V d=diskSample(rng);V lensOrigin=camera+right*(d.x*lensRadius)+up*(d.y*lensRadius);
     ray=Ray(lensOrigin,unit(focusPoint-lensOrigin));
    }
    V sample=photoTrace(ray,rng,opt.depth);c+=sample;float lum=dot(sample,V(.2126f,.7152f,.0722f));lum2+=lum*lum;
   }
   size_t idx=size_t(y)*opt.w+x;image[idx]=c/float(opt.spp);
   float nx=2.f*(x+.5f)/opt.w-1.f,ny=1.f-2.f*(y+.5f)/opt.h;Ray primary(camera,fwd);
   if(opt.ortho)primary=Ray(camera+right*(nx*aspect*orthoHalf)+up*(ny*orthoHalf),fwd);
   else primary=Ray(camera,unit(fwd*opt.focal+right*(nx*opt.sensorWidth*.5f)+up*(ny*sensorHeight*.5f)));
   Hit first;V guideN(0),guideA(0);float gd=0,materialID=-1;
   if(sceneHit(primary,first,false)){
    gd=first.t;
    if(first.floor){guideN=V(0,0,1);guideA=photoFloor.color;materialID=-2;}
    else if(first.tri>=0){const auto&t=tris[first.tri];guideN=unit(t.n0*(1-first.u-first.v)+t.n1*first.u+t.n2*first.v);guideA=mats[t.mat].color;materialID=t.mat;}
   }
   float avg=dot(image[idx],V(.2126f,.7152f,.0722f));float var=std::max(0.f,(lum2/opt.spp-avg*avg)/std::max(1,opt.spp-1));
   for(int k=0;k<3;k++){guides[idx*9+k]=guideN[k];guides[idx*9+3+k]=guideA[k];}
   guides[idx*9+6]=gd;guides[idx*9+7]=var;guides[idx*9+8]=materialID;
  }
  int done=++rows;if(done%std::max(1,opt.h/10)==0){float sec=std::chrono::duration<float>(std::chrono::steady_clock::now()-start).count();
   #pragma omp critical
   std::cerr<<100*done/opt.h<<"% "<<sec<<"s\n";
  }
 }
 std::ofstream pf(opt.out+".pfm",std::ios::binary);pf<<"PF\n"<<opt.w<<" "<<opt.h<<"\n-1.0\n";
 for(int y=opt.h-1;y>=0;y--)pf.write(reinterpret_cast<const char*>(&image[size_t(y)*opt.w]),sizeof(V)*opt.w);pf.close();
 std::ofstream out(opt.out,std::ios::binary);out<<"P6\n"<<opt.w<<" "<<opt.h<<"\n255\n";
 for(auto v:image)for(int k=0;k<3;k++){float t=aces(v[k]*opt.exposure);t=t<=.0031308f?12.92f*t:1.055f*std::pow(t,1/2.4f)-.055f;unsigned char b=static_cast<unsigned char>(clamp(t)*255+.5f);out.write(reinterpret_cast<char*>(&b),1);}
 std::ofstream ga(opt.out+".guides",std::ios::binary);uint32_t dims[2]={uint32_t(opt.w),uint32_t(opt.h)};ga.write(reinterpret_cast<char*>(dims),8);ga.write(reinterpret_cast<char*>(guides.data()),guides.size()*sizeof(float));ga.close();
 float sec=std::chrono::duration<float>(std::chrono::steady_clock::now()-start).count();
 std::cerr<<"Wrote "<<opt.out<<"; "<<opt.w<<"x"<<opt.h<<", "<<opt.spp<<" spp, f/"<<opt.fstop<<", "<<sec<<" seconds\n";
 return 0;
}
