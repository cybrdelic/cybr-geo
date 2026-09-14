// Photographic offline renderer built on the verified Mechanism Lab transport core.
// Reuses the BVH, triangle intersection, GGX BSDF, area-light sampling and MIS
// from pathtrace.cpp, but replaces the presentation layer: physical sensor/lens
// rays, thin-lens depth of field, camera-relative softboxes, subdued controllable
// environment transport, an automatically placed matte studio floor, and richer
// *explicit* material microfinish. No image generation or post-hoc fake geometry.
#define main mechanism_pathtrace_legacy_main
#include "pathtrace.cpp"
#undef main
#include "photo_bsdf.h"

struct PhotoOptions {
 std::string mesh,out,materials;
 int w=1280,h=800,spp=128,depth=12,threads=4,seed=2026;
 float az=220.f,el=18.f,tx=0,ty=0,tz=0;
 float focal=65.f,sensorWidth=36.f,cameraDistance=420.f,fstop=11.f,focusDistance=0.f;
 float exposure=1.f,envStrength=.24f,backgroundStrength=1.f,lightSize=1.35f,lightIntensity=1.f;
 float floorGap=0.f,floorRoughness=.73f,orthoScale=120.f;
 float studioScale=1.f,studioAz=0.f,studioEl=0.f,floorHeight=0.f;
 V studioTarget{0,0,0};bool fixedStudio=false,fixedFloor=false;
 bool ortho=false,noFloor=false;
 std::string studio="product";
 V floorColor{.09f,.095f,.105f},backgroundColor{.012f,.016f,.022f};
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
  else if(a=="--studio")o.studio=value();
  else if(a=="--studio-scale")o.studioScale=std::stof(value());
  else if(a=="--studio-az")o.studioAz=std::stof(value());
  else if(a=="--studio-el")o.studioEl=std::stof(value());
  else if(a=="--studio-target"){o.fixedStudio=true;o.studioTarget.x=std::stof(value());o.studioTarget.y=std::stof(value());o.studioTarget.z=std::stof(value());}
  else if(a=="--floor-z"){o.fixedFloor=true;o.floorHeight=std::stof(value());}
  else if(a=="--floor-color"){o.floorColor.x=std::stof(value());o.floorColor.y=std::stof(value());o.floorColor.z=std::stof(value());}
  else if(a=="--background-color"){o.backgroundColor.x=std::stof(value());o.backgroundColor.y=std::stof(value());o.backgroundColor.z=std::stof(value());}
  else {std::cerr<<"Unknown option "<<a<<"\n";std::exit(1);}
 }
 if(o.w<1||o.h<1||o.spp<1||o.depth<1||o.threads<1||o.focal<=0||o.sensorWidth<=0||o.cameraDistance<=0||o.fstop<=0){
  std::cerr<<"Invalid photographic render options\n";std::exit(1);
 }
 if(o.focusDistance<=0)o.focusDistance=o.cameraDistance;
 if(o.studio!="classic"&&o.studio!="product"){std::cerr<<"Unknown studio\n";std::exit(1);}
 return o;
}

float photoEnvStrength=.24f,photoBackgroundStrength=1.f;
Material photoFloor{V(.019f,.021f,.025f),0.f,.82f,0};
V photoBackground(.0065f,.0075f,.0095f);
float photoPixelAngle=0,photoOrthoFootprint=0;
std::vector<float> photoLightProb,photoLightCDF;

V photoEnvironment(V d,bool camera){
 // Primary background is deliberately much darker than the old inspection room.
 // Indirect environment remains non-zero so metals are never lit by black space.
 float sky=clamp(d.z*.5f+.5f);
 if(camera){
  float horizon=.88f+.12f*sky;
  return photoBackground*(photoBackgroundStrength*horizon);
 }
 V cool=V(.16f,.175f,.195f)*(0.68f+0.32f*sky);
 float horizon=std::exp(-sqr(d.z/.28f));
 V bounce=V(.12f,.105f,.09f)*(.16f*horizon);
 return (cool+bounce)*photoEnvStrength;
}

void photoPerturb(V p,V &n,Material &m,float footprint){
 // Authored finish coordinates follow each rigid part. Unresolved frequencies
 // fade into the BRDF roughness rather than sparkling independently per frame.
 if(!m.pattern)return;
 Frame fr=photoFrame(m,n,p);V q=p-m.grainOrigin,a=unit(m.grainAxis);
 float axial=dot(q,a),radius=len(q-a*axial);
 auto wave=[&](float position,float frequency){return std::sin(position*frequency)*std::exp(-.5f*sqr(frequency*footprint));};
 float grain=0;
 if(m.pattern==1||m.pattern==8){
  float coordinate=std::fabs(dot(n,a))>.8f?radius:axial;
  grain=.009f*wave(coordinate,36.f)+.003f*wave(coordinate,105.f);
 }else if(m.pattern==3||m.pattern==5||m.pattern==6){
  grain=.006f*wave(dot(q,fr.b),52.f)+.002f*wave(dot(q,fr.b),139.f);
 }else if(m.pattern==2||m.pattern==7){
  grain=.003f*wave(axial+radius*.63f,113.f)*wave(radius-axial*.41f,71.f);
 }else if(m.pattern==4){
  grain=.002f*wave(axial+radius,28.f);
 }
 n=unit(n+fr.b*grain);
 float broad=std::sin(axial*.91f+radius*.47f)*std::sin(radius*1.23f-axial*.29f);
 m.rough=clamp(m.rough*(1.f+.018f*broad),.035f,1.f);
}

V photoTrace(Ray ray,RNG&rng,int maxDepth){
 V L(0),throughput(1),prevP;float lastPDF=0,distanceTravelled=0;bool lastSpec=true;
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
     float pdf=photoLightProb[hit.light]*dist2/(light.area*std::max(1e-8f,dot(light.n,-ray.d)));
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
  distanceTravelled+=hit.t;
  photoPerturb(p,n,mat,std::max(photoOrthoFootprint,distanceTravelled*photoPixelAngle)/std::max(.15f,std::fabs(dot(n,-ray.d))));
  Frame shading=photoFrame(mat,n,p);
  V v=-ray.d;
  float selector=rng.uniform();int li=0;while(li+1<int(lights.size())&&selector>photoLightCDF[li])li++;
  const auto&light=lights[li];
  V lp=light.point(rng),delta=lp-p;float dist2=dot(delta,delta),dist=std::sqrt(dist2);V l=delta/dist;
  float nl=dot(n,l),cl=dot(light.n,-l);
  if(nl>0&&cl>0&&dot(gn,l)>0){
   float lightPDF=photoLightProb[li]*dist2/(light.area*cl);
   if(!occluded(Ray(p+gn*EPS*2,l),dist-EPS*8)){
    float bsdfPDF=photoPDF(mat,shading,v,l),w=powerHeuristic(lightPDF,bsdfPDF);
    L+=throughput*photoEval(mat,shading,v,l)*light.emission*(nl*w/lightPDF);
   }
  }
  V lnext=photoSample(mat,shading,v,rng);float cosine=dot(n,lnext),pdf=photoPDF(mat,shading,v,lnext);
  if(cosine<=0||pdf<1e-12f||dot(gn,lnext)<=0)break;
  V f=photoEval(mat,shading,v,lnext);throughput*=f*(cosine/pdf);
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
  std::vector<Material> supplied;std::string line;bool extended=false;
  while(std::getline(mf,line)){
   if(line.empty())continue;
   if(line=="CYBR_PHOTO_MATERIALS 2"){extended=true;continue;}
   std::istringstream row(line);Material m;
   if(!(row>>m.color.x>>m.color.y>>m.color.z>>m.metal>>m.rough>>m.pattern)){std::cerr<<"Malformed material\n";return 4;}
   if(extended&&!(row>>m.ior>>m.coat>>m.coatRough>>m.aniso>>m.rotation
                >>m.grainAxis.x>>m.grainAxis.y>>m.grainAxis.z
                >>m.grainOrigin.x>>m.grainOrigin.y>>m.grainOrigin.z)){std::cerr<<"Malformed photographic material\n";return 4;}
   if(m.metal<0||m.metal>1||m.rough<=0||m.rough>1||m.ior<=1||m.coat<0||m.coat>1||std::fabs(m.aniso)>1||len(m.grainAxis)<.01f){std::cerr<<"Invalid material\n";return 4;}
   supplied.push_back(m);
  }
  if(supplied.empty()){std::cerr<<"Empty material table\n";return 4;}
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
 buildSceneAcceleration();
 photoEnvStrength=opt.envStrength;photoBackgroundStrength=opt.backgroundStrength;photoFloor.rough=opt.floorRoughness;
 photoFloor.color=opt.floorColor;photoBackground=opt.backgroundColor;
 photoPixelAngle=opt.ortho?0.f:opt.sensorWidth/(opt.focal*opt.w);
 photoOrthoFootprint=opt.ortho?opt.orthoScale/opt.h:0.f;
 enableFloor=!opt.noFloor;floorZ=opt.fixedFloor?opt.floorHeight:sceneBounds.lo.z-opt.floorGap;
 V target(opt.tx,opt.ty,opt.tz);
 float az=opt.az*PI/180.f,el=opt.el*PI/180.f;
 V camera=target+V(std::cos(az)*std::cos(el),std::sin(az)*std::cos(el),std::sin(el))*opt.cameraDistance;
 V fwd=unit(target-camera),right=unit(cross(fwd,V(0,0,1))),up=cross(right,fwd);

 // Camera-relative softboxes. Sizes change independently from distance so shadow
 // softness is a controllable photographic property rather than a scene-scale accident.
 V lightTarget=opt.fixedStudio?opt.studioTarget:target;
 float laz=(opt.fixedStudio?opt.studioAz:opt.az)*PI/180.f,lel=(opt.fixedStudio?opt.studioEl:opt.el)*PI/180.f;
 V towardCamera=V(std::cos(laz)*std::cos(lel),std::sin(laz)*std::cos(lel),std::sin(lel));
 V lightRight=unit(cross(-towardCamera,V(0,0,1))),lightUp=cross(lightRight,-towardCamera);
 auto addLight=[&](V offset,float width,float height,V emission){
  Light l(lightTarget+offset*opt.studioScale,lightTarget,lightRight,width*opt.lightSize*opt.studioScale,height*opt.lightSize*opt.studioScale,emission*opt.lightIntensity);lights.push_back(l);
 };
 if(opt.studio=="product"){
  addLight(towardCamera*380.f-lightRight*340.f+lightUp*420.f,500.f,350.f,V(6.5f,6.3f,6.0f));
  addLight(towardCamera*220.f+lightRight*440.f+lightUp*90.f,300.f,500.f,V(1.7f,1.85f,2.0f));
  addLight(-towardCamera*220.f+lightRight*250.f+lightUp*450.f,220.f,500.f,V(4.2f,4.5f,4.8f));
  addLight(-towardCamera*180.f-lightRight*350.f+lightUp*160.f,90.f,360.f,V(3.1f,3.2f,3.3f));
 }else{
 addLight(towardCamera*175.f-lightRight*145.f+lightUp*170.f,250.f,175.f,V(2.25f,2.32f,2.42f));
 addLight(towardCamera*95.f+lightRight*190.f+lightUp*35.f,210.f,145.f,V(.92f,.88f,.82f));
 addLight(-towardCamera*175.f+lightRight*115.f+lightUp*145.f,205.f,90.f,V(1.70f,1.82f,2.05f));
 addLight(-towardCamera*70.f-lightRight*210.f+lightUp*20.f,95.f,235.f,V(.70f,.76f,.84f));
 }
 float totalPower=0;
 for(const auto&light:lights){float weight=light.area*dot(light.emission,V(.2126f,.7152f,.0722f));photoLightProb.push_back(weight);totalPower+=weight;}
 float accumulated=0;for(float &weight:photoLightProb){
  // An environment-only studio is valid: zero-emission softboxes must not
  // turn the sampling probabilities into NaNs.
  weight=totalPower>0?weight/totalPower:1.f/float(lights.size());
  accumulated+=weight;photoLightCDF.push_back(accumulated);
 }photoLightCDF.back()=1.f;

 float aspect=float(opt.w)/opt.h;
 float sensorHeight=opt.sensorWidth/aspect;
 float lensRadius=opt.focal/(2.f*opt.fstop);
 float orthoHalf=opt.orthoScale*.5f;
 std::vector<V> image(size_t(opt.w)*opt.h);std::vector<float> guides(image.size()*9);std::atomic<int> rows{0};
 std::vector<float> surfaces(image.size()*4);
 #pragma omp parallel for schedule(dynamic,1)
 for(int y=0;y<opt.h;y++){
  for(int x=0;x<opt.w;x++){
   uint64_t pixelSeed=(uint64_t(y)*opt.w+x)*0x9e3779b97f4a7c15ULL+opt.seed;
   RNG shiftRng(pixelSeed);float shiftX=shiftRng.uniform(),shiftY=shiftRng.uniform();V c;float lum2=0;
   for(int s=0;s<opt.spp;s++){
    RNG rng(pixelSeed+uint64_t(s+1)*0xd1b54a32d192ed03ULL);
    float jx=std::fmod((s+.5f)/opt.spp+shiftX,1.f),jy=std::fmod(radicalInverse(s)+shiftY,1.f);
    float nx=2.f*(x+jx)/opt.w-1.f,ny=1.f-2.f*(y+jy)/opt.h;
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
   V point=gd>0?primary.o+primary.d*gd:V(0);
   for(int k=0;k<3;k++)surfaces[idx*4+k]=point[k];surfaces[idx*4+3]=materialID;
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
 std::ofstream sp(opt.out+".surfaces",std::ios::binary);sp.write(reinterpret_cast<char*>(dims),8);sp.write(reinterpret_cast<char*>(surfaces.data()),surfaces.size()*sizeof(float));sp.close();
 float sec=std::chrono::duration<float>(std::chrono::steady_clock::now()-start).count();
 std::cerr<<"Wrote "<<opt.out<<"; "<<opt.w<<"x"<<opt.h<<", "<<opt.spp<<" spp, f/"<<opt.fstop<<", "<<sec<<" seconds\n";
 return 0;
}
