// Offline triangle-geometry path tracer for the reference reconstruction.
// C++17 / OpenMP. No neural images, image billboards, raster shadow tricks,
// screen-space occlusion, or generative textures. All visibility is ray traced.
// Includes a binned SAH BVH, smooth normals, GGX visible-normal sampling,
// diffuse/GGX mixture PDFs, area-light next-event estimation and MIS,
// camera jitter, Russian roulette, and procedural micromachining.
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>
#include <omp.h>

constexpr float PI=3.14159265358979323846f, EPS=.002f, INF=1.e30f;
struct V {
 float x=0,y=0,z=0;
 V(){} V(float a):x(a),y(a),z(a){} V(float a,float b,float c):x(a),y(b),z(c){}
 float &operator[](int i){return (&x)[i];} float operator[](int i)const{return (&x)[i];}
 V operator+(V b)const{return {x+b.x,y+b.y,z+b.z};}
 V operator-(V b)const{return {x-b.x,y-b.y,z-b.z};}
 V operator-()const{return {-x,-y,-z};}
 V operator*(float b)const{return {x*b,y*b,z*b};}
 V operator/(float b)const{return *this*(1.f/b);}
 V operator*(V b)const{return {x*b.x,y*b.y,z*b.z};}
 V &operator+=(V b){x+=b.x;y+=b.y;z+=b.z;return *this;}
 V &operator*=(V b){x*=b.x;y*=b.y;z*=b.z;return *this;}
 V &operator*=(float b){x*=b;y*=b;z*=b;return *this;}
};
inline V operator*(float a,V b){return b*a;}
inline float dot(V a,V b){return a.x*b.x+a.y*b.y+a.z*b.z;}
inline V cross(V a,V b){return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};}
inline float len(V a){return std::sqrt(dot(a,a));}
inline V unit(V a){return a/std::sqrt(std::max(1e-30f,dot(a,a)));}
inline V vmin(V a,V b){return {std::min(a.x,b.x),std::min(a.y,b.y),std::min(a.z,b.z)};}
inline V vmax(V a,V b){return {std::max(a.x,b.x),std::max(a.y,b.y),std::max(a.z,b.z)};}
inline float maxc(V a){return std::max(a.x,std::max(a.y,a.z));}
inline float clamp(float a,float lo=0.f,float hi=1.f){return std::max(lo,std::min(hi,a));}
inline V reflect(V i,V n){return i-2*dot(i,n)*n;}
inline float sqr(float x){return x*x;}
struct RNG{
 uint64_t state;
 RNG(uint64_t seed):state(seed+0x9e3779b97f4a7c15ULL){for(int i=0;i<4;i++)next();}
 uint32_t next(){state^=state>>12;state^=state<<25;state^=state>>27;return uint32_t((state*2685821657736338717ULL)>>32);}
 float uniform(){return (next()>>8)*(1.f/16777216.f);}
};
struct Ray {V o,d,inv; Ray(V a,V b):o(a),d(b),inv(1.f/b.x,1.f/b.y,1.f/b.z){} };
struct Box{
 V lo{INF},hi{-INF};
 void grow(V v){lo=vmin(lo,v);hi=vmax(hi,v);}
 void grow(const Box& b){lo=vmin(lo,b.lo);hi=vmax(hi,b.hi);}
 float area()const{V d=vmax(hi-lo,V(0));return 2*(d.x*d.y+d.x*d.z+d.y*d.z);}
 bool hit(const Ray&r,float maxT,float *entry=nullptr)const {
  float t0=EPS,t1=maxT;
  for(int k=0;k<3;k++){
   float a=(lo[k]-r.o[k])*r.inv[k],b=(hi[k]-r.o[k])*r.inv[k];
   if(a>b)std::swap(a,b);t0=std::max(t0,a);t1=std::min(t1,b);
   if(t0>t1)return false;
  }
  if(entry)*entry=t0;return true;
 }
};
struct Tri{
 V p,e1,e2,n0,n1,n2; int mat=0,group=0;
 V centroid()const{return p+(e1+e2)*(1.f/3);}
 Box bounds()const{Box b;b.grow(p);b.grow(p+e1);b.grow(p+e2);return b;}
 bool hit(const Ray&r,float maxT,float &t,float &u,float &v)const{
  V pvec=cross(r.d,e2);float det=dot(e1,pvec);
  if(std::fabs(det)<1e-10f)return false;
  float inv=1.f/det;V tv=r.o-p;u=dot(tv,pvec)*inv;
  if(u<0||u>1)return false;
  V q=cross(tv,e1);v=dot(r.d,q)*inv;if(v<0||u+v>1)return false;
  t=dot(e2,q)*inv;return t>EPS&&t<maxT;
 }
};
struct Node {Box b;int left=-1,right=-1,start=0,count=0;};
struct Hit {float t=INF,u=0,v=0;int tri=-1,light=-1;bool floor=false;};
std::vector<Tri> tris;std::vector<int> order;std::vector<Node> nodes;
int build(int start,int count){
 int ni=nodes.size();nodes.emplace_back();Box bounds,cb;
 for(int j=start;j<start+count;j++){bounds.grow(tris[order[j]].bounds());cb.grow(tris[order[j]].centroid());}
 nodes[ni].b=bounds;
 if(count<=5){nodes[ni].start=start;nodes[ni].count=count;return ni;}
 constexpr int NB=12;
 float best=INF;int ba=-1,bb=-1;
 for(int ax=0;ax<3;ax++){
  float extent=cb.hi[ax]-cb.lo[ax];if(extent<1e-5f)continue;
  Box bins[NB];int cnt[NB]={};
  for(int j=start;j<start+count;j++){
   const auto &tr=tris[order[j]];int b=std::min(NB-1,int(NB*(tr.centroid()[ax]-cb.lo[ax])/extent));
   bins[b].grow(tr.bounds());cnt[b]++;
  }
  Box left[NB-1],right[NB-1];int lc[NB-1],rc[NB-1];Box cur;int n=0;
  for(int k=0;k<NB-1;k++){cur.grow(bins[k]);n+=cnt[k];left[k]=cur;lc[k]=n;}
  cur=Box();n=0;
  for(int k=NB-1;k>0;k--){cur.grow(bins[k]);n+=cnt[k];right[k-1]=cur;rc[k-1]=n;}
  for(int k=0;k<NB-1;k++)if(lc[k]&&rc[k]){
   float c=left[k].area()*lc[k]+right[k].area()*rc[k];
   if(c<best){best=c;ba=ax;bb=k;}
  }
 }
 if(ba<0){nodes[ni].start=start;nodes[ni].count=count;return ni;}
 float split=cb.lo[ba]+(cb.hi[ba]-cb.lo[ba])*(float(bb+1)/NB);
 auto it=std::partition(order.begin()+start,order.begin()+start+count,[&](int id){return tris[id].centroid()[ba]<split;});
 int mid=int(it-order.begin());
 if(mid==start||mid==start+count)mid=start+count/2;
 int l=build(start,mid-start),r=build(mid,start+count-mid);
 nodes[ni].left=l;nodes[ni].right=r;return ni;
}
bool meshHit(const Ray&r,Hit &h,bool any=false){
 int stack[96],sp=0;stack[sp++]=0;bool found=false;
 while(sp){
  int ni=stack[--sp];const auto&nd=nodes[ni];
  if(!nd.b.hit(r,h.t))continue;
  if(nd.count){
   for(int i=nd.start;i<nd.start+nd.count;i++){
    int id=order[i];float t,u,v;
    if(tris[id].hit(r,h.t,t,u,v)){h.t=t;h.u=u;h.v=v;h.tri=id;h.floor=false;h.light=-1;found=true;if(any)return true;}
   }
  }else{
   float a,b;bool ha=nodes[nd.left].b.hit(r,h.t,&a),hb=nodes[nd.right].b.hit(r,h.t,&b);
   if(ha&&hb){if(a<b){stack[sp++]=nd.right;stack[sp++]=nd.left;}else{stack[sp++]=nd.left;stack[sp++]=nd.right;}}
   else if(ha)stack[sp++]=nd.left;else if(hb)stack[sp++]=nd.right;
  }
 }
 return found;
}
struct Light{
 V c,u,v,n,emission;float area;
 Light(V center,V target,V across,float width,float height,V e):c(center),emission(e){
  n=unit(target-center);u=unit(across-n*dot(across,n));v=unit(cross(n,u));
  u*=width*.5f;v*=height*.5f;area=width*height;
 }
 bool hit(const Ray&r,float maxT,float &t)const{
  float den=dot(r.d,n);if(std::fabs(den)<1e-8f)return false;
  t=dot(c-r.o,n)/den;if(t<EPS||t>=maxT)return false;
  V q=r.o+r.d*t-c;
  return std::fabs(dot(q,u))<=dot(u,u)&&std::fabs(dot(q,v))<=dot(v,v);
 }
 V point(RNG &rng)const{return c+u*(2*rng.uniform()-1)+v*(2*rng.uniform()-1);}
};
std::vector<Light> lights;
float floorZ=-55.6f;bool enableFloor=true;float exposure=1.f;
struct Material{V color;float metal,rough;int pattern;};
Material mats[]={
 {{.53f,.55f,.56f},1,.27f,2},{{.64f,.66f,.67f},1,.22f,1},
 {{.32f,.33f,.34f},1,.25f,3},{{.69f,.71f,.73f},1,.16f,1},
 {{.095f,.11f,.125f},.88f,.26f,1},{{.29f,.215f,.105f},.93f,.29f,1},
 {{.022f,.026f,.029f},.55f,.39f,0},{{.014f,.017f,.019f},0,.48f,0},
 {{.037f,.041f,.044f},.10f,.46f,4}
};
V environment(V d,bool camera=false){
 if(camera)return V(.008f,.011f,.013f)*(1.0f+.3f*std::max(d.z,0.f));
 float sky=clamp(d.z*.5f+.5f);
 return V(.25f,.26f,.27f)*(.70f+.30f*sky);
}
bool sceneHit(const Ray&r,Hit &h,bool seeLights=true){
 bool found=meshHit(r,h);
 if(enableFloor&&r.d.z<-.000001f){float t=(floorZ-r.o.z)/r.d.z;if(t>EPS&&t<h.t){h.t=t;h.tri=-1;h.floor=true;h.light=-1;found=true;}}
 if(seeLights)for(int i=0;i<int(lights.size());i++){float t;if(lights[i].hit(r,h.t,t)){h.t=t;h.tri=-1;h.floor=false;h.light=i;found=true;}}
 return found;
}
bool occluded(const Ray&r,float dist){
 if(enableFloor&&r.d.z<-1e-6f){float t=(floorZ-r.o.z)/r.d.z;if(t>EPS&&t<dist)return true;}
 Hit h;h.t=dist;return meshHit(r,h,true);
}
struct Frame{
 V t,b,n;
 Frame(V normal):n(normal){t=unit(cross(std::fabs(n.z)<.99f?V(0,0,1):V(0,1,0),n));b=cross(n,t);}
 V local(V a)const{return {dot(a,t),dot(a,b),dot(a,n)};}
 V world(V a)const{return t*a.x+b*a.y+n*a.z;}
};
float G1(float cosine,float alpha){return cosine>0?2*cosine/(cosine+std::sqrt(alpha*alpha+(1-alpha*alpha)*cosine*cosine)):0;}
float Dggx(float nH,float alpha){float a2=alpha*alpha;return a2/(PI*sqr(nH*nH*(a2-1)+1));}
V fresnel(float vH,V f0){float f=std::pow(1-clamp(vH),5);return f0+(V(1)-f0)*f;}
float specProb(const Material&m){return m.metal>.7f?.96f:.35f;}
V eval(const Material&m,V n,V v,V l){
 float nv=dot(n,v),nl=dot(n,l);if(nv<=0||nl<=0)return V(0);
 V h=unit(v+l);float nh=clamp(dot(n,h)),vh=clamp(dot(v,h));
 float a=std::max(.015f,m.rough*m.rough);
 V f0=m.color*m.metal+V(.04f)*(1-m.metal),F=fresnel(vh,f0);
 V s=F*(Dggx(nh,a)*G1(nv,a)*G1(nl,a)/(4*nv*nl));
 V d=m.color*((1-m.metal)/PI)*(V(1)-F);
 return s+d;
}
float pdfBSDF(const Material&m,V n,V v,V l){
 float nv=dot(n,v),nl=dot(n,l);if(nv<=0||nl<=0)return 0;
 V h=unit(v+l);float a=std::max(.015f,m.rough*m.rough),ps=specProb(m);
 return ps*(Dggx(clamp(dot(n,h)),a)*G1(nv,a)/(4*nv))+(1-ps)*nl/PI;
}
V sampleBSDF(const Material&m,V n,V v,RNG&rng){
 Frame fr(n);V vv=fr.local(v);
 if(rng.uniform()>specProb(m)){
  float u=rng.uniform(),phi=2*PI*rng.uniform(),r=std::sqrt(u);
  return fr.world({r*std::cos(phi),r*std::sin(phi),std::sqrt(1-u)});
 }
 float alpha=std::max(.015f,m.rough*m.rough);
 V vh=unit(V(alpha*vv.x,alpha*vv.y,std::max(0.f,vv.z)));
 float lensq=vh.x*vh.x+vh.y*vh.y;
 V t1=lensq>0?V(-vh.y,vh.x,0)/std::sqrt(lensq):V(1,0,0),t2=cross(vh,t1);
 float r=std::sqrt(rng.uniform()),phi=2*PI*rng.uniform();
 float x=r*std::cos(phi),y=r*std::sin(phi),s=.5f*(1+vh.z);
 y=(1-s)*std::sqrt(std::max(0.f,1-x*x))+s*y;
 V nh=t1*x+t2*y+vh*std::sqrt(std::max(0.f,1-x*x-y*y));
 V h=unit(V(alpha*nh.x,alpha*nh.y,std::max(0.f,nh.z)));
 return fr.world(reflect(-vv,h));
}
float powerHeuristic(float a,float b){return a*a/(a*a+b*b+1e-30f);}

void perturb(V p,V &n,Material &m){
 // Imperceptible-scale machining, not high-contrast fake geometric edges.
 if(m.pattern==1){
  float radius=std::sqrt(p.y*p.y+p.z*p.z);
  if(std::fabs(n.x)>.92f&&radius>1){
   V radial(0,p.y/radius,p.z/radius);radial=unit(radial-n*dot(radial,n));
   float gr=std::sin(radius*74.0f)*.007f+std::sin(radius*411.1f)*.0035f;
   n=unit(n+radial*gr);m.rough*=1+.045f*std::sin(radius*37.3f);
  }else{
   m.rough*=1+.025f*std::sin(p.x*98.f);
  }
 }else if(m.pattern==2){
  float q=std::sin(p.x*113.31f+p.y*29.61f+p.z*163.19f)*std::sin(p.x*41.7f-p.y*153.4f);
  m.rough*=1+.065f*q;
 }else if(m.pattern==4){
  float q=std::sin(p.x*5.1f+p.y*8.33f)*std::sin(p.y*7.42f-p.x*9.21f);
  m.color*=1+.07f*q;
 }
}

V trace(Ray ray,RNG&rng,int maxDepth){
 V L(0),throughput(1),prevP;float lastPDF=0;bool lastSpec=true;
 for(int depth=0;depth<maxDepth;depth++){
  Hit hit;
  if(!sceneHit(ray,hit,depth>0)){L+=throughput*environment(ray.d,depth==0);break;}
  V p=ray.o+ray.d*hit.t;
  if(hit.light>=0){
   const auto&light=lights[hit.light];
   if(dot(light.n,-ray.d)>0){
    float w=1;
    if(depth>0&&!lastSpec){float dist2=dot(p-prevP,p-prevP);float pdf=dist2/(light.area*std::max(1e-8f,dot(light.n,-ray.d))*lights.size());w=powerHeuristic(lastPDF,pdf);}
    L+=throughput*light.emission*w;
   }
   break;
  }
  V n,gn;Material mat;
  if(hit.floor){n=gn=V(0,0,1);mat=mats[8];}
  else{
   const auto&t=tris[hit.tri];n=unit(t.n0*(1-hit.u-hit.v)+t.n1*hit.u+t.n2*hit.v);gn=unit(cross(t.e1,t.e2));mat=mats[t.mat];
   if(dot(gn,-ray.d)<0)gn=-gn;
   if(dot(n,gn)<0)n=-n;
   // Correct interpolated normals if they point through the visible surface.
   if(dot(n,-ray.d)<.02f)n=unit(n+gn*(.021f-dot(n,-ray.d)));
  }
  perturb(p,n,mat);
  V v=-ray.d;
  // One uniformly selected emitter. BSDF sampling complements glossy direct
  // illumination; the power heuristic prevents double counting.
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
  // Remaining light transport comes from sampled paths, including the room
  // environment, visible reflection cards and multiple metal reflections.
  V lnext=sampleBSDF(mat,n,v,rng);
  float cosine=dot(n,lnext),pdf=pdfBSDF(mat,n,v,lnext);
  if(cosine<=0||pdf<1e-12f||dot(gn,lnext)<=0)break;
  V f=eval(mat,n,v,lnext);throughput*=f*(cosine/pdf);
  if(!std::isfinite(maxc(throughput)))break;
  if(depth>=3){float q=clamp(maxc(throughput),.08f,.93f);if(rng.uniform()>q)break;throughput*=1/q;}
  prevP=p;lastPDF=pdf;lastSpec=false;
  ray=Ray(p+gn*EPS*2,unit(lnext));
 }
 return L;
}

V rotateY(V p,float a){return {std::cos(a)*p.x+std::sin(a)*p.z,p.y,-std::sin(a)*p.x+std::cos(a)*p.z};}
struct Options{
 std::string mesh,out,view="hero";int w=1000,h=900,spp=32,depth=7,threads=5,seed=2026;
 float az=9999.f,el=18.5f,explode=0,exposure=1.f;
 bool ortho=false;
};
Options parse(int argc,char**argv){
 Options o;
 if(argc<3){std::cerr<<"usage: pathtrace scene.meshbin output.ppm [--w 1400 --h 1200 --spp 128 --view hero|rear|front|side|core|exploded --az 230 --el 18.5 --threads 5 --exposure 1]\n";exit(1);}
 o.mesh=argv[1];o.out=argv[2];
 for(int i=3;i<argc;i++){
  std::string a=argv[i];auto value=[&](){if(i+1>=argc){std::cerr<<"Missing value "<<a<<"\n";exit(1);}return std::string(argv[++i]);};
  if(a=="--w")o.w=std::stoi(value());else if(a=="--h")o.h=std::stoi(value());else if(a=="--spp")o.spp=std::stoi(value());
  else if(a=="--depth")o.depth=std::stoi(value());else if(a=="--threads")o.threads=std::stoi(value());else if(a=="--seed")o.seed=std::stoi(value());
  else if(a=="--view")o.view=value();else if(a=="--az")o.az=std::stof(value());else if(a=="--el")o.el=std::stof(value());else if(a=="--exposure")o.exposure=std::stof(value());
  else if(a=="--explode")o.explode=std::stof(value());else if(a=="--ortho")o.ortho=true;else {std::cerr<<"Unknown option "<<a<<"\n";exit(1);}
 }
 if(o.w<1||o.h<1||o.spp<1||o.threads<1)exit(1);
 return o;
}
float aces(float x){return clamp(x*(2.51f*x+.03f)/(x*(2.43f*x+.59f)+.14f));}
int main(int argc,char**argv){
 auto opt=parse(argc,argv);omp_set_num_threads(opt.threads);auto start=std::chrono::steady_clock::now();
 std::ifstream in(opt.mesh,std::ios::binary);if(!in){std::cerr<<"Cannot read mesh\n";return 2;}
 uint32_t n=0;in.read(reinterpret_cast<char*>(&n),4);tris.reserve(n);
 // Group IDs are the order in manifest: carrier, front_flange, rear_flange,
 // front_hub, shaft, rear_hub, gears, clutch, bearing, marking.
 for(uint32_t i=0;i<n;i++){
  float a[20];in.read(reinterpret_cast<char*>(a),80);if(!in){std::cerr<<"Truncated mesh\n";return 3;}
  int mat=int(a[18]),g=int(a[19]);
  if(opt.view=="core"&&(g==0||g==1||g==2||g==3||g==5||g==9))continue;
  V p0(a[0],a[1],a[2]),p1(a[3],a[4],a[5]),p2(a[6],a[7],a[8]);
  V n0(a[9],a[10],a[11]),n1(a[12],a[13],a[14]),n2(a[15],a[16],a[17]);
  float shift=0;
  if(opt.view=="exploded"||opt.explode>0){
   float s=opt.explode>0?opt.explode:38;
   if(g==1)shift=-s*1.20f;else if(g==3)shift=-s*2.00f;else if(g==2)shift=s*1.25f;
   else if(g==5)shift=s*2.1f;else if(g==7)shift=s*.50f;else if(g==8)shift=p0.x<0?-s*.50f:s*.85f;
   else if(g==9)shift=0;
   p0.x+=shift;p1.x+=shift;p2.x+=shift;
  }
  float angle=-.058f;
  p0=rotateY(p0,angle);p1=rotateY(p1,angle);p2=rotateY(p2,angle);
  n0=rotateY(n0,angle);n1=rotateY(n1,angle);n2=rotateY(n2,angle);
  Tri t;t.p=p0;t.e1=p1-p0;t.e2=p2-p0;t.n0=n0;t.n1=n1;t.n2=n2;t.mat=mat;t.group=g;
  if(dot(cross(t.e1,t.e2),cross(t.e1,t.e2))>1e-17f)tris.push_back(t);
 }
 std::cerr<<"Triangles "<<tris.size()<<"\n";
 order.resize(tris.size());std::iota(order.begin(),order.end(),0);nodes.reserve(tris.size()/2);build(0,order.size());
 std::cerr<<"BVH nodes "<<nodes.size()<<"\n";
 lights.emplace_back(V(-55,-95,190),V(0,0,0),V(1,0,0),185,120,V(1.75f,1.80f,1.86f));
 lights.emplace_back(V(-165,65,100),V(-10,0,0),V(0,0,1),170,43,V(2.7f,2.7f,2.65f));
 lights.emplace_back(V(95,100,135),V(0,0,0),V(1,0,0),150,100,V(2.65f,2.7f,2.75f));
 lights.emplace_back(V(-205,-165,42),V(-10,0,0),V(0,0,1),135,105,V(1.1f,1.12f,1.15f));
 lights.emplace_back(V(-150,150,-15),V(-45,0,0),V(0,0,1),165,160,V(.36f,.37f,.38f));
 V target(0,0,-1),camera(-230,-310,136);
 float scale=183.f;bool ortho=opt.ortho;
 if(opt.view=="rear")camera=V(235,-315,128);
 if(opt.view=="front"){camera=V(-430,0,5);target=V(-25,0,-1.5);ortho=true;scale=145;}
 if(opt.view=="side"){camera=V(0,-450,12);ortho=true;scale=148;}
 if(opt.view=="core"){camera=V(-220,-310,148);scale=132;floorZ=-47;}
 if(opt.view=="exploded"){camera=V(-220,-470,180);scale=185;ortho=true;enableFloor=false;}
 if(opt.az<1000.f){
  float az=opt.az*PI/180,el=opt.el*PI/180;
  camera=target+V(std::cos(az)*std::cos(el),std::sin(az)*std::cos(el),std::sin(el))*420.f;
 }
 V fwd=unit(target-camera),right=unit(cross(fwd,V(0,0,1))),up=cross(right,fwd);
 float distance=len(target-camera),sensor=scale*.5f,dscale=sensor/distance,aspect=float(opt.w)/opt.h;
 std::vector<V> image(size_t(opt.w)*opt.h);std::vector<float> guides(image.size()*9);std::atomic<int> rows{0};
 #pragma omp parallel for schedule(dynamic,1)
 for(int y=0;y<opt.h;y++){
  for(int x=0;x<opt.w;x++){
   RNG rng((uint64_t(y)*opt.w+x)*0x9e3779b97f4a7c15ULL+opt.seed);V c;float lum2=0;
   for(int s=0;s<opt.spp;s++){
    float dx=(2*(x+rng.uniform())/opt.w-1)*aspect,dy=(1-2*(y+rng.uniform())/opt.h);
    Ray ray=ortho?Ray(camera+right*(dx*sensor)+up*(dy*sensor),fwd):Ray(camera,unit(fwd+right*(dx*dscale)+up*(dy*dscale)));
    V sample=trace(ray,rng,opt.depth); c+=sample;float lum=dot(sample,V(.2126f,.7152f,.0722f));lum2+=lum*lum;
   }
   size_t idx=size_t(y)*opt.w+x;
   image[idx]=c/float(opt.spp);
   float dx=(2*(x+.5f)/opt.w-1)*aspect,dy=1-2*(y+.5f)/opt.h;
   Ray primary=ortho?Ray(camera+right*(dx*sensor)+up*(dy*sensor),fwd):Ray(camera,unit(fwd+right*(dx*dscale)+up*(dy*dscale)));
   Hit first;V guideN,guideA;float gd=0,materialID=-1;
   if(sceneHit(primary,first,false)){
    gd=first.t;
    if(first.floor){guideN=V(0,0,1);guideA=mats[8].color;materialID=8;}
    else if(first.tri>=0){const auto&t=tris[first.tri];guideN=unit(t.n0*(1-first.u-first.v)+t.n1*first.u+t.n2*first.v);guideA=mats[t.mat].color;materialID=t.mat;}
   }
   float avg=dot(image[idx],V(.2126f,.7152f,.0722f));
   float var=std::max(0.f,(lum2/opt.spp-avg*avg)/std::max(1,opt.spp-1));
   for(int k=0;k<3;k++){guides[idx*9+k]=guideN[k];guides[idx*9+3+k]=guideA[k];}
   guides[idx*9+6]=gd;guides[idx*9+7]=var;guides[idx*9+8]=materialID;
  }
  int done=++rows;
  if(done%std::max(1,opt.h/10)==0){float sec=std::chrono::duration<float>(std::chrono::steady_clock::now()-start).count();
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
 std::cerr<<"Wrote "<<opt.out<<"; "<<opt.w<<"x"<<opt.h<<", "<<opt.spp<<" spp, "<<sec<<" seconds\n";
}
