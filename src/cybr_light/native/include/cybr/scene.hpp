#pragma once
#include "material.hpp"
#include "volume.hpp"
namespace cybr {
struct EnvironmentLobe {Vec3 direction;double exponent=30,strength=1,kelvin=6500;};
struct Environment {
 Spectrum base{Vec3(.2,.24,.3)};double strength=0;bool flat=false;double rotation=0;
 std::shared_ptr<ImageTexture> texture;
 std::vector<EnvironmentLobe> lobes;std::vector<double> cdf,probability;
 static constexpr int width=256,height=128;
 double radiance(Vec3 d,double nm)const{
  double v=strength*base.eval(nm)*(flat?1:(.3+.7*std::max(0.,d.y)));
  if(texture){double u=std::atan2(d.z,d.x)/(2*pi)+rotation/(2*pi);u-=std::floor(u);double t=std::acos(clamp(d.y,-1,1))/pi;v=strength*texture->eval(u,t,nm);}
  for(auto&l:lobes)v+=l.strength*std::pow(std::max(0.,dot(d,l.direction)),l.exponent)*blackbody_normalized(nm,l.kelvin);return v;
 }
 void build(){cdf.resize(width*height);probability.resize(width*height);double sum=0;for(int y=0;y<height;y++){double c0=std::cos(pi*y/height),c1=std::cos(pi*(y+1)/height),c=.5*(c0+c1),s=std::sqrt(1-c*c),area=2*pi/width*(c0-c1);for(int x=0;x<width;x++){double p=2*pi*(x+.5)/width;Vec3 d{s*std::cos(p),c,s*std::sin(p)};double value=(radiance(d,450)+radiance(d,550)+radiance(d,650))/3;sum+=probability[y*width+x]=std::max(1e-8,value)*area;cdf[y*width+x]=sum;}}for(double&v:cdf)v/=sum;for(double&v:probability)v/=sum;}
 double pdf(Vec3 d)const{if(probability.empty())return 0;int y=std::min(height-1,int(std::acos(clamp(d.y,-1,1))*height/pi));double p=std::atan2(d.z,d.x);if(p<0)p+=2*pi;int x=std::min(width-1,int(p*width/(2*pi)));double area=2*pi/width*(std::cos(pi*y/height)-std::cos(pi*(y+1)/height));return probability[y*width+x]/area;}
 Vec3 sample(RNG&r,double& density)const{int idx=std::min(int(cdf.size())-1,int(std::lower_bound(cdf.begin(),cdf.end(),r.uniform())-cdf.begin()));int y=idx/width,x=idx%width;double c0=std::cos(pi*y/height),c1=std::cos(pi*(y+1)/height),c=c0+(c1-c0)*r.uniform(),s=std::sqrt(1-c*c),p=2*pi*(x+r.uniform())/width;density=probability[idx]/(2*pi/width*(c0-c1));return {s*std::cos(p),c,s*std::sin(p)};}
 bool active()const{return strength>0||!lobes.empty();}
};
struct DeltaEmitter {
 // kind 0 point: intensity [W/sr]; 1 directional: irradiance [W/m^2]; 2 spot.
 int kind=0;Vec3 position,direction{0,-1,0};Spectrum intensity{1.};double scale=1,cutoff=pi/6,beam=pi/8;
 double falloff(Vec3 from_light)const{if(kind!=2)return 1;double c=dot(direction,from_light),a=std::cos(cutoff),b=std::cos(beam);if(c<=a)return 0;if(c>=b)return 1;return (c-a)/(b-a);}
};
struct LightSample {Vec3 wi;double distance=inf,pdf=0,radiance=0;bool delta=false;};
struct RenderSettings {
 int width=640,height=480,spp=96,bands=8,max_depth=16,rr_depth=5,threads=4,active_material=-1;
 uint64_t seed=1337;bool polarized=false,ad=false,mis=true,nee=true;double exposure=1.;std::string observer_path;
 std::string integrator="path",filter="box",film_format="pfm";int sampler=0;double ao_distance=1,photon_radius=.15;uint64_t photon_count=200000;
};
struct ShadowResult {double value=1,probability=1;Mueller pol=Mueller::identity();};
inline ShadowResult sheet_transmission(const Material&m,const Hit&h,Vec3 direction,double nm){
 auto e=null_component(m,h,-direction,nm,false);return {e.numerator.v,e.pdf,e.pol};
}

struct Scene {
 std::vector<std::string> dependencies;std::vector<std::unique_ptr<Meshlet>> meshlets;std::vector<Primitive> primitives;std::vector<Material> materials;std::vector<Volume> volumes;std::vector<DeltaEmitter> delta_lights;
 Camera camera;Environment environment;BVH bvh;Observer observer;RenderSettings settings;
 std::vector<int> lights,light_index;std::vector<double> light_probs,light_cdf,delta_probs,delta_cdf;double env_prob=0;
 void build(){
  for(const auto&volume:volumes)volume.validate();
  for(auto&p:primitives){if(p.material<0||p.material>=int(materials.size()))throw std::runtime_error("Unknown material on primitive");if(p.area()<=1e-15||!std::isfinite(p.area()))throw std::runtime_error("Degenerate or nonfinite geometry");}
  bvh.build(primitives,std::min({0.,camera.shutter_open,camera.shutter_close}),std::max({1.,camera.shutter_open,camera.shutter_close}));environment.build();
  lights.clear();light_index.assign(primitives.size(),-1);light_probs.clear();light_cdf.clear();delta_probs.clear();delta_cdf.clear();double total=0;
  for(size_t i=0;i<primitives.size();i++){const auto&m=materials[primitives[i].material];if(m.emission>0){light_index[i]=int(lights.size());lights.push_back(int(i));double w=primitives[i].area()*m.emission*std::max(.01,m.color.eval(550));light_probs.push_back(w);total+=w;}}
  for(auto&l:delta_lights){double w=std::max(1e-10,l.scale*l.intensity.eval(550)*4*pi);delta_probs.push_back(w);total+=w;}
  double env_weight=environment.active()?(total==0?1:total*.25):0;total+=env_weight;env_prob=total>0?env_weight/total:0;
  double acc=0;for(double&w:light_probs){w/=total;acc+=w;light_cdf.push_back(acc);}for(double&w:delta_probs){w/=total;acc+=w;delta_cdf.push_back(acc);}
  if(!settings.observer_path.empty()){observer.table.clear();observer.load(settings.observer_path);}
  if(settings.ad&&settings.polarized)throw std::runtime_error("Combined native forward AD and polarized transport is not supported");
  std::function<void(Material&,int)> mark=[&](Material&m,int fallback){
   m.differentiable=((m.source_index<0?fallback:m.source_index)==settings.active_material);
   if(m.type==MaterialType::Blend&&((m.child1&&m.child1->dielectric())||(m.child2&&m.child2->dielectric())))throw std::runtime_error("Blending distinct dielectric medium transitions is unsupported");
   if(m.child1)mark(*m.child1,fallback);if(m.child2)mark(*m.child2,fallback);
  };
  for(int i=0;i<int(materials.size());i++)mark(materials[i],i);
 }
 ShadowResult shadow(const Ray&r,double distance,double nm,RNG&rng)const{
  ShadowResult result;Ray ray=r;double remaining=distance;
  for(int count=0;count<1024;count++){
   Hit h;h.t=std::isfinite(remaining)?std::max(0.,remaining-2e-5):remaining;
   if(!bvh.hit(ray,h))break;
   auto tr=sheet_transmission(materials[h.material],h,ray.d,nm);
   if(tr.value<=0){result.value=0;result.probability=0;result.pol=Mueller{};return result;}
   result.value*=tr.value;result.probability*=tr.probability;result.pol=result.pol*tr.pol;
   Vec3 next=offset(h.p,h.gn,ray.d);remaining-=h.t+norm(next-h.p);if(remaining<=0)break;ray.o=next;
   if(count==1023)throw std::runtime_error("Transparent shadow crossed over 1024 surfaces");
  }
  double tr=1;for(auto&v:volumes)tr*=v.transmittance(r,distance,nm,rng);result.value*=tr;result.pol=result.pol*tr;return result;
 }
 double shadow_transmittance(const Ray&r,double distance,double nm,RNG&rng)const{return shadow(r,distance,nm,rng).value;}
 LightSample sample_light(Vec3 p,double time,double nm,RNG&r)const{
  LightSample s;if(lights.empty()&&delta_lights.empty()&&!environment.active())return s;
  double u=r.uniform();auto it=std::lower_bound(light_cdf.begin(),light_cdf.end(),u);
  if(it==light_cdf.end()){
   auto dt=std::lower_bound(delta_cdf.begin(),delta_cdf.end(),u);
   if(dt!=delta_cdf.end()){
    int k=int(dt-delta_cdf.begin());auto&l=delta_lights[k];s.delta=true;s.pdf=delta_probs[k];
    if(l.kind==1){s.wi=-l.direction;s.radiance=l.intensity.eval(nm)*l.scale;}
    else{Vec3 d=l.position-p;s.distance=norm(d);if(s.distance<1e-9)return LightSample{};s.wi=d/s.distance;s.radiance=l.intensity.eval(nm)*l.scale/sqr(s.distance)*l.falloff(-s.wi);}
    return s;
   }
   if(env_prob<=0)return s;s.wi=environment.sample(r,s.pdf);s.pdf*=env_prob;s.radiance=environment.radiance(s.wi,nm);return s;
  }
  int index=int(it-light_cdf.begin()),id=lights[index];Vec3 q,n;primitives[id].sample(r,time,q,n);Vec3 d=q-p;double dist2=norm2(d);s.distance=std::sqrt(dist2);if(s.distance<=1e-8)return s;s.wi=d/s.distance;double cosine=dot(n,-s.wi);if(cosine<=1e-9)return s;s.pdf=light_probs[index]*dist2/(cosine*primitives[id].area());Hit lh;Ray lr;lr.o=p;lr.d=s.wi;lr.time=time;primitives[id].hit(lr,1e-9,lh);s.radiance=materials[primitives[id].material].radiance(nm,lh.u,lh.v);return s;
 }
 double light_pdf(Vec3 origin,const Hit&h,Vec3 direction)const{
  if(h.primitive<0||h.primitive>=int(light_index.size())||!h.front)return 0;int k=light_index[h.primitive];if(k<0)return 0;double c=std::abs(dot(h.gn,-direction));return light_probs[k]*norm2(h.p-origin)/(std::max(1e-15,c)*primitives[h.primitive].area());
 }
};
inline Mueller phase_mueller(const Volume&v,Vec3 wo,Vec3 wi){
 double c=-dot(wo,wi);if(v.phase!=1)return Mueller::depolarizer(phase_eval(v,c));
 Mueller m;double a=3/(16*pi);m.at(0,0)=m.at(1,1)=a*(1+c*c);m.at(0,1)=m.at(1,0)=a*(1-c*c);m.at(2,2)=m.at(3,3)=2*a*c;
 Vec3 n=normalize(wo+wi);return frame_scattering(m,wo,wi,n);
}
}

