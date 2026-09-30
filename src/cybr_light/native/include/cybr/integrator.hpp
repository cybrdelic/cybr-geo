#pragma once
#include "scene.hpp"
namespace cybr {
struct MediumEntry {int object,material;};
// Inline storage avoids one heap allocation per spectral path. Unusually
// deep nested media spill to a dynamic array without imposing a nesting limit.
class MediumStack {
 std::array<MediumEntry,16> local;
 std::vector<MediumEntry> spill;
 size_t count=0;
 public:
 bool empty()const{return count==0;}
 size_t size()const{return count;}
 MediumEntry* begin(){return count>16?spill.data():local.data();}
 MediumEntry* end(){return begin()+count;}
 auto rbegin(){return std::reverse_iterator<MediumEntry*>(end());}
 auto rend(){return std::reverse_iterator<MediumEntry*>(begin());}
 MediumEntry& back(){return begin()[count-1];}
 MediumEntry& operator[](size_t i){return begin()[i];}
 void push_back(MediumEntry e){if(count<16)local[count]=e;else{if(count==16)spill.assign(local.begin(),local.end());spill.push_back(e);}count++;}
 void erase(MediumEntry* position){size_t i=position-begin();if(count>16){spill.erase(spill.begin()+i);count--;if(count==16){std::copy(spill.begin(),spill.end(),local.begin());spill.clear();}}else{for(size_t j=i+1;j<count;j++)local[j-1]=local[j];count--;}}
};
struct PathResult {Dual radiance;Stokes stokes{};int bounces=0;};
inline void dielectric_indices(const Scene&s,const Material&m,const Hit&h,MediumStack&stack,double nm,double&ni,double&nt){
 ni=stack.empty()?1:s.materials[stack.back().material].ior(nm);nt=ni;
 if(!m.dielectric())return;
 if(h.front)nt=m.ior(nm);
 else{auto it=std::find_if(stack.rbegin(),stack.rend(),[&](auto e){return e.object==h.object;});
  if(it!=stack.rend()){int i=int(stack.size())-1-int(it-stack.rbegin());nt=i>0?s.materials[stack[i-1].material].ior(nm):1.;}
  else{ni=m.ior(nm);nt=stack.empty()?1:s.materials[stack.back().material].ior(nm);}
 }
}
inline void transition_medium(MediumStack&stack,const Hit&h){
 if(h.front)stack.push_back({h.object,h.material});
 else{for(int j=int(stack.size())-1;j>=0;j--)if(stack[j].object==h.object){stack.erase(stack.begin()+j);break;}}
}
inline PathResult trace_path(const Scene&s,Ray ray,double nm,RNG&r){
 PathResult out;Dual throughput(1),radiance(0);Mueller polarization=Mueller::identity();Stokes stokes{};
 MediumStack stack;double previous_pdf=0;bool previous_delta=true;Vec3 previous_point;int nulls=0;
 int limit=s.settings.integrator=="direct"?1:s.settings.max_depth;
 auto add=[&](double incident,double weight){radiance=radiance+throughput*Dual(incident*weight);if(s.settings.polarized)for(int i=0;i<4;i++)stokes[i]+=polarization.at(i,0)*incident*weight;};
 if(s.settings.integrator=="ao"){
  Hit h;if(!s.bvh.hit(ray,h))return out;Vec3 d=Frame(h.n).world(cosine_hemisphere(r));
  out.radiance=s.bvh.occluded({offset(h.p,h.gn,d),d,ray.time},s.settings.ao_distance)?0:1;out.stokes[0]=out.radiance.v;return out;
 }
 for(int depth=0;limit<0||depth<=limit;depth++){
  out.bounces=depth;Hit hit;s.bvh.hit(ray,hit);
  int volume_id=-1;double event_t=hit.t;
  for(int v=0;v<int(s.volumes.size());v++){double t;if(s.volumes[v].sample(ray,event_t,nm,r,t)){event_t=t;volume_id=v;}}
  if(!stack.empty()){
   double sigma=s.materials[stack.back().material].absorption_value(nm),attenuation=sigma>0?std::exp(-sigma*event_t):1;
   throughput=throughput*Dual(attenuation);if(s.settings.polarized)polarization=polarization*attenuation;
  }
  if(volume_id>=0){
   if(limit>=0&&depth>=limit)break;
   const auto&v=s.volumes[volume_id];Vec3 point=ray.at(event_t);double albedo=v.albedo.eval(nm);throughput=throughput*Dual(albedo);if(s.settings.polarized)polarization=polarization*albedo;
   if(s.settings.nee){
    auto light=s.sample_light(point,ray.time,nm,r);
    if(light.pdf>0){
     double phase=phase_eval(v,dot(ray.d,light.wi));auto shadow=s.shadow({point+light.wi*2e-6,light.wi,ray.time},light.distance,nm,r);
     double w=light.delta?1:s.settings.mis?power_heuristic(light.pdf,phase*shadow.probability):1;
     double scale=light.radiance*w/light.pdf;radiance=radiance+throughput*Dual(phase*shadow.value*scale);
     if(s.settings.polarized){Mueller M=polarization*phase_mueller(v,-ray.d,light.wi)*shadow.pol;for(int j=0;j<4;j++)stokes[j]+=M.at(j,0)*scale;}
    }
   }
   Vec3 direction=phase_sample(v,ray.d,r);previous_pdf=phase_eval(v,dot(ray.d,direction));
   if(s.settings.polarized)polarization=polarization*phase_mueller(v,-ray.d,direction)*(1/previous_pdf);
   previous_delta=false;previous_point=point;ray={point+direction*2e-6,direction,ray.time};
  }else if(hit.primitive<0){
   double w=1;if(!previous_delta&&s.settings.nee)w=s.settings.mis?power_heuristic(previous_pdf,s.env_prob*s.environment.pdf(ray.d)):0;
   add(s.environment.radiance(ray.d,nm),w);break;
  }else{
   const Material&m=s.materials[hit.material];Vec3 wo=-ray.d;
   if(m.emission>0&&hit.front){double w=1;if(!previous_delta&&s.settings.nee)w=s.settings.mis?power_heuristic(previous_pdf,s.light_pdf(previous_point,hit,ray.d)):0;add(m.radiance(nm,hit.u,hit.v),w);}
   if(m.type==MaterialType::Emitter)break;
   double ni,nt;dielectric_indices(s,m,hit,stack,nm,ni,nt);
   // Permit null/optical sheets even at the scattering limit; they are not bounces.
   bool terminal=limit>=0&&depth>=limit;
   if(!m.delta()&&s.settings.nee&&!terminal){
    auto light=s.sample_light(hit.p,ray.time,nm,r);
    if(light.pdf>0){
     auto ev=evaluate_bsdf(m,hit,wo,light.wi,nm,ni,nt,s.settings.ad);
     if(ev.f.v>0){
      auto shadow=s.shadow({offset(hit.p,hit.gn,light.wi),light.wi,ray.time},light.distance,nm,r);
      double attenuation=1;if(!stack.empty()){double a=s.materials[stack.back().material].absorption_value(nm);if(a>0)attenuation=std::exp(-a*light.distance);}
      double w=light.delta?1:s.settings.mis?power_heuristic(light.pdf,ev.pdf*shadow.probability):1;
      double factor=std::abs(dot(hit.n,light.wi))*light.radiance*attenuation*w/light.pdf;
      radiance=radiance+throughput*ev.f*Dual(shadow.value*factor);
      if(s.settings.polarized){Mueller M=polarization*ev.pol*shadow.pol;for(int j=0;j<4;j++)stokes[j]+=M.at(j,0)*factor;}
     }
    }
   }
   BSDFSample bounce=sample_bsdf(m,hit,wo,nm,ni,nt,r,s.settings.ad,s.settings.polarized);
   if(bounce.pdf<=0||bounce.weight.v<=0||!std::isfinite(bounce.weight.v))break;
   bool null=bounce.delta&&bounce.transmission&&!bounce.medium_transition;
   if(terminal&&!null)break;
   throughput=throughput*bounce.weight;if(s.settings.polarized)polarization=polarization*bounce.pol;
   if(bounce.medium_transition)transition_medium(stack,hit);
   ray={offset(hit.p,hit.gn,bounce.wi),bounce.wi,ray.time};
   if(null){previous_pdf*=bounce.pdf;if(++nulls>1024)throw std::runtime_error("Over 1024 null surfaces in a path");depth--;continue;}
   previous_pdf=bounce.pdf;previous_delta=bounce.delta;previous_point=hit.p;
  }
  if(depth>=s.settings.rr_depth){constexpr double survival=.85;if(r.uniform()>survival)break;throughput=throughput/Dual(survival);if(s.settings.polarized)polarization=polarization*(1/survival);}
 }
 out.radiance=radiance;out.stokes=stokes;if(s.settings.polarized)out.radiance=stokes[0];return out;
}
}

