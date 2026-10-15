#pragma once
#include "integrator.hpp"
#include <unordered_map>
namespace cybr {
struct Photon {Vec3 p,wi,normal;double flux;int object;};
struct PhotonCell {int x,y,z;bool operator==(const PhotonCell&o)const{return x==o.x&&y==o.y&&z==o.z;}};
struct PhotonCellHash {size_t operator()(const PhotonCell&c)const{return mix64(uint64_t(int64_t(c.x))*73856093ULL^uint64_t(int64_t(c.y))*19349663ULL^uint64_t(int64_t(c.z))*83492791ULL);}};
class PhotonMap {
 using Grid=std::unordered_map<PhotonCell,std::vector<Photon>,PhotonCellHash>;
 std::vector<Grid> grids;double radius=.1;uint64_t emitted=0,stored=0;int bands=0;
 PhotonCell cell(Vec3 p)const{return {int(std::floor(p.x/radius)),int(std::floor(p.y/radius)),int(std::floor(p.z/radius))};}
public:
 uint64_t emitted_count()const{return emitted;}uint64_t stored_count()const{return stored;}
 double wavelength(int band)const{return 360+(band+.5)*470/bands;}
 void build(const Scene&s){
  if(!s.volumes.empty()||s.settings.polarized||s.settings.ad)throw std::runtime_error("Surface photon mapping does not support volumes, polarization or native AD");
  for(auto&p:s.primitives)if(norm2(p.geometry().velocity)>0)throw std::runtime_error("Photon mapping currently requires static geometry");
  radius=s.settings.photon_radius;bands=s.settings.bands;if(radius<=0||bands<1)throw std::runtime_error("Invalid photon settings");grids.resize(bands);
  std::vector<double> cdf;std::vector<int> kind,index;double total=0;
  auto add=[&](int k,int i,double w){if(w>0){total+=w;cdf.push_back(total);kind.push_back(k);index.push_back(i);}};
  for(int id:s.lights)add(0,id,s.primitives[id].area()*s.materials[s.primitives[id].material].emission*pi);
  for(int i=0;i<int(s.delta_lights.size());i++)add(1,i,s.delta_lights[i].scale*4*pi);
  if(s.environment.active())add(2,0,total>0?total*.25:1);
  if(cdf.empty())throw std::runtime_error("Photon mapping requires an emitter");for(double&v:cdf)v/=total;
  Bounds bounds;for(auto&p:s.primitives)bounds.grow(p.bounds());Vec3 center=bounds.center();double scene_radius=std::max(1.,norm(bounds.hi-bounds.lo)*.51);
  uint64_t per_band=(s.settings.photon_count+bands-1)/bands;emitted=per_band*bands;stored=0;
  for(int band=0;band<bands;band++){
   double nm=wavelength(band);
   for(uint64_t n=0;n<per_band;n++){
    RNG rng(mix64(s.settings.seed^mix64(n)^mix64(uint64_t(band)+0xabcd123)));
    int e=std::min(int(cdf.size())-1,int(std::lower_bound(cdf.begin(),cdf.end(),rng.uniform())-cdf.begin()));double selection=cdf[e]-(e?cdf[e-1]:0);
    Ray ray;double flux=0;ray.time=s.camera.shutter_open;
    if(kind[e]==0){
     const auto&p=s.primitives[index[e]];Vec3 normal;p.sample(rng,ray.time,ray.o,normal);ray.d=Frame(normal).world(cosine_hemisphere(rng));ray.o=offset(ray.o,normal,ray.d);
     flux=s.materials[p.material].radiance(nm)*p.area()*pi/selection;
    }else if(kind[e]==1){
     const auto&l=s.delta_lights[index[e]];
     if(l.kind==1){auto disk=concentric_disk(rng.uniform(),rng.uniform());Frame f(l.direction);ray.d=l.direction;ray.o=center-ray.d*scene_radius+(f.x*disk[0]+f.y*disk[1])*scene_radius;flux=l.intensity.eval(nm)*l.scale*pi*sqr(scene_radius)/selection;}
     else{ray.o=l.position;ray.d=uniform_sphere(rng);flux=l.intensity.eval(nm)*l.scale*4*pi*l.falloff(ray.d)/selection;}
    }else{
     double pd;Vec3 d=s.environment.sample(rng,pd);auto disk=concentric_disk(rng.uniform(),rng.uniform());Frame f(d);ray.d=-d;ray.o=center+d*scene_radius+(f.x*disk[0]+f.y*disk[1])*scene_radius;
     flux=s.environment.radiance(d,nm)*pi*sqr(scene_radius)/(pd*selection);
    }
    flux/=per_band;MediumStack stack;int nulls=0;
    for(int depth=0;(s.settings.max_depth<0||depth<s.settings.max_depth)&&flux>0;depth++){
     Hit h;if(!s.bvh.hit(ray,h))break;const auto&m=s.materials[h.material];if(m.type==MaterialType::Emitter)break;
     if(!stack.empty())flux*=std::exp(-s.materials[stack.back().material].absorption_value(nm)*h.t);
     if(!m.delta()&&depth>0){Photon p{h.p,-ray.d,h.front?h.gn:-h.gn,flux,h.object};grids[band][cell(h.p)].push_back(p);stored++;}
     double ni,nt;dielectric_indices(s,m,h,stack,nm,ni,nt);auto b=sample_bsdf(m,h,-ray.d,nm,ni,nt,rng,false,false);
     if(b.pdf<=0||b.weight.v<=0)break;flux*=b.weight.v;
     // Light subpaths use importance transport, cancelling the camera-mode eta^2 factor.
     if(b.medium_transition){flux*=sqr(nt/ni);transition_medium(stack,h);}
     bool pass=b.delta&&b.transmission&&!b.medium_transition;ray={offset(h.p,h.gn,b.wi),b.wi,ray.time};
     if(pass){if(++nulls>1024)break;depth--;continue;}
     if(depth>=s.settings.rr_depth){if(rng.uniform()>.85)break;flux/=.85;}
    }
   }
  }
 }
 double gather(const Scene&s,const Hit&h,Vec3 wo,double nm,int band,double ni,double nt)const{
  PhotonCell c=cell(h.p);double sum=0;Vec3 outward=h.front?h.gn:-h.gn;const auto&m=s.materials[h.material];
  for(int x=-1;x<=1;x++)for(int y=-1;y<=1;y++)for(int z=-1;z<=1;z++){
   auto it=grids[band].find({c.x+x,c.y+y,c.z+z});if(it==grids[band].end())continue;
   for(auto&p:it->second){double d2=norm2(p.p-h.p);if(d2>=radius*radius||p.object!=h.object||dot(outward,p.normal)<.8)continue;
    auto e=evaluate_bsdf(m,h,wo,p.wi,nm,ni,nt,false);double kernel=2*(1-d2/sqr(radius))/(pi*sqr(radius));sum+=e.f.v*p.flux*kernel;
   }
  }
  return sum;
 }
 PathResult trace(const Scene&s,Ray ray,int band,RNG&rng)const{
  PathResult result;double beta=1,L=0,nm=wavelength(band);MediumStack stack;int nulls=0;
  for(int depth=0;(s.settings.max_depth<0||depth<s.settings.max_depth);depth++){
   Hit h;if(!s.bvh.hit(ray,h)){L+=beta*s.environment.radiance(ray.d,nm);break;}
   if(!stack.empty())beta*=std::exp(-s.materials[stack.back().material].absorption_value(nm)*h.t);
   const auto&m=s.materials[h.material];if(h.front)L+=beta*m.radiance(nm);if(m.type==MaterialType::Emitter)break;
   double ni,nt;dielectric_indices(s,m,h,stack,nm,ni,nt);
   if(!m.delta()){
    auto l=s.sample_light(h.p,ray.time,nm,rng);
    if(l.pdf>0){auto e=evaluate_bsdf(m,h,-ray.d,l.wi,nm,ni,nt,false);auto sh=s.shadow({offset(h.p,h.gn,l.wi),l.wi,ray.time},l.distance,nm,rng);L+=beta*e.f.v*std::abs(dot(h.n,l.wi))*l.radiance*sh.value/l.pdf;}
    L+=beta*gather(s,h,-ray.d,nm,band,ni,nt);
   }
   auto b=sample_bsdf(m,h,-ray.d,nm,ni,nt,rng,false,false);if(b.pdf<=0||b.weight.v<=0||!b.delta)break;beta*=b.weight.v;
   if(b.medium_transition)transition_medium(stack,h);ray={offset(h.p,h.gn,b.wi),b.wi,ray.time};
   if(b.delta&&b.transmission&&!b.medium_transition){if(++nulls>1024)break;depth--;}
   if(depth>=s.settings.rr_depth){if(rng.uniform()>.85)break;beta/=.85;}
   result.bounces=depth;
  }
  result.radiance=L;result.stokes[0]=L;return result;
 }
};
}

