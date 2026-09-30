#pragma once
#include "spectrum.hpp"
namespace cybr {
inline double smoothstep(double a,double b,double x){double t=clamp((x-a)/(b-a));return t*t*(3-2*t);}
inline double lattice_noise(int x,int y,int z){uint64_t h=mix64(uint64_t(int64_t(x)*73856093)^uint64_t(int64_t(y)*19349663)^uint64_t(int64_t(z)*83492791));return (h>>11)*(1./9007199254740992.);}
inline double value_noise(Vec3 p){
 int x=int(std::floor(p.x)),y=int(std::floor(p.y)),z=int(std::floor(p.z));double u=p.x-x,v=p.y-y,w=p.z-z;u=u*u*(3-2*u);v=v*v*(3-2*v);w=w*w*(3-2*w);double out=0;
 for(int a=0;a<2;a++)for(int b=0;b<2;b++)for(int c=0;c<2;c++)out+=lattice_noise(x+a,y+b,z+c)*(a?u:1-u)*(b?v:1-v)*(c?w:1-w);return out;
}
inline double fbm(Vec3 p){return .55*value_noise(p)+.28*value_noise(p*2.07+Vec3(7.1))+.12*value_noise(p*4.17+Vec3(3.7))+.05*value_noise(p*8.13);}
struct CloudLobe {Vec3 center,radii;double strength=1.;};
struct Volume {
 Bounds bounds;Spectrum extinction{1.},albedo{.95};double g=.2,scale=1.,density_majorant=1.;int kind=0,phase=0;
 std::vector<CloudLobe> lobes;std::vector<float> grid;int nx=0,ny=0,nz=0;
 // Validate once at scene construction, before entering parallel transport.
 static size_t grid_count(int x,int y,int z){
  constexpr size_t limit=128000000;
  size_t count=1;
  for(int n:{x,y,z}){
   if(n<2||size_t(n)>limit/count)throw std::runtime_error("Invalid or excessive volume grid dimensions");
   count*=size_t(n);
  }
  return count;
 }
 void validate()const{
  if(kind<0||kind>2||phase<0||phase>1)throw std::runtime_error("Unsupported volume or phase type");
  for(int i=0;i<3;i++){
   if(!std::isfinite(bounds.lo[i])||!std::isfinite(bounds.hi[i])||bounds.hi[i]<=bounds.lo[i])throw std::runtime_error("Invalid volume bounds");
   if(!std::isfinite(extinction.controls[i])||extinction.controls[i]<0||!std::isfinite(albedo.controls[i])||albedo.controls[i]<0||albedo.controls[i]>1)throw std::runtime_error("Invalid volume optical coefficients");
  }
  if(!std::isfinite(g)||std::abs(g)>=1||!std::isfinite(scale)||scale<=0||!std::isfinite(density_majorant)||density_majorant<0)throw std::runtime_error("Invalid volume sampling parameters");
  for(const auto&l:lobes){
   if(!std::isfinite(l.strength)||l.strength<0)throw std::runtime_error("Invalid cloud lobe strength");
   for(int i=0;i<3;i++)if(!std::isfinite(l.center[i])||!std::isfinite(l.radii[i])||l.radii[i]<=0)throw std::runtime_error("Invalid cloud lobe geometry");
  }
  if(kind==2){
   if(grid.size()!=grid_count(nx,ny,nz))throw std::runtime_error("Missing or inconsistent volume grid data");
   for(float d:grid)if(!std::isfinite(d)||d<0||d>density_majorant*(1+1e-7))throw std::runtime_error("Invalid density or volume majorant");
  }else if(kind==1&&density_majorant<1)throw std::runtime_error("Procedural cloud majorant must cover its unit density bound");
 }
 double density(Vec3 p)const{
  if(p.x<bounds.lo.x||p.y<bounds.lo.y||p.z<bounds.lo.z||p.x>bounds.hi.x||p.y>bounds.hi.y||p.z>bounds.hi.z)return 0;
  if(kind==0)return 1.;
  if(kind==1){double q=0;for(const auto&l:lobes){Vec3 d=p-l.center;double e=sqr(d.x/l.radii.x)+sqr(d.y/l.radii.y)+sqr(d.z/l.radii.z);q+=l.strength*std::max(0.,1-e);}
   double n=fbm(p*scale);return clamp(q*(.4+.9*n)-.32*(1-n));}
  if(kind==2){Vec3 sz=bounds.hi-bounds.lo;Vec3 t{(p.x-bounds.lo.x)/sz.x*(nx-1),(p.y-bounds.lo.y)/sz.y*(ny-1),(p.z-bounds.lo.z)/sz.z*(nz-1)};int ix=std::min(nx-2,int(t.x)),iy=std::min(ny-2,int(t.y)),iz=std::min(nz-2,int(t.z));double v=0;for(int a=0;a<2;a++)for(int b=0;b<2;b++)for(int c=0;c<2;c++)v+=grid[((iz+c)*ny+iy+b)*nx+ix+a]*(a?t.x-ix:1-t.x+ix)*(b?t.y-iy:1-t.y+iy)*(c?t.z-iz:1-t.z+iz);return std::max(0.,v);}
  throw std::runtime_error("Unknown volume kind");
 }
 // Analog delta tracking: null events do not change the path throughput.
 bool sample(const Ray&r,double max_t,double nm,RNG&rng,double& t)const{
  double near,far;if(!bounds.interval(r,max_t,near,far)||near>=far)return false;double sigma=extinction.eval(nm);if(sigma<=0)return false;
  if(kind==0){t=near-std::log(rng.uniform())/sigma;return t<far;}
  double majorant=sigma*density_majorant;if(majorant<=0)return false;t=near;
  while(true){t-=std::log(rng.uniform())/majorant;if(t>=far)return false;double d=density(r.at(t));if(d>density_majorant*(1+1e-7))throw std::runtime_error("Invalid density majorant");if(rng.uniform()<d/density_majorant)return true;}
 }
 // Ratio tracking estimates transmittance without a fixed ray-march step.
 double transmittance(const Ray&r,double max_t,double nm,RNG&rng)const{
  double near,far;if(!bounds.interval(r,max_t,near,far)||near>=far)return 1;double sigma=extinction.eval(nm);if(sigma<=0)return 1;if(kind==0)return std::exp(-sigma*(far-near));
  double majorant=sigma*density_majorant,t=near,T=1;if(majorant<=0)return 1;
  while(true){t-=std::log(rng.uniform())/majorant;if(t>=far)return T;double d=density(r.at(t));if(d>density_majorant*(1+1e-7))throw std::runtime_error("Invalid density majorant");T*=std::max(0.,1-d/density_majorant);if(T==0)return 0;}
 }
};
inline double henyey_greenstein(double c,double g){double a=1+g*g-2*g*c;return (1-g*g)/(4*pi*a*std::sqrt(a));}
inline Vec3 sample_hg(Vec3 incoming,double g,RNG&r){double u=r.uniform(),c;if(std::abs(g)<1e-4)c=1-2*u;else{double t=(1-g*g)/(1-g+2*g*u);c=clamp((1+g*g-t*t)/(2*g),-1,1);}double a=2*pi*r.uniform(),s=std::sqrt(1-c*c);return Frame(incoming).world({s*std::cos(a),s*std::sin(a),c});}
inline double rayleigh_phase(double c){return 3*(1+c*c)/(16*pi);}
inline Vec3 sample_rayleigh(Vec3 incoming,RNG&r){double c;do{c=2*r.uniform()-1;}while(r.uniform()>(1+c*c)*.5);double a=2*pi*r.uniform(),t=std::sqrt(std::max(0.,1-c*c));return Frame(incoming).world({t*std::cos(a),t*std::sin(a),c});}
inline double phase_eval(const Volume&v,double c){return v.phase==1?rayleigh_phase(c):henyey_greenstein(c,v.g);}
inline Vec3 phase_sample(const Volume&v,Vec3 incoming,RNG&r){return v.phase==1?sample_rayleigh(incoming,r):sample_hg(incoming,v.g,r);}

}

