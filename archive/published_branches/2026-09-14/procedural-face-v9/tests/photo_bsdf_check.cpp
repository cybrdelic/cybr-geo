#define main legacy_renderer_main
#include "../native/pathtrace.cpp"
#undef main
#include "../native/photo_bsdf.h"
#include <cassert>

int main(){
 Frame frame(V(0,0,1));RNG rng(9137);float maxDifference=0,maximumEnergy=0;
 Material m{V(.62f,.65f,.68f),1.f,.29f,0};
 // A reciprocal BSDF must agree when incident and outgoing directions swap.
 for(int i=0;i<10000;i++){
  V v=unit(V(rng.uniform()-.5f,rng.uniform()-.5f,.1f+rng.uniform()));
  V l=unit(V(rng.uniform()-.5f,rng.uniform()-.5f,.1f+rng.uniform()));
  V a=photoEval(m,frame,v,l),b=photoEval(m,frame,l,v);
  maxDifference=std::max(maxDifference,len(a-b)/(1+len(a)));
 }
 assert(maxDifference<2e-4f);
 assert(std::fabs(photoDielectricFresnel(1.f,1.5f)-.04f)<1e-6f);
 assert(std::fabs(photoDielectricFresnel(0.f,1.5f)-1.f)<1e-6f);
 assert(std::fabs(photoDielectricFresnel(.5f,1.5f)-.0891867f)<1e-5f);
 // Monte Carlo white-furnace tests integrate transport, including rejected
 // below-surface VNDF samples. No material may create energy.
 for(float metal:{0.f,1.f})for(float rough:{.12f,.3f,.7f})for(float aniso:{-.7f,0.f,.7f})for(float coat:{0.f,.5f}){
  m={V(.85f),metal,rough,0};m.aniso=aniso;m.coat=coat;m.coatRough=.2f;
  for(float z:{.2f,1.f}){
   V v=unit(V(std::sqrt(1-z*z),0,z));double energy=0;int samples=80000;
   for(int i=0;i<samples;i++){
    V l=photoSample(m,frame,v,rng);if(l.z<=0)continue;
    float pdf=photoPDF(m,frame,v,l);assert(pdf>0&&std::isfinite(pdf));
    V value=photoEval(m,frame,v,l)*(l.z/pdf);
    assert(std::isfinite(maxc(value))&&value.x>=0&&value.y>=0&&value.z>=0);
    energy+=value.x;
   }
   maximumEnergy=std::max(maximumEnergy,float(energy/samples));
   if(energy/samples>=1.02)std::cerr<<"Energy failure "<<metal<<" "<<rough<<" "<<aniso<<" "<<coat<<" "<<z<<" "<<energy/samples<<"\n";
   assert(energy/samples<1.02);
  }
 }
 // Rotating the authored grain rotates the elongated reflection lobe.
 m={V(.6f),1.f,.35f,3};m.aniso=.7f;m.grainAxis=V(1,0,0);
 Frame a=photoFrame(m,V(0,0,1),V(2,3,0));m.rotation=PI/2;
 Frame b=photoFrame(m,V(0,0,1),V(2,3,0));V v(0,0,1),l=unit(V(.4f,0,1));
 assert(std::fabs(photoEval(m,a,v,l).x-photoEval(m,b,v,l).x)>.02f);
 std::cout<<"{\"reciprocity_relative_error\":"<<maxDifference<<",\"maximum_white_furnace_energy\":"<<maximumEnergy<<",\"transport_cases\":72,\"dielectric_reference_values_passed\":true}\n";
 return 0;
}
