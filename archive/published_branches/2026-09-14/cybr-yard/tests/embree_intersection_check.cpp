#define main legacy_renderer_main
#include "../native/pathtrace.cpp"
#undef main
#include <cassert>
int main(){
 RNG rng(27041);
 for(int i=0;i<180;i++){
  Tri t;t.p=V(10*rng.uniform()-5,10*rng.uniform()-5,10*rng.uniform()-5);
  t.e1=V(rng.uniform()+.2f,rng.uniform(),rng.uniform());t.e2=V(rng.uniform(),rng.uniform()+.2f,-rng.uniform());
  t.n0=t.n1=t.n2=unit(cross(t.e1,t.e2));tris.push_back(t);
 }
 order.resize(tris.size());std::iota(order.begin(),order.end(),0);build(0,order.size());buildSceneAcceleration();
 int hits=0,shadows=0;float error=0;
 for(int i=0;i<25000;i++){
  V o(20*rng.uniform()-10,20*rng.uniform()-10,20*rng.uniform()-10);
  V target(10*rng.uniform()-5,10*rng.uniform()-5,10*rng.uniform()-5);Ray ray(o,unit(target-o));
  Hit a,b;bool ar=meshHit(ray,a),br=acceleratedHit(ray,b);assert(ar==br);
  if(ar){hits++;error=std::max(error,std::fabs(a.t-b.t));assert(std::fabs(a.t-b.t)<2e-4f);}
  a=Hit();b=Hit();a.t=b.t=15*rng.uniform();assert(meshHit(ray,a,true)==acceleratedHit(ray,b,true));shadows++;
 }
 std::cout<<"{\"rays\":25000,\"hits\":"<<hits<<",\"shadow_tests\":"<<shadows<<",\"maximum_distance_error_mm\":"<<error<<",\"passed\":true}\n";
 return 0;
}
