#include "cybr/integrator.hpp"
#include <iostream>
#include <functional>
#include <iomanip>
using namespace cybr;
int main(){int passed=0,failed=0;auto test=[&](const std::string&name,bool ok){std::cout<<(ok?"PASS ":"FAIL ")<<name<<"\n";if(ok)passed++;else failed++;};
 test("normal-incidence Fresnel",std::abs(fresnel_dielectric(1,1,1.5)-.04)<1e-12);
 test("total internal reflection",fresnel_dielectric(.5,1.5,1)==1);
 Vec3 t;bool refr=refract(normalize(Vec3(.5,-std::sqrt(.75),0)),Vec3(0,1,0),1/1.5,t);test("Snell refraction",refr&&std::abs(t.x-1./3)<1e-12);
 test("dispersion short wavelengths have higher IOR",cauchy_ior(450,1.5,.004)>cauchy_ior(650,1.5,.004));
 for(double a:{0.,pi/8,pi/4,pi/2}){Stokes r=polarizer(a)*(polarizer(0)*Stokes{1,0,0,0});test("Malus law "+std::to_string(a),std::abs(r[0]-.5*sqr(std::cos(a)))<1e-12);}
 Stokes circular=retarder(pi/4,pi/2)*(polarizer(0)*Stokes{1,0,0,0});test("quarter-wave plate creates circular polarization",std::abs(std::abs(circular[3])-.5)<1e-12&&std::abs(circular[1])<1e-12);
 double brewster=std::atan(1.5);Mueller m=dielectric_mueller(std::cos(brewster),1,1.5,false);test("Brewster reflected light is fully linear",std::abs(std::abs(m.at(1,0))-m.at(0,0))<1e-12);
 Dual a=Dual::variable(.4,0),b=Dual::variable(.7,1);Dual z=exp(a*b)/(a+Dual(2));double eps=1e-6;auto f=[](double x,double y){return std::exp(x*y)/(x+2);};test("forward AD product quotient exponential",std::abs(z.d[0]-(f(.4+eps,.7)-f(.4-eps,.7))/(2*eps))<1e-9&&std::abs(z.d[1]-(f(.4,.7+eps)-f(.4,.7-eps))/(2*eps))<1e-9);
 RNG rng(891);Vec3 sum;double hgmean=0;int N=100000;for(int i=0;i<N;i++){sum+=cosine_hemisphere(rng);hgmean+=sample_hg({0,0,1},.6,rng).z;}sum=sum/N;test("cosine hemisphere mean",std::abs(sum.z-2./3)<.004&&std::abs(sum.x)<.005);test("HG phase first moment",std::abs(hgmean/N-.6)<.004);
 double integral=0;for(int i=0;i<N;i++){double c=2*(i+.5)/N-1;integral+=henyey_greenstein(c,.6)*4*pi/N;}test("HG phase normalization",std::abs(integral-1)<1e-7);
 double Dnorm=0;for(int i=0;i<N;i++){double c=(i+.5)/N;Dnorm+=ggx_D(c,.3)*c*2*pi/N;}test("GGX projected area normalization",std::abs(Dnorm-1)<1e-7);
 Bounds box;box.grow({-1,-1,-1});box.grow({1,1,1});double near,far;test("parallel slab inside",box.interval({{0,0,-3},{0,0,1}},inf,near,far)&&std::abs(near-2)<1e-12);test("parallel slab outside",!box.hit({{2,0,-3},{0,0,1}},inf));
 std::vector<Primitive> ps;for(int i=0;i<64;i++){Primitive p;p.edit().a={rng.uniform()*8-4,rng.uniform()*8-4,rng.uniform()*8-4};p.radius=.1+rng.uniform()*.4;p.edit().velocity={.1,0,0};ps.push_back(p);}BVH bvh;bvh.build(ps);bool agrees=true;for(int i=0;i<10000;i++){Ray r{{rng.uniform()*12-6,rng.uniform()*12-6,rng.uniform()*12-6},uniform_sphere(rng),rng.uniform()};Hit fast,slow;bool h=bvh.hit(r,fast),brute=false;for(auto&p:ps)brute=p.hit(r,1e-6,slow)||brute;if(h!=brute||(h&&std::abs(fast.t-slow.t)>1e-10))agrees=false;}test("BVH agrees with brute force on 10000 moving-geometry rays",agrees);
 Volume v;v.bounds=box;v.extinction=Spectrum(.7);Ray vr{{0,0,-2},{0,0,1}};double exact=std::exp(-1.4);test("homogeneous Beer-Lambert",std::abs(v.transmittance(vr,4,550,rng)-exact)<1e-14);int survived=0;for(int i=0;i<N;i++){double d;if(!v.sample(vr,4,550,rng,d))survived++;}test("homogeneous free-flight survival",std::abs(double(survived)/N-exact)<.005);
 v.kind=2;v.nx=v.ny=v.nz=2;v.grid.assign(8,.4f);v.density_majorant=1;double rat=0;for(int i=0;i<20000;i++)rat+=v.transmittance(vr,4,550,rng);test("heterogeneous ratio tracking constant-grid limit",std::abs(rat/20000-std::exp(-.56))<.008);
 Environment env;env.strength=1;env.build();double ep=0;for(int i=0;i<N;i++)ep+=env.pdf(uniform_sphere(rng))*4*pi/N;test("environment importance PDF normalization",std::abs(ep-1)<.008);
 Material metal;metal.type=MaterialType::Metal;metal.roughness=.5;Hit h;h.n=h.gn={0,0,1};Vec3 wo=normalize(Vec3(.3,0,1));bool finite=true;for(int i=0;i<10000;i++){auto q=sample_bsdf(metal,h,wo,550,1,1,rng,false,false);if(q.pdf>0){auto e=evaluate_bsdf(metal,h,wo,q.wi,550,1,1,false);if(std::abs(e.pdf-q.pdf)>1e-12||!std::isfinite(q.weight.v)||q.weight.v<0)finite=false;}}test("GGX sampler and PDF agreement",finite);
 Material mirror;mirror.type=MaterialType::Mirror;mirror.eta=Spectrum(.3);mirror.k=Spectrum(3.4);auto scalar_mirror=sample_bsdf(mirror,h,wo,550,1,1,rng,false,false);auto polarized_mirror=sample_bsdf(mirror,h,wo,550,1,1,rng,false,true);test("scalar and polarized conductor mirror agree for unpolarized input",std::abs(scalar_mirror.weight.v-polarized_mirror.weight.v)<1e-12);
 test("conductor Fresnel normal-incidence analytic limit",std::abs(fresnel_conductor(1,.3,3.4)-(sqr(.3-1)+sqr(3.4))/(sqr(.3+1)+sqr(3.4)))<1e-12);
 Primitive moving;moving.edit().velocity={5,0,0};moving.radius=.5;std::vector<Primitive> ms{moving};BVH moving_bvh;moving_bvh.build(ms,-2,3);bool moving_ok=true;for(double tm:{-2.,-.5,1.5,3.}){Ray mr{{5*tm,0,-3},{0,0,1},tm};Hit mh;moving_ok=moving_ok&&moving_bvh.hit(mr,mh)&&std::abs(mh.t-2.5)<1e-12;}test("BVH handles negative and greater-than-one shutter times",moving_ok);
 std::cout<<"TOTAL "<<passed<<" passed, "<<failed<<" failed\n";return failed?1:0;
}

