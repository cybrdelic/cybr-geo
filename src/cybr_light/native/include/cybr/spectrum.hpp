#pragma once
#include "math.hpp"
#include <vector>
#include <fstream>
#include <sstream>
namespace cybr {
struct Spectrum {
 // Basis coefficients are spectral controls, not colorimetric sRGB values.
 Vec3 controls{.5};
 bool constant=false;
 std::vector<std::pair<double,double>> table;
 Spectrum()=default;explicit Spectrum(double a):controls(a),constant(true){}
 explicit Spectrum(Vec3 c):controls(c){}
 Vec3 weights(double nm)const {
  if(constant)return {1,0,0};
  // All nonconstant spectra share the same basis. Cache the exact result
  // within a thread for the duration of one wavelength path; no quantization.
  static thread_local double cached_nm=-1;
  static thread_local Vec3 cached;
  if(nm==cached_nm)return cached;
  double r=std::exp(-.5*sqr((nm-610)/43.)),g=std::exp(-.5*sqr((nm-545)/34.)),b=std::exp(-.5*sqr((nm-450)/27.));
  double sum=r+g+b+1e-30;cached_nm=nm;cached={r/sum,g/sum,b/sum};return cached;
 }
 double eval(double nm)const{
  if(!table.empty()){
   auto it=std::lower_bound(table.begin(),table.end(),nm,[](auto a,double x){return a.first<x;});
   if(it==table.begin())return it->second;if(it==table.end())return table.back().second;
   auto a=*(it-1);auto b=*it;return a.second+(b.second-a.second)*(nm-a.first)/(b.first-a.first);
  }
  if(constant)return controls.x;return dot(controls,weights(nm));
 }
 Dual eval_ad(double nm,bool active)const{
  Dual r(eval(nm));if(active&&table.empty()){Vec3 w=weights(nm);r.d={w.x,w.y,w.z};}return r;
 }
};
struct Observer {
 std::vector<std::array<double,4>> table;
 // Analytic approximation fallback, explicitly reported when a CIE table is absent.
 static Vec3 analytic(double w){
  auto g=[&](double m,double a,double b){double t=(w-m)*(w<m?a:b);return std::exp(-.5*t*t);};
  return {1.056*g(599.8,.0264,.0323)+.362*g(442,.0624,.0374)-.065*g(501.1,.049,.0382),
  .821*g(568.8,.0213,.0247)+.286*g(530.9,.0613,.0322),1.217*g(437,.0845,.0278)+.681*g(459,.0385,.0725)};
 }
 void load(const std::string& file){std::ifstream f(file);if(!f)throw std::runtime_error("Cannot load observer: "+file);std::string line;while(std::getline(f,line)){for(char& c:line)if(c==',')c=' ';std::istringstream ss(line);std::array<double,4> a;if(ss>>a[0]>>a[1]>>a[2]>>a[3])table.push_back(a);}if(table.size()<10)throw std::runtime_error("Invalid observer table");}
 Vec3 xyz(double nm)const{if(table.empty())return analytic(nm);auto it=std::lower_bound(table.begin(),table.end(),nm,[](auto a,double x){return a[0]<x;});if(it==table.begin())return {(*it)[1],(*it)[2],(*it)[3]};if(it==table.end())return {};auto a=*(it-1),b=*it;double t=(nm-a[0])/(b[0]-a[0]);return {a[1]*(1-t)+b[1]*t,a[2]*(1-t)+b[2]*t,a[3]*(1-t)+b[3]*t};}
};
inline Vec3 xyz_to_rgb(Vec3 a){return {3.2404542*a.x-1.5371385*a.y-.4985314*a.z,-.9692660*a.x+1.8760108*a.y+.0415560*a.z,.0556434*a.x-.2040259*a.y+1.0572252*a.z};}
inline double blackbody(double nm,double kelvin){double l=nm*1e-9;return 1/(std::pow(l,5)*std::expm1(.01438776877/(l*kelvin)));}
inline double blackbody_normalized(double nm,double kelvin){
 static thread_local double last_nm=-1;
 static thread_local std::array<double,8> temperatures{},values{};
 static thread_local int count=0;
 if(nm!=last_nm){last_nm=nm;count=0;}
 for(int i=0;i<count;i++)if(temperatures[i]==kelvin)return values[i];
 double value=blackbody(nm,kelvin)/blackbody(560,kelvin);
 if(count<8){temperatures[count]=kelvin;values[count]=value;count++;}
 return value;
}
inline double cauchy_ior(double nm,double a,double b){double um=nm*.001;return a+b/(um*um);}
// Real-index Fresnel, with signed amplitude conventions in polarization.hpp.
inline double fresnel_dielectric(double c,double ni,double nt){
 c=clamp(std::abs(c));double st2=sqr(ni/nt)*(1-c*c);if(st2>=1)return 1;double ct=std::sqrt(1-st2);
 double rs=(ni*c-nt*ct)/(ni*c+nt*ct),rp=(nt*c-ni*ct)/(nt*c+ni*ct);return .5*(rs*rs+rp*rp);
}
inline double fresnel_conductor(double c,double eta,double k){
 c=clamp(std::abs(c));double c2=c*c,s2=1-c2,e2=eta*eta,k2=k*k,t0=e2-k2-s2;
 double a2b2=std::sqrt(t0*t0+4*e2*k2),a=std::sqrt(std::max(0.,.5*(a2b2+t0)));
 double t1=a2b2+c2,t2=2*c*a,rs=(t1-t2)/(t1+t2);
 double t3=c2*a2b2+s2*s2,t4=t2*s2,rp=rs*(t3-t4)/(t3+t4+1e-30);return clamp(.5*(rs+rp));
}
}

