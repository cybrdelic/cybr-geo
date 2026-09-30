#pragma once
#include <cmath>
#include <algorithm>
#include <array>
#include <cstdint>
#include <limits>
#include <stdexcept>
namespace cybr {
constexpr double pi=3.1415926535897932384626433832795;
constexpr double inf=std::numeric_limits<double>::infinity();
inline double clamp(double x,double a=0.,double b=1.) { return std::min(b,std::max(a,x)); }
inline double sqr(double x){ return x*x; }
struct Vec3 {
 double x=0,y=0,z=0;
 Vec3()=default;
 explicit Vec3(double v):x(v),y(v),z(v){}
 Vec3(double X,double Y,double Z):x(X),y(Y),z(Z){}
 double operator[](int i)const{return i==0?x:i==1?y:z;}
 double& operator[](int i){return i==0?x:i==1?y:z;}
 Vec3 operator-()const{return {-x,-y,-z};}
 Vec3& operator+=(Vec3 b){x+=b.x;y+=b.y;z+=b.z;return *this;}
 Vec3& operator*=(double s){x*=s;y*=s;z*=s;return *this;}
};
inline Vec3 operator+(Vec3 a,Vec3 b){return {a.x+b.x,a.y+b.y,a.z+b.z};}
inline Vec3 operator-(Vec3 a,Vec3 b){return {a.x-b.x,a.y-b.y,a.z-b.z};}
inline Vec3 operator*(Vec3 a,double b){return {a.x*b,a.y*b,a.z*b};}
inline Vec3 operator*(double a,Vec3 b){return b*a;}
inline Vec3 operator*(Vec3 a,Vec3 b){return {a.x*b.x,a.y*b.y,a.z*b.z};}
inline Vec3 operator/(Vec3 a,double b){return a*(1/b);}
inline Vec3 vmin(Vec3 a,Vec3 b){return {std::min(a.x,b.x),std::min(a.y,b.y),std::min(a.z,b.z)};}
inline Vec3 vmax(Vec3 a,Vec3 b){return {std::max(a.x,b.x),std::max(a.y,b.y),std::max(a.z,b.z)};}
inline double dot(Vec3 a,Vec3 b){return a.x*b.x+a.y*b.y+a.z*b.z;}
inline Vec3 cross(Vec3 a,Vec3 b){return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};}
inline double norm2(Vec3 a){return dot(a,a);}
inline double norm(Vec3 a){return std::sqrt(norm2(a));}
inline Vec3 normalize(Vec3 a){double n=norm(a);return n>0?a/n:Vec3(0,1,0);}
inline Vec3 reflect(Vec3 v,Vec3 n){return v-2*dot(v,n)*n;}
inline bool refract(Vec3 incident,Vec3 n,double eta,Vec3& out){
 double c=-dot(incident,n), k=1-eta*eta*(1-c*c);
 if(k<0)return false;
 out=normalize(eta*incident+(eta*c-std::sqrt(k))*n);return true;
}
struct Frame {
 Vec3 x,y,z;
 explicit Frame(Vec3 n):z(n){x=normalize(cross(std::abs(n.z)<.999?Vec3(0,0,1):Vec3(0,1,0),n));y=cross(z,x);}
 Frame(Vec3 n,Vec3 tangent,double handedness=1):z(n){x=tangent-z*dot(tangent,z);if(norm2(x)<1e-18)x=Frame(n).x;else x=normalize(x);y=cross(z,x)*handedness;}
 Vec3 world(Vec3 v)const{return x*v.x+y*v.y+z*v.z;}
 Vec3 local(Vec3 v)const{return {dot(v,x),dot(v,y),dot(v,z)};}
};
inline uint64_t mix64(uint64_t x){x+=0x9e3779b97f4a7c15ULL;x=(x^(x>>30))*0xbf58476d1ce4e5b9ULL;x=(x^(x>>27))*0x94d049bb133111ebULL;return x^(x>>31);}
struct RNG {
 uint64_t state,key=0,sample_index=0;int sampler=0,dimension=0;
 explicit RNG(uint64_t seed):state(mix64(seed)){}
 uint32_t next(){state=state*6364136223846793005ULL+1442695040888963407ULL;uint32_t x=((state>>18)^state)>>27;uint32_t r=state>>59;return (x>>r)|(x<<((-r)&31));}
 void configure(uint64_t sample,uint64_t pixel_key,int kind){sample_index=sample;key=pixel_key;sampler=kind;dimension=0;}
 double uniform(){
  if(sampler==1&&dimension<64){
   static const auto primes=[](){std::array<int,64>a{};int n=0;for(int p=2;n<64;p++){bool ok=true;for(int d=2;d*d<=p;d++)if(p%d==0){ok=false;break;}if(ok)a[n++]=p;}return a;}();
   int dim=dimension++,base=primes[dim];uint64_t v=sample_index+1;double inv=1./base,f=inv,u=0;while(v){u+=(v%base)*f;v/=base;f*=inv;}
   double shift=(mix64(key^mix64(dim))>>11)*(1./9007199254740992.);u+=shift;return std::min(1.-1e-15,std::max(1e-15,u-std::floor(u)));
  }
  dimension++;return (next()+.5)*(1./4294967296.);
 }
};
inline Vec3 cosine_hemisphere(RNG& r){double a=2*pi*r.uniform(),s=std::sqrt(r.uniform());return {s*std::cos(a),s*std::sin(a),std::sqrt(1-s*s)};}
inline Vec3 uniform_sphere(RNG& r){double z=1-2*r.uniform(),a=2*pi*r.uniform(),q=std::sqrt(std::max(0.,1-z*z));return {q*std::cos(a),q*std::sin(a),z};}
inline std::array<double,2> concentric_disk(double u,double v){double x=2*u-1,y=2*v-1;if(x==0&&y==0)return {0,0};double r,t;if(std::abs(x)>std::abs(y)){r=x;t=pi*.25*y/x;}else{r=y;t=pi*.5-pi*.25*x/y;}return {r*std::cos(t),r*std::sin(t)};}
inline double power_heuristic(double a,double b){return a*a/(a*a+b*b+1e-300);}
struct Ray { Vec3 o,d;double time=0.; Vec3 at(double t)const{return o+d*t;} };
struct Bounds {
 Vec3 lo{inf},hi{-inf};
 void grow(Vec3 p){lo=vmin(lo,p);hi=vmax(hi,p);}
 void grow(const Bounds& b){grow(b.lo);grow(b.hi);}
 Vec3 center()const{return (lo+hi)*.5;}
 double area()const{Vec3 e=vmax(hi-lo,Vec3(0));return 2*(e.x*e.y+e.y*e.z+e.z*e.x);}
 bool interval(const Ray& r,double limit,double& near,double& far)const{
  near=0;far=limit;
  for(int k=0;k<3;++k){if(std::abs(r.d[k])<1e-18){if(r.o[k]<lo[k]||r.o[k]>hi[k])return false;continue;}
   double a=(lo[k]-r.o[k])/r.d[k],b=(hi[k]-r.o[k])/r.d[k];if(a>b)std::swap(a,b);near=std::max(near,a);far=std::min(far,b);if(near>far)return false;}
  return far>=near;
 }
 bool hit(const Ray&r,double t)const{double a,b;return interval(r,t,a,b);}
};
inline Vec3 offset(Vec3 p,Vec3 n,Vec3 d){double e=2e-6*std::max(1.,std::max({std::abs(p.x),std::abs(p.y),std::abs(p.z)}));return p+n*(dot(n,d)>0?e:-e);}
// First-order forward-mode dual scalar. Geometry and probability distributions
// are deliberately held fixed in the material-parameter derivative integrator.
struct Dual {
 double v=0.;std::array<double,3> d{{0,0,0}};
 Dual()=default;Dual(double x):v(x){}
 static Dual variable(double x,int k){Dual r(x);if(k>=0&&k<3)r.d[k]=1;return r;}
};
inline Dual operator+(Dual a,Dual b){Dual r(a.v+b.v);for(int i=0;i<3;i++)r.d[i]=a.d[i]+b.d[i];return r;}
inline Dual operator-(Dual a,Dual b){Dual r(a.v-b.v);for(int i=0;i<3;i++)r.d[i]=a.d[i]-b.d[i];return r;}
inline Dual operator*(Dual a,Dual b){Dual r(a.v*b.v);for(int i=0;i<3;i++)r.d[i]=a.d[i]*b.v+a.v*b.d[i];return r;}
inline Dual operator/(Dual a,Dual b){Dual r(a.v/b.v);for(int i=0;i<3;i++)r.d[i]=(a.d[i]*b.v-a.v*b.d[i])/(b.v*b.v);return r;}
inline Dual exp(Dual a){Dual r(std::exp(a.v));for(int i=0;i<3;i++)r.d[i]=r.v*a.d[i];return r;}
}
