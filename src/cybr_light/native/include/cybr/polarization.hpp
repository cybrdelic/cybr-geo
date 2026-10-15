#pragma once
#include "spectrum.hpp"
#include <complex>
namespace cybr {
using Stokes=std::array<double,4>;
struct Mueller {
 std::array<double,16> m{};
 double& at(int r,int c){return m[r*4+c];} double at(int r,int c)const{return m[r*4+c];}
 static Mueller identity(){Mueller a;for(int i=0;i<4;i++)a.at(i,i)=1;return a;}
 static Mueller depolarizer(double a){Mueller r;r.at(0,0)=a;return r;}
};
inline Mueller operator+(Mueller a,const Mueller&b){for(int i=0;i<16;i++)a.m[i]+=b.m[i];return a;}
inline Mueller operator*(const Mueller&a,const Mueller&b){Mueller r;for(int i=0;i<4;i++)for(int j=0;j<4;j++)for(int k=0;k<4;k++)r.at(i,j)+=a.at(i,k)*b.at(k,j);return r;}
inline Mueller operator*(Mueller a,double b){for(double&x:a.m)x*=b;return a;}
inline Stokes operator*(const Mueller&a,const Stokes&b){Stokes r{};for(int i=0;i<4;i++)for(int j=0;j<4;j++)r[i]+=a.at(i,j)*b[j];return r;}
inline Mueller rotation(double theta){Mueller r=Mueller::identity();double c=std::cos(2*theta),s=std::sin(2*theta);r.at(1,1)=c;r.at(1,2)=s;r.at(2,1)=-s;r.at(2,2)=c;return r;}
inline Mueller diagonal_jones(std::complex<double> s,std::complex<double> p){
 Mueller r;double a=std::norm(s),b=std::norm(p);auto z=s*std::conj(p);r.at(0,0)=r.at(1,1)=.5*(a+b);r.at(0,1)=r.at(1,0)=.5*(a-b);r.at(2,2)=r.at(3,3)=z.real();r.at(2,3)=-z.imag();r.at(3,2)=z.imag();return r;
}
inline Mueller polarizer(double angle){return rotation(-angle)*diagonal_jones(1.,0.)*rotation(angle);}
inline Mueller retarder(double angle,double retardance){return rotation(-angle)*diagonal_jones(std::polar(1.,retardance*.5),std::polar(1.,-retardance*.5))*rotation(angle);}
inline double axis_angle(Vec3 direction,Vec3 axis){Frame f(direction);Vec3 projected=axis-direction*dot(direction,axis);if(norm2(projected)<1e-20)return 0;return std::atan2(dot(projected,f.y),dot(projected,f.x));}
inline Mueller frame_scattering(Mueller local,Vec3 wo,Vec3 wi,Vec3 normal){
 Vec3 kin=-wi,kout=wo;Vec3 si=cross(kin,normal),so=cross(kout,normal);
 if(norm2(si)<1e-18)si=Frame(kin).x;if(norm2(so)<1e-18)so=Frame(kout).x;
 return rotation(-axis_angle(kout,so))*local*rotation(axis_angle(kin,si));
}
inline Mueller dielectric_mueller(double c,double ni,double nt,bool transmission){
 c=clamp(std::abs(c));using C=std::complex<double>;C ct=std::sqrt(C(1-sqr(ni/nt)*(1-c*c),0));
 C rs=(ni*c-nt*ct)/(ni*c+nt*ct),rp=(nt*c-ni*ct)/(nt*c+ni*ct);
 if(!transmission)return diagonal_jones(rs,rp);
 double s=std::sqrt(std::max(0.,1-std::norm(rs))),p=std::sqrt(std::max(0.,1-std::norm(rp)));return diagonal_jones(s,p);
}
inline Mueller conductor_mueller(double c,double eta,double k){using C=std::complex<double>;c=clamp(std::abs(c));C n(eta,k);C ct=std::sqrt(C(1)-(1-c*c)/(n*n));C rs=(c-n*ct)/(c+n*ct),rp=(n*c-ct)/(n*c+ct);return diagonal_jones(rs,rp);}
}

