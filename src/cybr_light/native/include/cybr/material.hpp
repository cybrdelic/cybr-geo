#pragma once
#include "geometry.hpp"
#include "polarization.hpp"
#include "texture.hpp"
#include "shader.hpp"
namespace cybr {
enum class MaterialType {
 Diffuse,Metal,Plastic,Glass,RoughGlass,Mirror,Polarizer,Retarder,Emitter,
 Null,ThinGlass,DiffuseTransmission,Blend,Mask,TwoSided,NormalMap,BumpMap
};
struct Material {
 MaterialType type=MaterialType::Diffuse;
 Spectrum color{Vec3(.5)},eta{Vec3(.25,.6,1.2)},k{Vec3(3.5,2.8,2.)},absorption{0.};
 double roughness=.2,ior_a=1.5,ior_b=0,emission=0,kelvin=6500,checker=0,angle=0,retardance=pi*.5;
 double alpha_u=0,alpha_v=0,weight=.5,opacity=1,bump_scale=1;bool uv_checker=false;
 int source_index=-1;bool two_sided=true;Vec3 axis{1,0,0};bool differentiable=false;Spectrum second{Vec3(.1)};
 std::shared_ptr<ShaderLibrary> shader;std::array<double,3> shader_parameters{{0,0,0}};
 std::shared_ptr<ImageTexture> texture,temperature_texture;std::shared_ptr<Material> child1,child2;
 bool wrapper()const{return type==MaterialType::Mask||type==MaterialType::TwoSided||type==MaterialType::NormalMap||type==MaterialType::BumpMap;}
 double ior(double nm)const{return wrapper()&&child1?child1->ior(nm):cauchy_ior(nm,ior_a,ior_b);}
 Dual albedo(double nm,const Hit&h,bool ad)const{
  bool alternate=checker>0&&((int(std::floor((uv_checker?h.u:h.p.x)*checker))+int(std::floor((uv_checker?h.v:h.p.z)*checker)))&1);
  Dual value=shader?shader->evaluate(nm,h.u,h.v,shader_parameters,ad&&differentiable):(alternate?second.eval_ad(nm,false):color.eval_ad(nm,ad&&differentiable));
  if(texture)value=value*Dual(texture->eval(h.u,h.v,nm));return value*Dual(Spectrum(h.tint).eval(nm));
 }
 bool delta()const{
  if(wrapper())return child1&&child1->delta();
  if(type==MaterialType::Blend)return child1&&child2&&child1->delta()&&child2->delta();
  return type==MaterialType::Glass||type==MaterialType::ThinGlass||type==MaterialType::Mirror||type==MaterialType::Null||type==MaterialType::Polarizer||type==MaterialType::Retarder;
 }
 bool dielectric()const{return wrapper()&&child1?child1->dielectric():type==MaterialType::Glass||type==MaterialType::RoughGlass;}
 double absorption_value(double nm)const{return wrapper()&&child1?child1->absorption_value(nm):absorption.eval(nm);}
 double radiance(double nm,double u=0,double v=0)const{if(temperature_texture){double T=temperature_texture->lookup(u,v).x;return emission*1.191042972e-25*blackbody(nm,std::max(1.,T));}return emission*color.eval(nm)*(kelvin>0?blackbody_normalized(nm,kelvin):1);}
 double coverage(const Hit&h)const{return clamp(opacity*(texture?texture->lookup(h.u,h.v).x:1));}
};
// Trowbridge-Reitz distribution and Smith masking. alpha values are slopes.
inline double ggx_D(double cos_h,double alpha){if(cos_h<=0)return 0;double d=cos_h*cos_h*(alpha*alpha-1)+1;return alpha*alpha/(pi*d*d);}
inline double ggx_lambda(double c,double alpha){c=std::abs(c);if(c<1e-12)return inf;return .5*(std::sqrt(1+alpha*alpha*(1-c*c)/(c*c))-1);}
inline double ggx_G1(double c,double alpha){return 1/(1+ggx_lambda(c,alpha));}
inline double ggx_G(double a,double b,double alpha){return 1/(1+ggx_lambda(a,alpha)+ggx_lambda(b,alpha));}
inline double ggx_D_aniso(Vec3 h,double ax,double ay){if(h.z<=0)return 0;return 1/(pi*ax*ay*sqr(sqr(h.x/ax)+sqr(h.y/ay)+h.z*h.z));}
inline double ggx_lambda_aniso(Vec3 v,double ax,double ay){if(std::abs(v.z)<1e-15)return inf;return .5*(std::sqrt(1+(sqr(ax*v.x)+sqr(ay*v.y))/sqr(v.z))-1);}
inline Vec3 sample_visible_ggx_aniso(Vec3 view,double ax,double ay,RNG&r){
 Vec3 v=normalize(Vec3(ax*view.x,ay*view.y,view.z));double q=v.x*v.x+v.y*v.y;
 Vec3 t=q>1e-20?Vec3(-v.y,v.x,0)/std::sqrt(q):Vec3(1,0,0),b=cross(v,t);
 double radius=std::sqrt(r.uniform()),phi=2*pi*r.uniform();double p=radius*std::cos(phi),s=radius*std::sin(phi),blend=.5*(1+v.z);
 s=(1-blend)*std::sqrt(std::max(0.,1-p*p))+blend*s;
 Vec3 h=t*p+b*s+v*std::sqrt(std::max(0.,1-p*p-s*s));return normalize(Vec3(ax*h.x,ay*h.y,std::max(1e-12,h.z)));
}
inline Vec3 sample_visible_ggx(Vec3 v,double a,RNG&r){return sample_visible_ggx_aniso(v,a,a,r);}
inline Hit modified_normal(const Material&m,Hit h){
 if(!m.texture)return h;Frame f(h.n,h.tangent);Vec3 n;
 if(m.type==MaterialType::NormalMap)n=normalize(m.texture->lookup(h.u,h.v)*2-Vec3(1));
 else{
  double du=1./m.texture->width,dv=1./m.texture->height;
  auto height=[&](double u,double v){return m.texture->lookup(u,v).x;};
  double a=(height(h.u+du,h.v)-height(h.u-du,h.v))/(2*du),b=(height(h.u,h.v+dv)-height(h.u,h.v-dv))/(2*dv);
  n=normalize(Vec3(-m.bump_scale*a,-m.bump_scale*b,1));
 }
 Vec3 world=normalize(f.world(n));if(dot(world,h.gn)>1e-5)h.n=world;return h;
}
struct NullEval {Dual numerator;double pdf=0;Mueller pol;};
inline NullEval null_component(const Material&m,const Hit&hit,Vec3 wo,double nm,bool ad){
 if(m.type==MaterialType::Null){auto a=m.albedo(nm,hit,ad);return {a,1,Mueller::identity()*a.v};}
 if(m.type==MaterialType::Polarizer||m.type==MaterialType::Retarder){double a=axis_angle(wo,m.axis)+m.angle;auto p=m.type==MaterialType::Polarizer?polarizer(a):retarder(a,m.retardance*550/nm);return {p.at(0,0),1,p};}
 if(m.type==MaterialType::ThinGlass){double f=fresnel_dielectric(std::abs(dot(hit.n,wo)),1,m.ior(nm)),t=1-2*f/(1+f);return {t,t,Mueller::depolarizer(t)};}
 if(m.type==MaterialType::Blend){
  auto a=null_component(*m.child1,hit,wo,nm,ad),b=null_component(*m.child2,hit,wo,nm,ad);
  return {a.numerator*Dual(1-m.weight)+b.numerator*Dual(m.weight),a.pdf*(1-m.weight)+b.pdf*m.weight,a.pol*(1-m.weight)+b.pol*m.weight};
 }
 if(m.wrapper()){
  Hit h=(m.type==MaterialType::NormalMap||m.type==MaterialType::BumpMap)?modified_normal(m,hit):hit;
  auto a=null_component(*m.child1,h,wo,nm,ad);
  if(m.type==MaterialType::Mask){double c=m.coverage(hit);return {Dual(1-c)+a.numerator*Dual(c),1-c+c*a.pdf,Mueller::identity()*(1-c)+a.pol*c};}
  return a;
 }
 return {};
}
struct BSDFEval {Dual f;double pdf=0;Mueller pol;};
struct BSDFSample {Vec3 wi;Dual weight;double pdf=0;bool delta=false,transmission=false,medium_transition=false;Mueller pol;};
inline BSDFEval evaluate_bsdf(const Material&m,const Hit&hit,Vec3 wo,Vec3 wi,double nm,double ni,double nt,bool ad){
 BSDFEval result;
 if(!m.two_sided&&!hit.front&&!m.dielectric())return result;
 if(m.wrapper()){
  if(!m.child1)throw std::runtime_error("Missing nested BSDF");
  Hit h=(m.type==MaterialType::NormalMap||m.type==MaterialType::BumpMap)?modified_normal(m,hit):hit;
  if(m.type==MaterialType::TwoSided)h.front=true;
  result=evaluate_bsdf(*m.child1,h,wo,wi,nm,ni,nt,ad);
  if(m.type==MaterialType::NormalMap||m.type==MaterialType::BumpMap){double ratio=std::abs(dot(h.n,wi))/std::max(1e-15,std::abs(dot(hit.n,wi)));result.f=result.f*Dual(ratio);result.pol=result.pol*ratio;}
  if(m.type==MaterialType::Mask){double c=m.coverage(hit);result.f=result.f*Dual(c);result.pdf*=c;result.pol=result.pol*c;}
  return result;
 }
 if(m.type==MaterialType::Blend){
  if(!m.child1||!m.child2)throw std::runtime_error("Missing blend BSDFs");
  auto a=evaluate_bsdf(*m.child1,hit,wo,wi,nm,ni,nt,ad),b=evaluate_bsdf(*m.child2,hit,wo,wi,nm,ni,nt,ad);
  return {a.f*Dual(1-m.weight)+b.f*Dual(m.weight),a.pdf*(1-m.weight)+b.pdf*m.weight,a.pol*(1-m.weight)+b.pol*m.weight};
 }
 double co=dot(hit.n,wo),ci=dot(hit.n,wi);if(co<=0||std::abs(ci)<1e-12||m.delta())return result;
 if(m.type==MaterialType::Diffuse||m.type==MaterialType::DiffuseTransmission){
  bool transmission=m.type==MaterialType::DiffuseTransmission;if(transmission?ci>=0:ci<=0)return result;
  result.f=m.albedo(nm,hit,ad)/Dual(pi);result.pdf=std::abs(ci)/pi;result.pol=Mueller::depolarizer(result.f.v);return result;
 }
 double ax=m.alpha_u>0?m.alpha_u:std::max(.002,m.roughness*m.roughness),ay=m.alpha_v>0?m.alpha_v:ax;
 Frame frame(hit.n,hit.tangent);Vec3 vo=frame.local(wo),vi=frame.local(wi);
 if(m.type==MaterialType::Metal||m.type==MaterialType::Plastic){
  if(ci<=0)return result;Vec3 half=normalize(wo+wi);double ch=dot(hit.n,half),vh=dot(wo,half);if(ch<=0||vh<=0)return result;
  double D=ggx_D_aniso(frame.local(half),ax,ay),G1=1/(1+ggx_lambda_aniso(vo,ax,ay));
  double G=1/(1+ggx_lambda_aniso(vo,ax,ay)+ggx_lambda_aniso(vi,ax,ay));
  double pdfh=D*G1*vh/co;
  Mueller local=m.type==MaterialType::Metal?conductor_mueller(vh,m.eta.eval(nm),m.k.eval(nm)):dielectric_mueller(vh,1,m.ior(nm),false);
  double factor=D*G/(4*co*ci),spec=local.at(0,0)*factor;
  result.pol=frame_scattering(local,wo,wi,half)*factor;
  if(m.type==MaterialType::Metal){double tint=Spectrum(hit.tint).eval(nm)*(m.texture?m.texture->eval(hit.u,hit.v,nm):1);result.f=spec*tint;result.pol=result.pol*tint;result.pdf=pdfh/(4*vh);}
  else{
   double Fo=fresnel_dielectric(co,1,m.ior(nm)),Fi=fresnel_dielectric(ci,1,m.ior(nm));Dual diff=m.albedo(nm,hit,ad)*Dual((1-Fo)*(1-Fi)/pi);
   result.f=Dual(spec)+diff;result.pdf=.5*pdfh/(4*vh)+.5*ci/pi;result.pol=result.pol+Mueller::depolarizer(diff.v);
  }
  return result;
 }
 if(m.type==MaterialType::RoughGlass){
  bool reflection=ci>0;double ratio=nt/ni;Vec3 half=normalize(wo+wi*(reflection?1:ratio));if(dot(hit.n,half)<0)half=-half;
  double oh=dot(wo,half),ih=dot(wi,half);if(oh*co<=0||ih*ci<=0)return result;
  double D=ggx_D_aniso(frame.local(half),ax,ay),G=1/(1+ggx_lambda_aniso(vo,ax,ay)+ggx_lambda_aniso(vi,ax,ay));
  double F=fresnel_dielectric(oh,ni,nt),ph=D/(1+ggx_lambda_aniso(vo,ax,ay))*std::abs(oh)/std::abs(co),factor;
  if(reflection){factor=D*G/(4*std::abs(co*ci));result.pdf=F*ph/(4*std::abs(oh));}
  else{double denom=sqr(ih+oh/ratio);if(denom<1e-22)return result;factor=D*G*std::abs(ih*oh/(ci*co*denom))/(ratio*ratio);result.pdf=(1-F)*ph*std::abs(ih)/denom;}
  Mueller local=dielectric_mueller(oh,ni,nt,!reflection);result.f=local.at(0,0)*factor;result.pol=frame_scattering(local,wo,wi,half)*factor;return result;
 }
 return result;
}
inline BSDFSample sample_bsdf(const Material&m,const Hit&hit,Vec3 wo,double nm,double ni,double nt,RNG&r,bool ad,bool polarized){
 BSDFSample s;
 if(!m.two_sided&&!hit.front&&!m.dielectric())return s;
 if(m.wrapper()){
  if(!m.child1)throw std::runtime_error("Missing nested BSDF");
  double coverage=m.type==MaterialType::Mask?m.coverage(hit):1;
  if(m.type==MaterialType::Mask&&r.uniform()>=coverage){auto a=null_component(m,hit,wo,nm,ad);s.wi=-wo;s.weight=a.numerator/Dual(a.pdf);s.pdf=a.pdf;s.delta=true;s.transmission=true;s.pol=a.pol*(1/a.pdf);return s;}
  Hit h=(m.type==MaterialType::NormalMap||m.type==MaterialType::BumpMap)?modified_normal(m,hit):hit;
  if(m.type==MaterialType::TwoSided)h.front=true;
  s=sample_bsdf(*m.child1,h,wo,nm,ni,nt,r,ad,polarized);s.pdf*=coverage;
  if(s.delta&&s.transmission&&!s.medium_transition){auto a=null_component(m,hit,wo,nm,ad);s.pdf=a.pdf;if(a.pdf>0){s.weight=a.numerator/Dual(a.pdf);s.pol=a.pol*(1/a.pdf);}}
  return s;
 }
 if(m.type==MaterialType::Blend){
  bool second=r.uniform()<m.weight;double selected=second?m.weight:1-m.weight;
  s=sample_bsdf(second?*m.child2:*m.child1,hit,wo,nm,ni,nt,r,ad,polarized);
  if(s.delta){
   s.pdf*=selected;
   if(s.transmission&&!s.medium_transition){auto a=null_component(m,hit,wo,nm,ad);s.pdf=a.pdf;if(a.pdf>0){s.weight=a.numerator/Dual(a.pdf);s.pol=a.pol*(1/a.pdf);}}
   return s;
  }
  auto e=evaluate_bsdf(m,hit,wo,s.wi,nm,ni,nt,ad);s.pdf=e.pdf;if(s.pdf>0){double factor=std::abs(dot(hit.n,s.wi))/s.pdf;s.weight=e.f*Dual(factor);s.pol=e.pol*factor;}return s;
 }
 Frame frame(hit.n,hit.tangent);
 if(m.type==MaterialType::Null){s.wi=-wo;s.weight=m.albedo(nm,hit,ad);s.pdf=1;s.delta=true;s.transmission=true;s.pol=Mueller::identity()*s.weight.v;return s;}
 if(m.type==MaterialType::Polarizer||m.type==MaterialType::Retarder){
  s.wi=-wo;s.delta=true;s.transmission=true;s.pdf=1;double a=axis_angle(wo,m.axis)+m.angle;
  s.pol=m.type==MaterialType::Polarizer?polarizer(a):retarder(a,m.retardance*550/nm);s.weight=s.pol.at(0,0);return s;
 }
 if(m.type==MaterialType::ThinGlass){
  double F=fresnel_dielectric(dot(wo,hit.n),1,m.ior(nm));F=2*F/(1+F);s.transmission=r.uniform()>=F;
  s.wi=s.transmission?-wo:reflect(-wo,hit.n);s.pdf=s.transmission?1-F:F;s.delta=true;s.weight=1;s.pol=Mueller::depolarizer(1);return s;
 }
 if(m.type==MaterialType::Glass||m.type==MaterialType::Mirror){
  double co=dot(wo,hit.n),F=m.type==MaterialType::Mirror?1:fresnel_dielectric(co,ni,nt);Vec3 transmitted;
  bool tr=m.type==MaterialType::Glass&&r.uniform()>=F&&refract(-wo,hit.n,ni/nt,transmitted);
  s.wi=tr?transmitted:reflect(-wo,hit.n);s.delta=true;s.transmission=tr;s.medium_transition=tr;s.pdf=tr?1-F:F;s.weight=tr?sqr(ni/nt):1;
  if(m.type==MaterialType::Mirror)s.weight=fresnel_conductor(co,m.eta.eval(nm),m.k.eval(nm));
  if(polarized){
   Mueller local=m.type==MaterialType::Mirror?conductor_mueller(co,m.eta.eval(nm),m.k.eval(nm)):dielectric_mueller(co,ni,nt,tr);
   if(m.type==MaterialType::Mirror){s.weight=local.at(0,0);s.pdf=1;}
   s.pol=frame_scattering(local,wo,s.wi,hit.n)*((tr?sqr(ni/nt):1)/std::max(1e-20,s.pdf));
  }else s.pol=Mueller::depolarizer(s.weight.v);return s;
 }
 if(m.type==MaterialType::Emitter)return s;
 double ax=m.alpha_u>0?m.alpha_u:std::max(.002,m.roughness*m.roughness),ay=m.alpha_v>0?m.alpha_v:ax;
 if(m.type==MaterialType::Diffuse||m.type==MaterialType::DiffuseTransmission||(m.type==MaterialType::Plastic&&r.uniform()<.5)){
  s.wi=frame.world(cosine_hemisphere(r));if(m.type==MaterialType::DiffuseTransmission){s.wi=-s.wi;s.transmission=true;}
 }else{
  Vec3 half=frame.world(sample_visible_ggx_aniso(frame.local(wo),ax,ay,r));
  if(m.type==MaterialType::RoughGlass&&r.uniform()>fresnel_dielectric(dot(wo,half),ni,nt)){
   if(!refract(-wo,half,ni/nt,s.wi))return s;s.transmission=true;s.medium_transition=true;if(dot(s.wi,hit.n)>=0)return BSDFSample{};
  }else{s.wi=reflect(-wo,half);if(dot(s.wi,hit.n)<=0)return BSDFSample{};}
 }
 if(s.transmission?dot(s.wi,hit.gn)>=0:dot(s.wi,hit.gn)<=0)return BSDFSample{};
 auto e=evaluate_bsdf(m,hit,wo,s.wi,nm,ni,nt,ad);s.pdf=e.pdf;if(s.pdf<=0)return BSDFSample{};
 double factor=std::abs(dot(hit.n,s.wi))/s.pdf;s.weight=e.f*Dual(factor);s.pol=polarized?e.pol*factor:Mueller::depolarizer(s.weight.v);return s;
}
}

