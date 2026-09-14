// Anisotropic Trowbridge-Reitz reflection with visible-normal sampling.
// Equations follow PBRT 4e, Roughness Using Microfacet Theory. The optional
// clearcoat is a conservative Schlick-attenuated opaque layered approximation.
#pragma once

Frame photoFrame(const Material&m,V n,V p){
 Frame fr(n);V axis=unit(m.grainAxis),relative=p-m.grainOrigin;
 V radial=relative-axis*dot(relative,axis),t=axis;
 if(m.pattern==8 || (m.pattern==1&&std::fabs(dot(n,axis))>.75f))t=cross(axis,radial);
 t=t-n*dot(t,n);
 if(dot(t,t)>1e-12f){fr.t=unit(t);fr.b=cross(n,fr.t);}
 if(m.rotation!=0){float c=std::cos(m.rotation),s=std::sin(m.rotation);V a=fr.t,b=fr.b;fr.t=a*c+b*s;fr.b=b*c-a*s;}
 return fr;
}

void photoAlpha(const Material&m,float &ax,float &ay){
 float alpha=std::max(.008f,m.rough*m.rough);
 float aspect=std::sqrt(1.f-.9f*std::min(.98f,std::fabs(m.aniso)));
 ax=alpha/aspect;ay=alpha*aspect;if(m.aniso<0)std::swap(ax,ay);
}
float photoD(V h,float ax,float ay){
 if(h.z<=0)return 0;float q=sqr(h.x/ax)+sqr(h.y/ay)+sqr(h.z);
 return 1.f/(PI*ax*ay*q*q);
}
float photoG1(V v,float ax,float ay){
 if(v.z<=0)return 0;
 float lambda=.5f*(std::sqrt(1.f+(sqr(ax*v.x)+sqr(ay*v.y))/sqr(v.z))-1.f);
 return 1.f/(1.f+lambda);
}
float photoCoatProbability(const Material&m){return .25f*m.coat;}
float photoDielectricFresnel(float c,float eta){
 c=clamp(std::fabs(c));float sin2=(1.f-c*c)/(eta*eta);
 if(sin2>=1.f)return 1.f;
 float ct=std::sqrt(std::max(0.f,1.f-sin2));
 float parallel=(eta*c-ct)/(eta*c+ct);
 float perpendicular=(c-eta*ct)/(c+eta*ct);
 return .5f*(parallel*parallel+perpendicular*perpendicular);
}
float photoG2(V v,V l,float ax,float ay){
 // Height-correlated Smith masking (PBRT 4e section 9.6).
 float gv=photoG1(v,ax,ay),gl=photoG1(l,ax,ay);
 return gv*gl/std::max(1e-12f,gv+gl-gv*gl);
}
V photoEval(const Material&m,const Frame&fr,V v,V l){
 V vv=fr.local(v),ll=fr.local(l);if(vv.z<=0||ll.z<=0)return V(0);
 V h=unit(vv+ll);float ax,ay;photoAlpha(m,ax,ay);
 float dielectric=sqr((m.ior-1.f)/(m.ior+1.f));
 V F=fresnel(dot(vv,h),m.color)*m.metal+V(photoDielectricFresnel(dot(vv,h),m.ior))*(1.f-m.metal);
 V base=F*(photoD(h,ax,ay)*photoG2(vv,ll,ax,ay)/(4.f*vv.z*ll.z));
 // Diffuse light crosses the dielectric interface on both entry and exit.
 // Using only the half-vector Fresnel can create energy at grazing angles.
 float entry=photoDielectricFresnel(vv.z,m.ior),leave=photoDielectricFresnel(ll.z,m.ior);
 base+=m.color*((1.f-m.metal)*(1.f-entry)*(1.f-leave)/PI);
 if(m.coat<=0)return base;
 float Fc=photoDielectricFresnel(dot(vv,h),1.5f);
 float ac=std::max(.008f,m.coatRough*m.coatRough);
 float coat=m.coat*Fc*Dggx(h.z,ac)*photoG2(vv,ll,ac,ac)/(4.f*vv.z*ll.z);
 float attenuation=(1.f-m.coat*photoDielectricFresnel(vv.z,1.5f))*(1.f-m.coat*photoDielectricFresnel(ll.z,1.5f));
 return base*attenuation+V(coat);
}
float photoPDF(const Material&m,const Frame&fr,V v,V l){
 V vv=fr.local(v),ll=fr.local(l);if(vv.z<=0||ll.z<=0)return 0;
 V h=unit(vv+ll);float ax,ay;photoAlpha(m,ax,ay);
 float ps=specProb(m),pc=photoCoatProbability(m);
 float base=ps*photoD(h,ax,ay)*photoG1(vv,ax,ay)/(4.f*vv.z)+(1.f-ps)*ll.z/PI;
 float ac=std::max(.008f,m.coatRough*m.coatRough);
 return (1.f-pc)*base+pc*Dggx(h.z,ac)*G1(vv.z,ac)/(4.f*vv.z);
}
V photoVisibleNormal(V v,float ax,float ay,RNG&rng){
 V vh=unit(V(ax*v.x,ay*v.y,std::max(0.f,v.z)));
 float lensq=vh.x*vh.x+vh.y*vh.y;
 V t1=lensq>0?V(-vh.y,vh.x,0)/std::sqrt(lensq):V(1,0,0),t2=cross(vh,t1);
 float r=std::sqrt(rng.uniform()),phi=2*PI*rng.uniform();
 float x=r*std::cos(phi),y=r*std::sin(phi),s=.5f*(1+vh.z);
 y=(1-s)*std::sqrt(std::max(0.f,1-x*x))+s*y;
 V nh=t1*x+t2*y+vh*std::sqrt(std::max(0.f,1-x*x-y*y));
 return unit(V(ax*nh.x,ay*nh.y,std::max(0.f,nh.z)));
}
V photoSample(const Material&m,const Frame&fr,V v,RNG&rng){
 V vv=fr.local(v);float ax,ay;
 if(rng.uniform()<photoCoatProbability(m))ax=ay=std::max(.008f,m.coatRough*m.coatRough);
 else{
  if(rng.uniform()>specProb(m)){
   float u=rng.uniform(),phi=2*PI*rng.uniform(),r=std::sqrt(u);
   return fr.world(V(r*std::cos(phi),r*std::sin(phi),std::sqrt(1-u)));
  }
  photoAlpha(m,ax,ay);
 }
 return fr.world(reflect(-vv,photoVisibleNormal(vv,ax,ay,rng)));
}

float radicalInverse(uint32_t bits){
 bits=(bits<<16)|(bits>>16);
 bits=((bits&0x55555555u)<<1)|((bits&0xaaaaaaaau)>>1);
 bits=((bits&0x33333333u)<<2)|((bits&0xccccccccu)>>2);
 bits=((bits&0x0f0f0f0fu)<<4)|((bits&0xf0f0f0f0u)>>4);
 bits=((bits&0x00ff00ffu)<<8)|((bits&0xff00ff00u)>>8);
 return bits*2.3283064365386963e-10f;
}
