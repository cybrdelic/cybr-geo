#pragma once
#include "spectrum.hpp"
#include <memory>
#include <cstring>
namespace cybr {
// Linear spectral-control images. PFM input is linear, never silently gamma decoded.
struct ImageTexture {
 int width=0,height=0;std::vector<Vec3> pixels;double scale_u=1,scale_v=1;bool repeat=true;
 void load(const std::string&path){
  std::ifstream f(path,std::ios::binary);if(!f)throw std::runtime_error("Cannot open texture: "+path);
  std::string magic;double scale;f>>magic>>width>>height>>scale;f.get();
  if(magic!="PF"||width<1||height<1||width>32768||height>32768||scale==0||!std::isfinite(scale))throw std::runtime_error("Texture must be a valid RGB PFM");
  size_t n=size_t(width)*height;if(n>268435456)throw std::runtime_error("Texture exceeds memory limit");pixels.resize(n);
  uint16_t one=1;bool hostlittle=*reinterpret_cast<unsigned char*>(&one)==1;
  for(int y=height-1;y>=0;y--)for(int x=0;x<width;x++){
   float values[3];f.read(reinterpret_cast<char*>(values),sizeof(values));if(!f)throw std::runtime_error("Truncated texture");
   for(int c=0;c<3;c++){if((scale<0)!=hostlittle){auto*b=reinterpret_cast<unsigned char*>(&values[c]);std::reverse(b,b+4);}if(!std::isfinite(values[c]))throw std::runtime_error("Nonfinite texture");}
   pixels[size_t(y)*width+x]=Vec3(values[0],values[1],values[2])*std::abs(scale);
  }
 }
 Vec3 lookup(double u,double v)const{
  if(pixels.empty())return Vec3(1);u*=scale_u;v*=scale_v;
  if(repeat){u-=std::floor(u);v-=std::floor(v);}else{u=clamp(u);v=clamp(v);}
  double x=u*width-.5,y=v*height-.5;int ix=int(std::floor(x)),iy=int(std::floor(y));double tx=x-ix,ty=y-iy;
  auto at=[&](int a,int b){if(repeat){a=((a%width)+width)%width;b=((b%height)+height)%height;}else{a=std::clamp(a,0,width-1);b=std::clamp(b,0,height-1);}return pixels[size_t(b)*width+a];};
  return at(ix,iy)*(1-tx)*(1-ty)+at(ix+1,iy)*tx*(1-ty)+at(ix,iy+1)*(1-tx)*ty+at(ix+1,iy+1)*tx*ty;
 }
 double eval(double u,double v,double nm)const{return Spectrum(lookup(u,v)).eval(nm);}
};
}

