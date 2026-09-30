#pragma once
#include "math.hpp"
#include <memory>
#include <string>
#ifdef _WIN32
#include <windows.h>
#else
#include <dlfcn.h>
#endif
namespace cybr {
struct ShaderLibrary {
 using Function=void(*)(double,double,double,const double*,double*,double*);
 void* handle=nullptr;Function function=nullptr;
 explicit ShaderLibrary(const std::string&path){
#ifdef _WIN32
  handle=reinterpret_cast<void*>(LoadLibraryA(path.c_str()));if(handle)function=reinterpret_cast<Function>(GetProcAddress(static_cast<HMODULE>(handle),"cybr_shader"));
#else
  handle=dlopen(path.c_str(),RTLD_NOW|RTLD_LOCAL);if(handle)function=reinterpret_cast<Function>(dlsym(handle,"cybr_shader"));
#endif
  if(!function)throw std::runtime_error("Cannot load compiled CYBR material shader: "+path);
 }
 ~ShaderLibrary(){if(handle){
#ifdef _WIN32
  FreeLibrary(static_cast<HMODULE>(handle));
#else
  dlclose(handle);
#endif
 }}
 ShaderLibrary(const ShaderLibrary&)=delete;ShaderLibrary& operator=(const ShaderLibrary&)=delete;
 Dual evaluate(double nm,double u,double v,const std::array<double,3>&parameters,bool ad)const{
  Dual result;double gradient[3];function(nm,u,v,parameters.data(),&result.v,gradient);
  if(!std::isfinite(result.v)||result.v<0||result.v>1)throw std::runtime_error("Shader reflectance must be finite and lie in [0,1]");
  if(ad)for(int i=0;i<3;i++){if(!std::isfinite(gradient[i]))throw std::runtime_error("Nonfinite shader derivative");result.d[i]=gradient[i];}return result;
 }
};
}

