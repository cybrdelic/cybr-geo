#pragma once
#include "io.hpp"
#include <cstdio>
namespace cybr {
struct PixelState {Vec3 xyz;std::array<Vec3,3> gradient{};std::array<Vec3,4> stokes{};double sy=0,sy2=0;};
inline uint64_t hash_bytes(const void*data,size_t size,uint64_t h=1469598103934665603ULL){auto p=static_cast<const unsigned char*>(data);for(size_t i=0;i<size;i++){h^=p[i];h*=1099511628211ULL;}return h;}
inline uint64_t hash_file(const std::string&name,uint64_t h=1469598103934665603ULL){std::ifstream f(name,std::ios::binary);if(!f)throw std::runtime_error("Cannot hash input "+name);std::array<char,65536> block;while(f){f.read(block.data(),block.size());h=hash_bytes(block.data(),size_t(f.gcount()),h);}return h;}
inline uint64_t scene_signature(const std::string&path,const Scene&s){
 std::ifstream f(path);std::string line;uint64_t h=1469598103934665603ULL;
 while(std::getline(f,line)){
  if(line.rfind("settings ",0)==0){std::istringstream in(line);std::string token;int i=0;while(in>>token){if(i!=3&&i!=7&&i!=9)h=hash_bytes(token.data(),token.size(),h);i++;}}
  else h=hash_bytes(line.data(),line.size(),h);
 }
 for(auto&p:s.dependencies)h=hash_file(p,h);
 int cfg[]={s.settings.width,s.settings.height,s.settings.bands};return hash_bytes(cfg,sizeof(cfg),h);
}
inline void save_checkpoint(const std::string&path,uint64_t signature,uint64_t completed,int w,int h,const std::vector<PixelState>&pixels){
 std::string temporary=path+".tmp";auto parent=std::filesystem::path(path).parent_path();if(!parent.empty())std::filesystem::create_directories(parent);
 std::ofstream f(temporary,std::ios::binary);if(!f)throw std::runtime_error("Cannot write checkpoint");
 const char magic[16]="CYBR-FILM-0002";uint64_t header[]={signature,completed,uint64_t(w),uint64_t(h),uint64_t(sizeof(PixelState))};
 f.write(magic,16);f.write(reinterpret_cast<char*>(header),sizeof(header));f.write(reinterpret_cast<const char*>(pixels.data()),std::streamsize(pixels.size()*sizeof(PixelState)));
 uint64_t checksum=hash_bytes(pixels.data(),pixels.size()*sizeof(PixelState),hash_bytes(header,sizeof(header)));f.write(reinterpret_cast<char*>(&checksum),8);f.flush();if(!f)throw std::runtime_error("Checkpoint write failed");f.close();
 if(std::rename(temporary.c_str(),path.c_str())!=0)throw std::runtime_error("Cannot atomically replace checkpoint; old checkpoint preserved");
}
inline uint64_t load_checkpoint(const std::string&path,uint64_t signature,int w,int h,std::vector<PixelState>&pixels){
 std::ifstream f(path,std::ios::binary);if(!f)throw std::runtime_error("Cannot read checkpoint");char magic[16];uint64_t header[5];f.read(magic,16);f.read(reinterpret_cast<char*>(header),sizeof(header));
 if(!f||std::memcmp(magic,"CYBR-FILM-0002",14)!=0||header[0]!=signature||header[2]!=uint64_t(w)||header[3]!=uint64_t(h)||header[4]!=sizeof(PixelState))throw std::runtime_error("Checkpoint does not match scene, assets, film or binary ABI");
 f.read(reinterpret_cast<char*>(pixels.data()),std::streamsize(pixels.size()*sizeof(PixelState)));uint64_t stored;f.read(reinterpret_cast<char*>(&stored),8);if(!f)throw std::runtime_error("Truncated checkpoint");
 uint64_t checksum=hash_bytes(pixels.data(),pixels.size()*sizeof(PixelState),hash_bytes(header,sizeof(header)));if(checksum!=stored)throw std::runtime_error("Checkpoint checksum mismatch");char extra;if(f.get(extra))throw std::runtime_error("Unexpected checkpoint trailing bytes");return header[1];
}
struct FilterSample {double offset=0,weight=1;};
inline double mitchell(double x){x=std::abs(x);constexpr double B=1./3,C=1./3;if(x<1)return ((12-9*B-6*C)*x*x*x+(-18+12*B+6*C)*x*x+6-2*B)/6;if(x<2)return ((-B-6*C)*x*x*x+(6*B+30*C)*x*x+(-12*B-48*C)*x+8*B+24*C)/6;return 0;}
inline double sinc(double x){return std::abs(x)<1e-10?1:std::sin(pi*x)/(pi*x);}
inline double lanczos(double x){return std::abs(x)<3?sinc(x)*sinc(x/3):0;}
inline FilterSample sample_filter(const std::string&kind,RNG&r){
 if(kind=="box")return {r.uniform()-.5,1};if(kind=="tent")return {r.uniform()-r.uniform(),1};
 if(kind=="gaussian"){double x;do{x=.5*std::sqrt(-2*std::log(r.uniform()))*std::cos(2*pi*r.uniform());}while(std::abs(x)>2);return {x,1};}
 if(kind=="mitchell"){double x=(r.uniform()-.5)*4;return {x,4*mitchell(x)};}
 if(kind=="lanczos"){
  static const double normalization=[](){double sum=0;for(int i=0;i<32768;i++)sum+=lanczos(-3+(i+.5)*6/32768);return sum*6/32768;}();
  double x=(r.uniform()-.5)*6;return {x,6*lanczos(x)/normalization};
 }
 throw std::runtime_error("Unknown reconstruction filter: "+kind);
}
}

