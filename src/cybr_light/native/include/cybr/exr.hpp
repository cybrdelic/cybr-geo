#pragma once
// Independent uncompressed, single-part, float32 scanline OpenEXR writer.
// Layout reference: https://openexr.com/en/latest/OpenEXRFileLayout.html
// No OpenEXR library is linked. This is intentionally not a compressed/deep EXR codec.
#include "math.hpp"
#include <cstring>
#include <fstream>
#include <map>
#include <string>
#include <vector>
namespace cybr {
namespace exr_detail {
using Bytes=std::vector<unsigned char>;
inline void integer(Bytes&b,uint64_t n,int bytes){for(int i=0;i<bytes;i++)b.push_back(static_cast<unsigned char>((n>>(i*8))&255));}
inline void text(Bytes&b,const std::string&s){b.insert(b.end(),s.begin(),s.end());b.push_back(0);}
inline void floating(Bytes&b,float v){uint32_t bits;std::memcpy(&bits,&v,4);integer(b,bits,4);}
inline void attribute(Bytes&b,const std::string&name,const std::string&type,const Bytes&data){text(b,name);text(b,type);integer(b,data.size(),4);b.insert(b.end(),data.begin(),data.end());}
inline void write(std::ostream&out,const Bytes&b){out.write(reinterpret_cast<const char*>(b.data()),std::streamsize(b.size()));}
}
inline void write_exr(const std::string&path,int width,int height,
                     const std::map<std::string,std::vector<float>>&channels){
 using namespace exr_detail;
 if(width<1||height<1||channels.empty())throw std::runtime_error("Invalid EXR dimensions/channels");
 Bytes header;integer(header,20000630,4);integer(header,2,4);Bytes list;
 for(auto&entry:channels){
  if(entry.first.empty()||entry.first.size()>31||entry.first.find('\0')!=std::string::npos||entry.second.size()!=size_t(width)*height)throw std::runtime_error("Invalid EXR channel");
  text(list,entry.first);integer(list,2,4);integer(list,0,4);integer(list,1,4);integer(list,1,4);
 }
 list.push_back(0);attribute(header,"channels","chlist",list);attribute(header,"compression","compression",Bytes{0});
 Bytes window;integer(window,0,4);integer(window,0,4);integer(window,uint32_t(width-1),4);integer(window,uint32_t(height-1),4);
 attribute(header,"dataWindow","box2i",window);attribute(header,"displayWindow","box2i",window);attribute(header,"lineOrder","lineOrder",Bytes{0});
 Bytes one;floating(one,1);Bytes zero;floating(zero,0);floating(zero,0);
 attribute(header,"pixelAspectRatio","float",one);attribute(header,"screenWindowCenter","v2f",zero);attribute(header,"screenWindowWidth","float",one);
 std::string software="CYBR LIGHT independent spectral renderer";attribute(header,"software","string",Bytes(software.begin(),software.end()));
 header.push_back(0);
 uint64_t row_bytes=uint64_t(width)*channels.size()*4;
 if(row_bytes>uint64_t(std::numeric_limits<int32_t>::max()))throw std::runtime_error("EXR scanline too large");
 std::ofstream out(path,std::ios::binary);if(!out)throw std::runtime_error("Cannot open EXR output: "+path);write(out,header);
 uint64_t offset=header.size()+uint64_t(height)*8;Bytes offsets;offsets.reserve(size_t(height)*8);
 for(int y=0;y<height;y++){integer(offsets,offset,8);offset+=row_bytes+8;}write(out,offsets);
 Bytes row;row.reserve(size_t(row_bytes+8));
 for(int y=0;y<height;y++){
  row.clear();integer(row,uint32_t(y),4);integer(row,row_bytes,4);
  for(auto&entry:channels)for(int x=0;x<width;x++)floating(row,entry.second[size_t(y)*width+x]);write(out,row);
 }
 out.flush();if(!out)throw std::runtime_error("EXR write failed: "+path);
}
inline void write_rgb_exr(const std::string&path,const std::vector<Vec3>&pixels,int w,int h){
 std::map<std::string,std::vector<float>> channels;
 for(auto&name:{"R","G","B"})channels[name].resize(pixels.size());
 for(size_t i=0;i<pixels.size();i++){channels["R"][i]=float(pixels[i].x);channels["G"][i]=float(pixels[i].y);channels["B"][i]=float(pixels[i].z);}
 write_exr(path,w,h,channels);
}
}

