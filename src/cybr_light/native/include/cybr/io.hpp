#pragma once
#include "integrator.hpp"
#include <filesystem>
#include <iomanip>
#include <cstring>
namespace cybr {
inline Vec3 read_vec(std::istream&f){Vec3 v;if(!(f>>v.x>>v.y>>v.z))throw std::runtime_error("Expected three floating point values");return v;}
inline MaterialType parse_material(const std::string&name){
 if(name=="null")return MaterialType::Null;if(name=="thindielectric")return MaterialType::ThinGlass;if(name=="difftrans")return MaterialType::DiffuseTransmission;if(name=="blendbsdf")return MaterialType::Blend;if(name=="mask")return MaterialType::Mask;if(name=="twosided")return MaterialType::TwoSided;if(name=="normalmap")return MaterialType::NormalMap;if(name=="bumpmap")return MaterialType::BumpMap;
 if(name=="diffuse")return MaterialType::Diffuse;if(name=="metal")return MaterialType::Metal;if(name=="plastic")return MaterialType::Plastic;if(name=="glass")return MaterialType::Glass;if(name=="roughglass")return MaterialType::RoughGlass;if(name=="mirror")return MaterialType::Mirror;if(name=="polarizer")return MaterialType::Polarizer;if(name=="retarder")return MaterialType::Retarder;if(name=="emitter")return MaterialType::Emitter;throw std::runtime_error("Unsupported material: "+name);
}
inline void read_scene(const std::string&path,Scene&s){
 std::ifstream f(path);if(!f)throw std::runtime_error("Cannot open scene: "+path);std::string line;int lineno=0;std::vector<std::array<int,3>> links;
 while(std::getline(f,line)){lineno++;auto comment=line.find('#');if(comment!=std::string::npos)line.resize(comment);std::istringstream q(line);std::string cmd;if(!(q>>cmd))continue;
  try{
   if(cmd=="settings"){auto&v=s.settings;q>>v.width>>v.height>>v.spp>>v.bands>>v.max_depth>>v.rr_depth>>v.threads>>v.seed>>v.exposure>>v.polarized>>v.ad>>v.active_material>>v.mis>>v.nee;}
   else if(cmd=="camera"){auto&c=s.camera;c.origin=read_vec(q);c.target=read_vec(q);c.up=read_vec(q);q>>c.fov>>c.aperture>>c.focus>>c.shutter_open>>c.shutter_close>>c.orthographic>>c.ortho_scale;}
   else if(cmd=="material"){Material m;std::string type;q>>type;m.source_index=int(s.materials.size());m.type=parse_material(type);m.color=Spectrum(read_vec(q));q>>m.roughness>>m.ior_a>>m.ior_b;m.eta=Spectrum(read_vec(q));m.k=Spectrum(read_vec(q));m.absorption=Spectrum(read_vec(q));q>>m.emission>>m.kelvin;s.materials.push_back(m);}
   else if(cmd=="sidedness"){int i;q>>i;q>>s.materials.at(i).two_sided;}
   else if(cmd=="shader"){int i;std::string name;q>>i>>std::quoted(name);auto&m=s.materials.at(i);s.dependencies.push_back(std::filesystem::absolute(std::filesystem::path(path).parent_path()/name).string());m.shader=std::make_shared<ShaderLibrary>(s.dependencies.back());for(double&v:m.shader_parameters)q>>v;}
   else if(cmd=="film_format"){q>>s.settings.film_format;if(s.settings.film_format!="pfm"&&s.settings.film_format!="openexr")throw std::runtime_error("Unsupported film format");}
   else if(cmd=="render_options"){q>>s.settings.integrator>>s.settings.sampler>>s.settings.filter>>s.settings.ao_distance;}
   else if(cmd=="photon_options"){q>>s.settings.photon_count>>s.settings.photon_radius;}
   else if(cmd=="anisotropy"){int i;q>>i;auto&m=s.materials.at(i);q>>m.alpha_u>>m.alpha_v;if(m.alpha_u<=0||m.alpha_v<=0)throw std::runtime_error("Invalid anisotropic slopes");}
   else if(cmd=="nested"){int i,a,b;q>>i>>a>>b;links.push_back({i,a,b});q>>s.materials.at(i).weight;}
   else if(cmd=="coverage"){int i;q>>i;q>>s.materials.at(i).opacity;}
   else if(cmd=="bump_scale"){int i;q>>i;q>>s.materials.at(i).bump_scale;}
   else if(cmd=="emission_temperature"){int i;std::string name;q>>i>>std::quoted(name);auto t=std::make_shared<ImageTexture>();s.dependencies.push_back((std::filesystem::path(path).parent_path()/name).string());t->load(s.dependencies.back());t->repeat=false;s.materials.at(i).temperature_texture=t;}
   else if(cmd=="texture"){int i;std::string name;q>>i>>std::quoted(name);auto t=std::make_shared<ImageTexture>();s.dependencies.push_back((std::filesystem::path(path).parent_path()/name).string());t->load(s.dependencies.back());q>>t->scale_u>>t->scale_v>>t->repeat;s.materials.at(i).texture=t;}
   else if(cmd=="uv_checker"){int i;q>>i;s.materials.at(i).uv_checker=true;}
   else if(cmd=="surface_uv"){int i;q>>i;auto&p=s.primitives.at(i);p.edit().ta=read_vec(q);p.edit().tb=read_vec(q);p.edit().tc=read_vec(q);p.edit().has_uv=true;}
   else if(cmd=="camera_spherical"){s.camera.spherical=true;}
   else if(cmd=="environment_flat"){s.environment.flat=true;}
   else if(cmd=="envmap"){std::string name;q>>std::quoted(name)>>s.environment.rotation;auto t=std::make_shared<ImageTexture>();s.dependencies.push_back((std::filesystem::path(path).parent_path()/name).string());t->load(s.dependencies.back());s.environment.texture=t;}
   else if(cmd=="delta_light"){DeltaEmitter l;q>>l.kind;l.position=read_vec(q);l.direction=normalize(read_vec(q));l.intensity=Spectrum(read_vec(q));q>>l.scale>>l.cutoff>>l.beam;if(l.kind<0||l.kind>2||l.scale<0||l.cutoff<l.beam)throw std::runtime_error("Invalid delta emitter");s.delta_lights.push_back(l);}
   else if(cmd=="phase"){int i;q>>i;q>>s.volumes.at(i).phase;if(s.volumes.at(i).phase<0||s.volumes.at(i).phase>1)throw std::runtime_error("Unknown phase function");}
   else if(cmd=="disk"||cmd=="cylinder"){Primitive p;p.shape=cmd=="disk"?Shape::Disk:Shape::Cylinder;q>>p.material>>p.object;p.edit().a=read_vec(q);p.edit().b=read_vec(q);q>>p.radius;p.edit().velocity=read_vec(q);if(p.radius<=0||norm2(p.edit().b-(cmd=="cylinder"?p.edit().a:Vec3(0)))<1e-20)throw std::runtime_error("Invalid disk or cylinder");if(cmd=="disk")p.edit().b=normalize(p.edit().b);s.primitives.push_back(p);}
   else if(cmd=="checker"){int i;q>>i;s.materials.at(i).checker=0;q>>s.materials.at(i).checker;s.materials.at(i).second=Spectrum(read_vec(q));}
   else if(cmd=="optic"){int i;q>>i;auto&m=s.materials.at(i);m.axis=read_vec(q);q>>m.angle>>m.retardance;}
   else if(cmd=="sphere"){Primitive p;p.shape=Shape::Sphere;q>>p.material>>p.object;p.edit().a=read_vec(q);q>>p.radius;p.edit().velocity=read_vec(q);if(p.radius<=0)throw std::runtime_error("Sphere radius must be positive");s.primitives.push_back(p);}
   else if(cmd=="quad"){Primitive p;p.shape=Shape::Quad;q>>p.material>>p.object;p.edit().a=read_vec(q);p.edit().b=read_vec(q);p.edit().c=read_vec(q);p.edit().velocity=read_vec(q);s.primitives.push_back(p);}
   else if(cmd=="meshlets"){
    std::string name;q>>std::quoted(name);auto file=std::filesystem::path(path).parent_path()/name;
    s.dependencies.push_back(file.string());std::ifstream mesh(file,std::ios::binary);
    auto read=[&](void*data,size_t bytes){mesh.read(reinterpret_cast<char*>(data),bytes);if(!mesh)throw std::runtime_error("Truncated CLM1 meshlets");};
    char magic[4];uint64_t count;uint32_t packets;read(magic,4);read(&count,8);read(&packets,4);
    if(std::memcmp(magic,"CLM1",4)||!count||count>30000000||!packets||packets>count)throw std::runtime_error("Invalid CLM1 header");
    s.primitives.reserve(s.primitives.size()+count);uint64_t actual=0;
    for(uint32_t packet=0;packet<packets;packet++){
     uint16_t nv,nt;read(&nv,2);read(&nt,2);if(!nv||nv>64||!nt||nt>124||actual+nt>count)throw std::runtime_error("Invalid meshlet limits");
     auto block=std::make_unique<Meshlet>();block->vertices.resize(nv);read(block->vertices.data(),size_t(nv)*44);
     for(auto&vertex:block->vertices){auto a=vertex.value;for(int k=0;k<11;k++)if(!std::isfinite(a[k]))throw std::runtime_error("Nonfinite meshlet vertex");for(int k=8;k<11;k++)if(a[k]<0)throw std::runtime_error("Negative meshlet tint");Vec3 n{a[3],a[4],a[5]};if(norm2(n)>0){n=normalize(n);a[3]=float(n.x);a[4]=float(n.y);a[5]=float(n.z);}}
     const Meshlet* pointer=block.get();s.meshlets.push_back(std::move(block));
     for(int i=0;i<nt;i++){
      uint8_t ids[3];uint32_t mat,obj;read(ids,3);read(&mat,4);read(&obj,4);
      if(ids[0]>=nv||ids[1]>=nv||ids[2]>=nv||mat>=s.materials.size()||obj>16777215)throw std::runtime_error("Invalid meshlet index/material/object");
      Primitive p;p.shape=Shape::Triangle;p.material=mat;p.object=obj;p.meshlet=pointer;p.indices={ids[0],ids[1],ids[2]};
      if(p.area()>1e-15)s.primitives.push_back(p);
     }
     actual+=nt;
    }
    if(actual!=count||mesh.peek()!=std::char_traits<char>::eof())throw std::runtime_error("Meshlet count/trailing data mismatch");
   }
   else if(cmd=="triangle"){Primitive p;p.shape=Shape::Triangle;q>>p.material>>p.object;p.edit().a=read_vec(q);p.edit().b=read_vec(q);p.edit().c=read_vec(q);p.edit().na=read_vec(q);p.edit().nb=read_vec(q);p.edit().nc=read_vec(q);q>>p.edit().smooth;p.edit().velocity=read_vec(q);s.primitives.push_back(p);}
   else if(cmd=="environment"){s.environment.base=Spectrum(read_vec(q));q>>s.environment.strength;}
   else if(cmd=="env_lobe"){EnvironmentLobe l;l.direction=normalize(read_vec(q));q>>l.exponent>>l.strength>>l.kelvin;s.environment.lobes.push_back(l);}
   else if(cmd=="volume"){Volume v;q>>v.kind;v.bounds.lo=read_vec(q);v.bounds.hi=read_vec(q);v.extinction=Spectrum(read_vec(q));v.albedo=Spectrum(read_vec(q));q>>v.g>>v.scale>>v.density_majorant;if(std::abs(v.g)>=1||v.density_majorant<=0)throw std::runtime_error("Invalid phase g or density majorant");s.volumes.push_back(v);}
   else if(cmd=="cloud_lobe"){int i;CloudLobe l;q>>i;l.center=read_vec(q);l.radii=read_vec(q);q>>l.strength;if(std::min({l.radii.x,l.radii.y,l.radii.z})<=0)throw std::runtime_error("Invalid cloud radius");s.volumes.at(i).lobes.push_back(l);}
   else if(cmd=="grid"){int i;std::string name;q>>i>>std::quoted(name);auto&v=s.volumes.at(i);auto file=std::filesystem::path(path).parent_path()/name;s.dependencies.push_back(file.string());std::ifstream g(file,std::ios::binary);int dims[3];g.read(reinterpret_cast<char*>(dims),sizeof(dims));if(!g||dims[0]<2||dims[1]<2||dims[2]<2)throw std::runtime_error("Invalid volume grid header");v.nx=dims[0];v.ny=dims[1];v.nz=dims[2];size_t count=Volume::grid_count(v.nx,v.ny,v.nz);v.grid.resize(count);g.read(reinterpret_cast<char*>(v.grid.data()),std::streamsize(count*sizeof(float)));if(!g)throw std::runtime_error("Truncated volume grid");if(g.peek()!=std::char_traits<char>::eof())throw std::runtime_error("Trailing volume grid bytes");v.density_majorant=0;for(float x:v.grid){if(!std::isfinite(x)||x<0)throw std::runtime_error("Negative or nonfinite density");v.density_majorant=std::max(v.density_majorant,double(x));}v.kind=2;}
   else if(cmd=="observer"){std::string name;q>>std::quoted(name);s.settings.observer_path=(std::filesystem::path(path).parent_path()/name).string();s.dependencies.push_back(s.settings.observer_path);}
   else if(cmd=="spectrum"){int i;std::string parameter,name;q>>i>>parameter>>std::quoted(name);auto&m=s.materials.at(i);Spectrum* target=parameter=="color"?&m.color:parameter=="eta"?&m.eta:parameter=="k"?&m.k:parameter=="absorption"?&m.absorption:nullptr;if(!target)throw std::runtime_error("Unknown spectral parameter");s.dependencies.push_back((std::filesystem::path(path).parent_path()/name).string());std::ifstream data(s.dependencies.back());double a,b;while(data>>a>>b){if(b<0)throw std::runtime_error("Negative spectrum");target->table.emplace_back(a,b);}if(target->table.empty())throw std::runtime_error("Empty spectrum");std::sort(target->table.begin(),target->table.end());}
   else throw std::runtime_error("Unknown scene directive: "+cmd);
   if(q.fail())throw std::runtime_error("Missing or malformed scene argument");std::string trailing;if(q>>trailing)throw std::runtime_error("Unexpected trailing scene argument: "+trailing);
  }catch(const std::exception&e){throw std::runtime_error(path+":"+std::to_string(lineno)+": "+e.what());}
 }
 std::vector<int> state(s.materials.size(),0);
 std::function<void(int)> resolve=[&](int id){if(id<0||id>=int(s.materials.size()))throw std::runtime_error("Invalid nested material index");if(state[id]==2)return;if(state[id]==1)throw std::runtime_error("Cyclic nested BSDF graph");state[id]=1;for(auto&link:links)if(link[0]==id){resolve(link[1]);s.materials[id].child1=std::make_shared<Material>(s.materials[link[1]]);if(link[2]>=0){resolve(link[2]);s.materials[id].child2=std::make_shared<Material>(s.materials[link[2]]);}}state[id]=2;};
 for(int i=0;i<int(s.materials.size());i++)resolve(i);
}
inline void write_pfm(const std::string&path,const std::vector<Vec3>&pixels,int width,int height){
 std::ofstream f(path,std::ios::binary);if(!f)throw std::runtime_error("Cannot write: "+path);f<<"PF\n"<<width<<" "<<height<<"\n-1.0\n";for(int y=height-1;y>=0;y--)for(int x=0;x<width;x++){Vec3 v=pixels[y*width+x];float a[3]={float(v.x),float(v.y),float(v.z)};f.write(reinterpret_cast<char*>(a),sizeof(a));}if(!f)throw std::runtime_error("Image write failed");
}
inline double display_transfer(double x){x=std::max(0.,x);double f=clamp(x*(2.51*x+.03)/(x*(2.43*x+.59)+.14));return f<=.0031308?12.92*f:1.055*std::pow(f,1/2.4)-.055;}
inline void write_ppm(const std::string&path,const std::vector<Vec3>&pixels,int width,int height,double exposure){std::ofstream f(path,std::ios::binary);f<<"P6\n"<<width<<" "<<height<<"\n255\n";for(auto v:pixels){unsigned char a[3];for(int c=0;c<3;c++)a[c]=(unsigned char)std::lround(255*display_transfer(v[c]*exposure));f.write(reinterpret_cast<char*>(a),3);}if(!f)throw std::runtime_error("Display image write failed");}
}

