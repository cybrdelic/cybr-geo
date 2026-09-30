#include "cybr/exr.hpp"
#include "cybr/film.hpp"
#include "cybr/photonmap.hpp"
#include <iostream>
#include <chrono>
#include <mutex>
#ifdef _OPENMP
#include <omp.h>
#endif
using namespace cybr;
int main(int argc,char**argv){
 try{
  std::string scene_path,prefix="render",resume,checkpoint;int spp=-1,width=-1,height=-1,bands=-1,threads=-1;
  auto arg=[&](int&i)->std::string{if(++i>=argc)throw std::runtime_error("Missing command-line argument");return argv[i];};
  for(int i=1;i<argc;i++){
   std::string a=argv[i];if(a=="--scene")scene_path=arg(i);else if(a=="--out")prefix=arg(i);else if(a=="--resume")resume=arg(i);else if(a=="--checkpoint")checkpoint=arg(i);
   else if(a=="--spp")spp=std::stoi(arg(i));else if(a=="--bands")bands=std::stoi(arg(i));else if(a=="--threads")threads=std::stoi(arg(i));else if(a=="--size"){width=std::stoi(arg(i));height=std::stoi(arg(i));}
   else if(a=="--features"){std::cout<<R"({"version":"0.2","native_cpu":true,"spectral":true,"polarized":true,"native_ad":"three diffuse or compiled spectral shader parameters","integrators":["path","volpath","direct","ao","photonmap"],"checkpoint_resume":true,"cuda_in_this_executable":false})"<<'\n';return 0;}
   else if(a=="--help"){std::cout<<"cybr-light --scene FILE.cys --out PREFIX [--size W H] [--spp TOTAL] [--bands N] [--threads N] [--resume FILE] [--checkpoint FILE]\n";return 0;}
   else throw std::runtime_error("Unknown argument: "+a);
  }
  if(scene_path.empty())throw std::runtime_error("--scene is required");Scene scene;read_scene(scene_path,scene);auto&cfg=scene.settings;
  if(spp>=0)cfg.spp=spp;if(width>=0)cfg.width=width;if(height>=0)cfg.height=height;if(bands>=0)cfg.bands=bands;if(threads>=0)cfg.threads=threads;
  if(cfg.width<1||cfg.height<1||cfg.spp<1||cfg.bands<1||cfg.bands>128||cfg.threads<1||cfg.width>16384||cfg.height>16384||uint64_t(cfg.width)*cfg.height>8000000)throw std::runtime_error("Invalid or excessive render settings");
  if(cfg.integrator!="path"&&cfg.integrator!="volpath"&&cfg.integrator!="direct"&&cfg.integrator!="ao"&&cfg.integrator!="photonmap")throw std::runtime_error("Unsupported integrator: "+cfg.integrator);
  if(cfg.sampler<0||cfg.sampler>1)throw std::runtime_error("Unsupported sampler");RNG filter_test(0);sample_filter(cfg.filter,filter_test);
  scene.build();auto outparent=std::filesystem::path(prefix).parent_path();if(!outparent.empty())std::filesystem::create_directories(outparent);
#ifdef _OPENMP
  omp_set_num_threads(cfg.threads);
#endif
  uint64_t meshlet_vertices=0;for(const auto&block:scene.meshlets)meshlet_vertices+=block->vertices.size();
  size_t count=size_t(cfg.width)*cfg.height;std::vector<PixelState> accumulator(count);uint64_t signature=scene_signature(scene_path,scene),begin_sample=0;
  if(!resume.empty())begin_sample=load_checkpoint(resume,signature,cfg.width,cfg.height,accumulator);
  if(begin_sample>uint64_t(cfg.spp))throw std::runtime_error("Checkpoint has more samples than requested total");
  auto start=std::chrono::steady_clock::now();PhotonMap photonmap;if(cfg.integrator=="photonmap")photonmap.build(scene);
  uint64_t bad=0,total_bounces=0;std::mutex error_mutex;std::string error;
  std::cerr<<"CYBR LIGHT 0.2: "<<cfg.width<<"x"<<cfg.height<<", samples "<<begin_sample<<".."<<cfg.spp<<", "<<cfg.bands<<" wavelengths; integrator="<<cfg.integrator<<"; primitives="<<scene.primitives.size()<<"\n";
#pragma omp parallel for schedule(dynamic,1) reduction(+:bad,total_bounces)
  for(int y=0;y<cfg.height;y++)for(int x=0;x<cfg.width;x++){
   try{
    size_t id=size_t(y)*cfg.width+x;auto&pixel=accumulator[id];
    for(uint64_t sample=begin_sample;sample<uint64_t(cfg.spp);sample++){
     uint64_t seed=mix64(cfg.seed^mix64(id)^mix64(sample+0xc001));RNG primary(seed);primary.configure(sample,mix64(cfg.seed^mix64(id)),cfg.sampler);
     auto fx=sample_filter(cfg.filter,primary),fy=sample_filter(cfg.filter,primary);double filter_weight=fx.weight*fy.weight;
     Ray ray=scene.camera.generate(x+.5+fx.offset,y+.5+fy.offset,cfg.width,cfg.height,primary);double shift=primary.uniform();RNG path_rng=primary;Vec3 packet;
     for(int band=0;band<cfg.bands;band++){
      double nm=cfg.integrator=="photonmap"?photonmap.wavelength(band):360+(band+shift)*470/cfg.bands;RNG rng=path_rng;
      auto result=cfg.integrator=="photonmap"?photonmap.trace(scene,ray,band,rng):trace_path(scene,ray,nm,rng);double value=result.radiance.v;total_bounces+=result.bounces;
      if(!std::isfinite(value)||value< -1e-10){bad++;continue;}Vec3 matching=scene.observer.xyz(nm)*(470/(106.856917101*cfg.bands))*filter_weight;packet+=matching*value;
      if(cfg.ad)for(int c=0;c<3;c++)pixel.gradient[c]+=matching*result.radiance.d[c];if(cfg.polarized)for(int c=0;c<4;c++)pixel.stokes[c]+=matching*result.stokes[c];
     }
     pixel.xyz+=packet;pixel.sy+=packet.y;pixel.sy2+=packet.y*packet.y;
    }
   }catch(const std::exception&e){std::lock_guard<std::mutex>lock(error_mutex);if(error.empty())error=e.what();}
  }
  if(!error.empty())throw std::runtime_error(error);if(bad)throw std::runtime_error("Invalid path samples: "+std::to_string(bad));
  if(!checkpoint.empty())save_checkpoint(checkpoint,signature,cfg.spp,cfg.width,cfg.height,accumulator);
  std::vector<Vec3> film(count),normal(count),albedo(count),depth(count),position(count),ids(count),stderr_image(count);
  std::array<std::vector<Vec3>,3> gradients;for(auto&g:gradients)if(cfg.ad)g.resize(count);std::array<std::vector<Vec3>,4> stokes;for(auto&g:stokes)if(cfg.polarized)g.resize(count);
#pragma omp parallel for schedule(static)
  for(int y=0;y<cfg.height;y++)for(int x=0;x<cfg.width;x++){
   size_t id=size_t(y)*cfg.width+x;auto&p=accumulator[id];film[id]=xyz_to_rgb(p.xyz/double(cfg.spp));double variance=cfg.spp>1?std::max(0.,(p.sy2-p.sy*p.sy/cfg.spp)/(cfg.spp-1)):0;stderr_image[id]=Vec3(std::sqrt(variance/cfg.spp));
   for(int c=0;c<3;c++)if(cfg.ad)gradients[c][id]=xyz_to_rgb(p.gradient[c]/double(cfg.spp));for(int c=0;c<4;c++)if(cfg.polarized)stokes[c][id]=xyz_to_rgb(p.stokes[c]/double(cfg.spp));
   RNG ar(cfg.seed+id);Camera camera=scene.camera;camera.aperture=0;camera.shutter_open=camera.shutter_close=(camera.shutter_open+camera.shutter_close)*.5;
   Hit hit;if(scene.bvh.hit(camera.generate(x+.5,y+.5,cfg.width,cfg.height,ar),hit)){auto guide=surface_guide(scene.materials[hit.material],hit);normal[id]=guide.normal*.5+Vec3(.5);depth[id]=Vec3(hit.t);position[id]=hit.p;ids[id]=Vec3(hit.object);albedo[id]=guide.albedo;}
  }
  double seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
  write_pfm(prefix+".pfm",film,cfg.width,cfg.height);if(cfg.film_format=="openexr")write_rgb_exr(prefix+".exr",film,cfg.width,cfg.height);write_ppm(prefix+".ppm",film,cfg.width,cfg.height,cfg.exposure);
  write_pfm(prefix+"_normal.pfm",normal,cfg.width,cfg.height);write_pfm(prefix+"_albedo.pfm",albedo,cfg.width,cfg.height);write_pfm(prefix+"_depth.pfm",depth,cfg.width,cfg.height);write_pfm(prefix+"_position.pfm",position,cfg.width,cfg.height);write_pfm(prefix+"_object.pfm",ids,cfg.width,cfg.height);write_pfm(prefix+"_stderr.pfm",stderr_image,cfg.width,cfg.height);
  if(cfg.ad)for(int c=0;c<3;c++)write_pfm(prefix+"_d"+std::to_string(c)+".pfm",gradients[c],cfg.width,cfg.height);if(cfg.polarized)for(int c=0;c<4;c++)write_pfm(prefix+"_stokes"+std::to_string(c)+".pfm",stokes[c],cfg.width,cfg.height);
  double max_value=0,mean_y=0;for(auto v:film){max_value=std::max(max_value,std::max({v.x,v.y,v.z}));mean_y+=dot(v,Vec3(.2126,.7152,.0722));}mean_y/=count;
  std::ofstream report(prefix+".json");report<<std::setprecision(12)<<"{\n  \"renderer\": \"CYBR LIGHT 0.2\",\n  \"backend\": \"C++17 OpenMP CPU\",\n  \"image_generation_used\": false,\n  \"denoising_used\": false,\n  \"clamping_used\": false,\n  \"width\": "<<cfg.width<<",\n  \"height\": "<<cfg.height<<",\n  \"packets_per_pixel\": "<<cfg.spp<<",\n  \"resumed_from_samples\": "<<begin_sample<<",\n  \"wavelengths_per_packet\": "<<cfg.bands<<",\n  \"spectral_paths_per_pixel\": "<<cfg.spp*cfg.bands<<",\n  \"wavelength_range_nm\": [360,830],\n  \"observer\": \""<<(scene.observer.table.empty()?"analytic CIE 1931 approximation":"tabulated observer")<<"\",\n  \"integrator\": \""<<cfg.integrator<<"\",\n  \"filter\": \""<<cfg.filter<<"\",\n  \"sampler\": \""<<(cfg.sampler?"random-shift Halton, first 64 dimensions":"independent PCG")<<"\",\n  \"max_depth\": "<<cfg.max_depth<<",\n  \"rr_depth\": "<<cfg.rr_depth<<",\n  \"seed\": "<<cfg.seed<<",\n  \"primitives\": "<<scene.primitives.size()<<",\n  \"meshlets\": "<<scene.meshlets.size()<<",\n  \"indexed_vertices\": "<<meshlet_vertices<<",\n  \"primitive_storage_bytes\": "<<sizeof(Primitive)*scene.primitives.size()<<",\n  \"meshlet_vertex_bytes\": "<<sizeof(MeshletVertex)*meshlet_vertices<<",\n  \"bvh_nodes\": "<<scene.bvh.node_count()<<",\n  \"volumes\": "<<scene.volumes.size()<<",\n  \"polarized\": "<<(cfg.polarized?"true":"false")<<",\n  \"forward_ad\": "<<(cfg.ad?"true":"false")<<",\n  \"invalid_path_samples\": "<<bad<<",\n  \"photons_emitted\": "<<photonmap.emitted_count()<<",\n  \"photons_stored\": "<<photonmap.stored_count()<<",\n  \"render_seconds\": "<<seconds<<",\n  \"mean_linear_luminance\": "<<mean_y<<",\n  \"max_linear_channel\": "<<max_value<<"\n}\n";
  if(!report)throw std::runtime_error("Cannot write render report");std::cerr<<"Completed: "<<seconds<<" s; mean luminance="<<mean_y<<"; photons="<<photonmap.stored_count()<<"\n";return 0;
 }catch(const std::exception&e){std::cerr<<"ERROR: "<<e.what()<<"\n";return 1;}
}
