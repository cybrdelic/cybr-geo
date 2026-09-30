#include "cybr/film.hpp"
#include "cybr/photonmap.hpp"
#include <iostream>
#include <functional>
#include <filesystem>
using namespace cybr;
struct Checks {
    int passed=0, failed=0;
    void operator()(const std::string &name, bool ok) {
        std::cout << (ok?"PASS ":"FAIL ") << name << '\n';
        ok?++passed:++failed;
    }
};
int main() {
    Checks check; RNG rng(927101); const int N=100000;
    Primitive disk; disk.shape=Shape::Disk; disk.edit().a={0,0,0}; disk.edit().b={0,0,1};disk.radius=2;
    Hit hit;check("disk exact axial intersection",disk.hit({{.5,.2,4},{0,0,-1}},1e-8,hit)&&std::abs(hit.t-4)<1e-12);
    Hit miss;check("disk rejects outside radius",!disk.hit({{2.1,0,4},{0,0,-1}},1e-8,miss));
    check("disk exact area",std::abs(disk.area()-4*pi)<1e-12);
    double mean_r2=0;bool in_disk=true;
    for(int i=0;i<N;i++){Vec3 p,n;disk.sample(rng,0,p,n);mean_r2+=norm2(p);in_disk=in_disk&&norm2(p)<4.00000001&&std::abs(dot(n,disk.edit().b)-1)<1e-12;}
    check("disk uniform-area sampler second moment",in_disk&&std::abs(mean_r2/N-2)<.015);
    Primitive cylinder;cylinder.shape=Shape::Cylinder;cylinder.edit().a={0,0,0};cylinder.edit().b={0,0,3};cylinder.radius=1;
    Hit ch;check("cylinder exact side intersection",cylinder.hit({{2,0,1.2},{-1,0,0}},1e-8,ch)&&std::abs(ch.t-1)<1e-12);
    Hit cm;check("uncapped cylinder rejects axial ray",!cylinder.hit({{0,0,4},{0,0,-1}},1e-8,cm));
    check("cylinder lateral area",std::abs(cylinder.area()-6*pi)<1e-12);
    bool on_cylinder=true;double mean_z=0;
    for(int i=0;i<N;i++){Vec3 p,n;cylinder.sample(rng,0,p,n);on_cylinder=on_cylinder&&std::abs(p.x*p.x+p.y*p.y-1)<1e-12&&p.z>=0&&p.z<=3;mean_z+=p.z;}
    check("cylinder uniform lateral sampling",on_cylinder&&std::abs(mean_z/N-1.5)<.015);
    Primitive triangle;triangle.shape=Shape::Triangle;triangle.edit().a={0,0,0};triangle.edit().b={1,0,0};triangle.edit().c={0,1,0};triangle.edit().has_uv=true;
    triangle.edit().ta={0,0,0};triangle.edit().tb={2,0,0};triangle.edit().tc={0,3,0};Hit th;
    check("triangle barycentric UV interpolation",triangle.hit({{.25,.25,1},{0,0,-1}},1e-8,th)&&std::abs(th.u-.5)<1e-12&&std::abs(th.v-.75)<1e-12);
    check("triangle texture tangent orientation",std::abs(dot(th.tangent,{1,0,0})-1)<1e-12);
    double dn=0;const int nz=800,np=1000;
    for(int i=0;i<nz;i++){double z=(i+.5)/nz,r=std::sqrt(1-z*z);for(int j=0;j<np;j++){double phi=2*pi*(j+.5)/np;dn+=ggx_D_aniso({r*std::cos(phi),r*std::sin(phi),z},.3,.65)*z*2*pi/(nz*np);}}
    check("anisotropic GGX projected-area normalization",std::abs(dn-1)<.0002);
    check("isotropic and anisotropic GGX limit",std::abs(ggx_D(.8,.4)-ggx_D_aniso({.6,0,.8},.4,.4))<1e-12);
    check("anisotropy rotates under axis exchange",std::abs(ggx_D_aniso({.3,.4,std::sqrt(.75)},.2,.6)-ggx_D_aniso({.4,.3,std::sqrt(.75)},.6,.2))<1e-12);
    Material metal;metal.type=MaterialType::Metal;metal.alpha_u=.12;metal.alpha_v=.45;metal.eta=Spectrum(.3);metal.k=Spectrum(3.2);
    Hit h;h.n=h.gn={0,0,1};h.tangent={1,0,0};Vec3 wo=normalize(Vec3(.3,.2,1));bool aniso_ok=true,pol_ok=true;
    for(int i=0;i<15000;i++){
        auto s=sample_bsdf(metal,h,wo,550,1,1,rng,false,true);if(s.pdf<=0)continue;
        auto e=evaluate_bsdf(metal,h,wo,s.wi,550,1,1,false);
        aniso_ok=aniso_ok&&std::abs(e.pdf-s.pdf)<1e-12&&std::isfinite(s.weight.v)&&s.weight.v>=0;
        pol_ok=pol_ok&&std::abs(e.f.v-e.pol.at(0,0))<1e-10;
        Stokes v=s.pol*Stokes{1,0,0,0};pol_ok=pol_ok&&v[0]>=-1e-12&&v[0]*v[0]+1e-10>=v[1]*v[1]+v[2]*v[2]+v[3]*v[3];
    }
    check("anisotropic GGX sampling/evaluation PDF agreement",aniso_ok);
    check("rough conductor Mueller intensity/physical Stokes",pol_ok);
    Material glass;glass.type=MaterialType::RoughGlass;glass.alpha_u=.18;glass.alpha_v=.32;
    bool roughglass_ok=true;for(int i=0;i<15000;i++){auto s=sample_bsdf(glass,h,wo,550,1,1.5,rng,false,true);if(s.pdf<=0)continue;auto e=evaluate_bsdf(glass,h,wo,s.wi,550,1,1.5,false);roughglass_ok=roughglass_ok&&std::abs(e.f.v-e.pol.at(0,0))<1e-9&&std::abs(e.pdf-s.pdf)<1e-10;}
    check("rough dielectric scalar/Mueller and PDF agreement",roughglass_ok);
    Volume volume;volume.phase=1;double phase_norm=0,phase_mean=0,phase_second=0;
    for(int i=0;i<N;i++){double c=2*(i+.5)/N-1;phase_norm+=rayleigh_phase(c)*4*pi/N;auto d=phase_sample(volume,{0,0,1},rng);phase_mean+=d.z;phase_second+=d.z*d.z;}
    check("Rayleigh phase normalization",std::abs(phase_norm-1)<1e-9);
    check("Rayleigh sampler first/second moments",std::abs(phase_mean/N)<.006&&std::abs(phase_second/N-.4)<.006);
    auto pm=phase_mueller(volume,{0,0,1},{1,0,0});Stokes ps=pm*Stokes{1,0,0,0};
    check("Rayleigh right-angle fully linear polarization",std::abs(std::sqrt(ps[1]*ps[1]+ps[2]*ps[2])/ps[0]-1)<1e-12);
    bool phase_scalar=true;for(int i=0;i<1000;i++){Vec3 wi=uniform_sphere(rng);auto m=phase_mueller(volume,wo,wi);phase_scalar=phase_scalar&&std::abs(m.at(0,0)-rayleigh_phase(-dot(wo,wi)))<1e-12;}
    check("Rayleigh Mueller M00 equals scalar phase",phase_scalar);
    Scene point;point.materials.emplace_back();DeltaEmitter light;light.position={0,0,3};light.intensity=Spectrum(10);point.delta_lights.push_back(light);point.build();
    auto near=point.sample_light({0,0,1},0,550,rng),far=point.sample_light({0,0,-1},0,550,rng);
    check("point emitter inverse-square radiance",near.delta&&far.delta&&near.pdf==1&&std::abs(near.radiance/far.radiance-4)<1e-12);
    point.delta_lights[0].kind=1;point.delta_lights[0].direction={0,0,-1};point.build();near=point.sample_light({0,0,1},0,550,rng);far=point.sample_light({0,0,-100},0,550,rng);
    check("directional emitter distance invariance",near.delta&&std::isinf(near.distance)&&near.radiance==far.radiance);
    Scene sheets;Material filter;filter.type=MaterialType::Null;filter.color=Spectrum(.6);sheets.materials.push_back(filter);Primitive quad;quad.shape=Shape::Quad;quad.edit().a={-2,-2,0};quad.edit().b={4,0,0};quad.edit().c={0,4,0};quad.material=0;sheets.primitives.push_back(quad);quad.edit().a.z=1;sheets.primitives.push_back(quad);sheets.build();
    Ray ray{{0,0,-1},{0,0,1}};auto shadow=sheets.shadow(ray,4,550,rng);
    check("transparent shadow multiplies attenuation",std::abs(shadow.value-.36)<1e-12&&shadow.probability==1);
    sheets.materials[0].type=MaterialType::Diffuse;sheets.build();check("opaque shadow still occludes",sheets.shadow(ray,4,550,rng).value==0);
    Material masked;masked.type=MaterialType::Mask;masked.opacity=.25;masked.child1=std::make_shared<Material>();auto tr=sheet_transmission(masked,h,{0,0,-1},550);
    check("mask shadow probability and throughput",std::abs(tr.value-.75)<1e-12&&std::abs(tr.probability-.75)<1e-12);
    Material thin;thin.type=MaterialType::ThinGlass;thin.ior_a=1.5;tr=sheet_transmission(thin,h,{0,0,-1},550);
    check("thin dielectric internal-reflection series",std::abs(tr.value-(1-.08/1.04))<1e-12);
    Material polar;polar.type=MaterialType::Polarizer;polar.axis={1,0,0};auto tr1=sheet_transmission(polar,h,{0,0,1},550);polar.axis={0,1,0};auto tr2=sheet_transmission(polar,h,{0,0,1},550);auto crossed=tr1.pol*tr2.pol;
    check("crossed polarizers on a shadow path",std::abs(crossed.at(0,0))<1e-12);
    Material nullm;nullm.type=MaterialType::Null;nullm.color=Spectrum(.7);auto ns=sample_bsdf(nullm,h,wo,550,1,1,rng,false,false);
    check("null BSDF transmits without a medium transition",ns.transmission&&!ns.medium_transition&&norm(ns.wi+wo)<1e-12);
    Material transmit;transmit.type=MaterialType::DiffuseTransmission;transmit.color=Spectrum(.7);auto ts=sample_bsdf(transmit,h,wo,550,1,1,rng,false,false);
    check("diffuse transmission crosses hemisphere",ts.pdf>0&&dot(ts.wi,h.n)<0&&std::abs(ts.weight.v-.7)<1e-12&&!ts.medium_transition);
    std::vector<PixelState> pixels(6);pixels[0].xyz={1,2,3};pixels[3].sy=4.5;auto path=std::filesystem::temp_directory_path()/"cybr-film-test.bin";
    save_checkpoint(path.string(),193,8,3,2,pixels);std::vector<PixelState> restored(6);auto count=load_checkpoint(path.string(),193,3,2,restored);
    check("checkpoint lossless state roundtrip",count==8&&std::memcmp(pixels.data(),restored.data(),pixels.size()*sizeof(PixelState))==0);
    bool mismatch=false;try{load_checkpoint(path.string(),194,3,2,restored);}catch(...){mismatch=true;}check("checkpoint rejects scene mismatch",mismatch);
    {std::fstream f(path,std::ios::in|std::ios::out|std::ios::binary);f.seekp(60);char c=17;f.write(&c,1);}bool corrupt=false;try{load_checkpoint(path.string(),193,3,2,restored);}catch(...){corrupt=true;}check("checkpoint rejects data corruption",corrupt);std::filesystem::remove(path);
    for(auto name:{"box","tent","gaussian","mitchell","lanczos"}){
        double normw=0,first=0;bool finite=true;for(int i=0;i<N;i++){auto f=sample_filter(name,rng);normw+=f.weight;first+=f.weight*f.offset;finite=finite&&std::isfinite(f.weight);}
        check(std::string(name)+" filter normalization/symmetry",finite&&std::abs(normw/N-1)<.016&&std::abs(first/N)<.014);
    }
    RNG a(100),b(100);a.configure(21,81,1);b.configure(21,81,1);bool same=true;for(int i=0;i<100;i++)same=same&&a.uniform()==b.uniform();check("Halton sample index determinism including fallback",same);
    double qmean=0;for(int i=0;i<1024;i++){RNG r(3);r.configure(i,93,1);qmean+=r.uniform();}check("random-shift Halton first-dimension uniformity",std::abs(qmean/1024-.5)<.0011);
    Primitive copied;copied.shape=Shape::Sphere;copied.edit().a={0,0,0};auto isolated=copied;isolated.edit().a.x=3;
    check("analytic primitives preserve copy-on-write value semantics",copied.geometry().a.x==0&&isolated.geometry().a.x==3);
    check("indexed primitive storage stays compact",sizeof(Primitive)<=64);
    Scene cache_scene;cache_scene.materials.emplace_back();cache_scene.primitives.push_back(copied);cache_scene.build();
    Ray cache_ray{{0,0,-3},{0,0,1}};Hit ca,cb;
    bool first=cache_scene.bvh.hit(cache_ray,ca),second=cache_scene.bvh.hit(cache_ray,cb);
    check("repeated ray preserves exact hit data",first&&second&&ca.t==cb.t&&ca.object==cb.object&&ca.primitive==cb.primitive);
    Hit clipped;clipped.t=1.5;
    check("ray cache respects maximum clipping",!cache_scene.bvh.hit(cache_ray,clipped)&&clipped.t==1.5);
    Hit skipped;
    check("ray cache respects minimum clipping",cache_scene.bvh.hit(cache_ray,skipped,2.1)&&skipped.t==4);
    cache_scene.primitives[0].edit().velocity={1,0,0};cache_scene.build();cache_ray.time=4;Hit moved;
    check("ray cache respects motion time",!cache_scene.bvh.hit(cache_ray,moved));
    cache_ray.time=0;Hit before;cache_scene.bvh.hit(cache_ray,before);
    cache_scene.primitives[0].edit().a={3,0,0};cache_scene.build();Hit rebuilt;
    check("BVH rebuild invalidates cached geometry",!cache_scene.bvh.hit(cache_ray,rebuilt));
    Scene empty;empty.environment.strength=.5;empty.build();Hit eh;
    check("environment-only scene without dummy materials",!empty.bvh.hit({{0,0,0},{0,0,1}},eh)&&empty.environment.active());
    auto throws=[](auto fn){try{fn();return false;}catch(const std::exception&){return true;}};
    Volume vg;vg.kind=2;vg.bounds.lo={0,0,0};vg.bounds.hi={1,1,1};vg.nx=vg.ny=vg.nz=2;vg.grid.assign(8,.5f);
    vg.validate();check("valid scalar density grid",std::abs(vg.density({.3,.4,.5})-.5)<1e-12);
    check("grid count prevents dimension product overflow",throws([]{Volume::grid_count(2147483647,2147483647,2147483647);}));
    auto badgrid=vg;badgrid.grid.clear();check("missing grid rejected before transport",throws([&]{badgrid.validate();}));
    badgrid=vg;badgrid.grid[0]=std::numeric_limits<float>::infinity();check("nonfinite grid rejected before transport",throws([&]{badgrid.validate();}));
    badgrid=vg;badgrid.density_majorant=.1;check("undersized density majorant rejected",throws([&]{badgrid.validate();}));
    badgrid=vg;badgrid.bounds.hi.x=0;check("degenerate volume bounds rejected",throws([&]{badgrid.validate();}));
    badgrid=vg;badgrid.grid.assign(8,0);badgrid.density_majorant=0;badgrid.validate();check("zero-density volume permits zero majorant",badgrid.transmittance({{.5,.5,-1},{0,0,1}},4,550,rng)==1);
    std::cout<<"TOTAL "<<check.passed<<" passed, "<<check.failed<<" failed\n";return check.failed?1:0;
}
