#include "cybr/io.hpp"
#include <iostream>
#include <filesystem>
using namespace cybr;
int main(){
 int failed=0,passed=0;
 auto check=[&](const char*name,bool ok){std::cout<<(ok?"PASS ":"FAIL ")<<name<<'\n';ok?passed++:failed++;};
 auto close=[](Vec3 a,Vec3 b){return norm(a-b)<1e-11;};
 auto tex=[](Vec3 a,Vec3 b){auto t=std::make_shared<ImageTexture>();t->width=2;t->height=1;t->pixels={a,b};return t;};
 Hit h;h.n=h.gn={0,0,1};h.tangent={1,0,0};h.u=.25;h.v=.5;h.tint=Vec3(.7);
 Material base;base.color=Spectrum(.4);base.texture=tex(Vec3(.5),Vec3(.9));
 auto a=surface_guide(base,h);check("textured albedo guide includes vertex tint",close(a.albedo,Vec3(.14)));
 h.u=.75;auto b=surface_guide(base,h);check("albedo guide follows UV texel",close(b.albedo,Vec3(.252)));
 Material mapped;mapped.type=MaterialType::NormalMap;mapped.texture=tex({.8,.5,.9},{.5,.8,.9});mapped.child1=std::make_shared<Material>(base);
 h.u=.25;auto n=surface_guide(mapped,h);
 check("normal guide uses tangent-space normal map",close(n.normal,{.6,0,.8}));
 check("normal wrapper does not replace child albedo",close(n.albedo,a.albedo));
 Material wrapped;wrapped.type=MaterialType::TwoSided;wrapped.child1=std::make_shared<Material>(mapped);
 check("guide resolves multiple nested wrappers",close(surface_guide(wrapped,h).normal,n.normal)&&close(surface_guide(wrapped,h).albedo,n.albedo));
 Material blend;blend.type=MaterialType::Blend;blend.weight=.25;blend.child1=std::make_shared<Material>();blend.child2=std::make_shared<Material>();
 blend.child1->color=Spectrum(.3);blend.child2->color=Spectrum(.7);h.tint=Vec3(1);
 check("blend albedo guide matches mixture",close(surface_guide(blend,h).albedo,Vec3(.4)));
 Material checker;checker.color=Spectrum(.2);checker.second=Spectrum(.8);checker.checker=2;checker.uv_checker=true;h.v=.1;h.u=.1;
 auto c=surface_guide(checker,h);h.u=.6;
 check("checker albedo guides retain local markings",close(c.albedo,Vec3(.2))&&close(surface_guide(checker,h).albedo,Vec3(.8)));
 Material plastic;plastic.type=MaterialType::Plastic;plastic.roughness=.6;plastic.roughness_texture=tex(Vec3(.15),Vec3(.85));h.u=.25;
 check("roughness texel is absolute perceptual roughness",std::abs(plastic.roughness_at(h)-.15)<1e-12);
 Vec3 wo=normalize(Vec3(.2,.1,1)),wi=normalize(Vec3(-.12,.2,1));
 auto low=evaluate_bsdf(plastic,h,wo,wi,550,1,1.5,false);h.u=.75;auto high=evaluate_bsdf(plastic,h,wo,wi,550,1,1.5,false);
 check("roughness texture changes actual GGX BSDF",std::abs(low.f.v-high.f.v)>.001&&std::abs(low.pdf-high.pdf)>.01);
 RNG rng(7291);bool consistent=true;
 for(int i=0;i<5000;i++){h.u=i%2?.25:.75;auto s=sample_bsdf(plastic,h,wo,550,1,1.5,rng,false,false);if(s.pdf<=0)continue;auto e=evaluate_bsdf(plastic,h,wo,s.wi,550,1,1.5,false);consistent&=std::abs(s.pdf-e.pdf)<1e-12&&std::isfinite(s.weight.v);}
 check("textured roughness sampling and evaluation agree",consistent);
 auto dir=std::filesystem::temp_directory_path()/"cybr-surface-guide-test";std::filesystem::create_directories(dir);
 auto pfm=dir/"rough.pfm",cys=dir/"scene.cys";write_pfm(pfm.string(),{Vec3(.2),Vec3(.8)},2,1);
 {std::ofstream f(cys);f<<"material plastic 1 1 1 .5 1.5 0 .25 .6 1.2 3.5 2.8 2 0 0 0 0 6500\nroughness_texture 0 \"rough.pfm\" 1 1 1\n";}
 Scene scene;read_scene(cys.string(),scene);h.u=.25;
 check("scene parser resolves roughness texture asset",std::abs(scene.materials[0].roughness_at(h)-.2)<1e-7&&scene.dependencies.size()==1);
 write_pfm(pfm.string(),{Vec3(1.2),Vec3(.8)},2,1);bool rejected=false;try{Scene bad;read_scene(cys.string(),bad);}catch(const std::exception&){rejected=true;}
 check("scene rejects out-of-range roughness before tracing",rejected);std::filesystem::remove_all(dir);
 Primitive triangle;triangle.shape=Shape::Triangle;triangle.edit().a={0,0,0};triangle.edit().b={1,0,0};triangle.edit().c={0,1,0};triangle.edit().has_uv=true;
 triangle.edit().ta={0,0,0};triangle.edit().tb={1,0,0};triangle.edit().tc={0,-1,0};Hit front,back;
 triangle.hit({{.2,.2,1},{0,0,-1}},1e-8,front);triangle.hit({{.2,.2,-1},{0,0,1}},1e-8,back);
 Material mirrored;mirrored.type=MaterialType::NormalMap;mirrored.texture=tex({.5,.8,.9},{.5,.8,.9});mirrored.child1=std::make_shared<Material>();
 check("mirrored UV normal map follows increasing v",close(surface_guide(mirrored,front).normal,{0,-.6,.8}));
 check("backface preserves UV orientation and flips normal",close(surface_guide(mirrored,back).normal,{0,-.6,-.8}));
 Primitive sphere;sphere.shape=Shape::Sphere;sphere.edit().a={0,0,0};sphere.radius=1;Hit sh;sphere.hit({{2,0,0},{-1,0,0}},1e-8,sh);
 check("sphere tangent follows spherical u coordinate",close(sh.tangent,{0,0,1}));
 Hit tilted;tilted.n=tilted.gn={0,0,1};tilted.tangent={1,0,0};tilted.u=.25;tilted.v=.5;
 Material crease;crease.type=MaterialType::NormalMap;crease.texture=tex({.9,.5,.8},{.9,.5,.8});crease.child1=std::make_shared<Material>();
 Vec3 camera_direction={0,0,1},beneath=normalize(Vec3(1,0,-.2)),above=normalize(Vec3(1,0,.2));
 for(auto type:{MaterialType::Diffuse,MaterialType::Plastic,MaterialType::Metal}){
  crease.child1->type=type;auto invalid=evaluate_bsdf(crease,tilted,camera_direction,beneath,550,1,1.5,false);auto valid=evaluate_bsdf(crease,tilted,camera_direction,above,550,1,1.5,false);
  check(type==MaterialType::Diffuse?"mapped diffuse rejects light beneath geometric surface":type==MaterialType::Plastic?"mapped plastic rejects light beneath geometric surface":"mapped metal rejects light beneath geometric surface",invalid.f.v==0&&invalid.pdf==0&&valid.f.v>0&&valid.pdf>0);
 }
 crease.child1->type=MaterialType::DiffuseTransmission;
 Vec3 wrong_side=normalize(Vec3(-1,0,.2)),correct_side=normalize(Vec3(-1,0,-.2));
 auto wrong=evaluate_bsdf(crease,tilted,camera_direction,wrong_side,550,1,1.5,false),correct=evaluate_bsdf(crease,tilted,camera_direction,correct_side,550,1,1.5,false);
 check("mapped transmission respects geometric surface side",wrong.f.v==0&&wrong.pdf==0&&correct.f.v>0&&correct.pdf>0);
 std::cout<<"TOTAL "<<passed<<" passed, "<<failed<<" failed\n";return failed?1:0;
}
