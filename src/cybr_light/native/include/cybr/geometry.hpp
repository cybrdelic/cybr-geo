#pragma once
#include "math.hpp"
#include <vector>
#include <numeric>
#include <functional>
#include <memory>
#include <atomic>
#include <cstring>
namespace cybr {
struct Hit { double t=inf;Vec3 p,n,gn,tangent,tint{1};double u=0,v=0;int primitive=-1,material=-1,object=-1;bool front=true; };
enum class Shape {Sphere, Triangle, Quad, Disk, Cylinder};
struct MeshletVertex {float value[11];};
struct Meshlet {std::vector<MeshletVertex> vertices;};
struct PrimitiveData {
 Vec3 ca{1},cb{1},cc{1},a,b,c,na,nb,nc,velocity,ta,tb,tc;
 bool has_uv=false,smooth=false;
};
struct Primitive {
 Shape shape=Shape::Sphere;double radius=1.;int material=0,object=0;
 // Analytic primitives retain their full authoring data. Meshlet triangles
 // share a vertex block and carry only three small indices and material IDs.
 std::shared_ptr<PrimitiveData> data;
 const Meshlet* meshlet=nullptr;std::array<uint8_t,3> indices{};
 PrimitiveData& edit(){if(!data)data=std::make_shared<PrimitiveData>();else if(!data.unique())data=std::make_shared<PrimitiveData>(*data);return *data;}
 PrimitiveData geometry()const{
  if(data)return *data;
  PrimitiveData g;if(!meshlet)return g;
  auto a=meshlet->vertices[indices[0]].value,b=meshlet->vertices[indices[1]].value,c=meshlet->vertices[indices[2]].value;
  g.a={a[0],a[1],a[2]};g.b={b[0],b[1],b[2]};g.c={c[0],c[1],c[2]};
  g.na={a[3],a[4],a[5]};g.nb={b[3],b[4],b[5]};g.nc={c[3],c[4],c[5]};
  g.smooth=norm2(g.na)>0&&norm2(g.nb)>0&&norm2(g.nc)>0;
  g.ta={a[6],a[7],0};g.tb={b[6],b[7],0};g.tc={c[6],c[7],0};g.has_uv=true;
  g.ca={a[8],a[9],a[10]};g.cb={b[8],b[9],b[10]};g.cc={c[8],c[9],c[10]};return g;
 }
 Bounds bounds(double time_min=0,double time_max=1)const{
  const auto g=geometry();const auto &a=g.a,&b=g.b,&c=g.c,&na=g.na,&nb=g.nb,&nc=g.nc,&velocity=g.velocity,&ta=g.ta,&tb=g.tb,&tc=g.tc,&ca=g.ca,&cb=g.cb,&cc=g.cc;bool has_uv=g.has_uv,smooth=g.smooth;
  Bounds r;if(shape==Shape::Sphere){r.grow(a-Vec3(radius));r.grow(a+Vec3(radius));}
  else if(shape==Shape::Cylinder){r.grow(vmin(a,b)-Vec3(radius));r.grow(vmax(a,b)+Vec3(radius));}
  else if(shape==Shape::Disk){r.grow(a-Vec3(radius));r.grow(a+Vec3(radius));}
  else if(shape==Shape::Triangle){r.grow(a);r.grow(b);r.grow(c);}
  else{r.grow(a);r.grow(a+b);r.grow(a+c);r.grow(a+b+c);}
  Bounds m;m.grow(r.lo+velocity*time_min);m.grow(r.hi+velocity*time_min);m.grow(r.lo+velocity*time_max);m.grow(r.hi+velocity*time_max);m.lo=m.lo-Vec3(1e-7);m.hi=m.hi+Vec3(1e-7);return m;
 }
 double area()const{const auto g=geometry();const auto &a=g.a,&b=g.b,&c=g.c,&na=g.na,&nb=g.nb,&nc=g.nc,&velocity=g.velocity,&ta=g.ta,&tb=g.tb,&tc=g.tc,&ca=g.ca,&cb=g.cb,&cc=g.cc;bool has_uv=g.has_uv,smooth=g.smooth;if(shape==Shape::Cylinder)return 2*pi*radius*norm(b-a);if(shape==Shape::Disk)return pi*radius*radius;return shape==Shape::Sphere?4*pi*radius*radius:shape==Shape::Triangle?.5*norm(cross(b-a,c-a)):norm(cross(b,c));}
 bool hit(const Ray&r,double min_t,Hit& h)const{
  const auto g=geometry();const auto &a=g.a,&b=g.b,&c=g.c,&na=g.na,&nb=g.nb,&nc=g.nc,&velocity=g.velocity,&ta=g.ta,&tb=g.tb,&tc=g.tc,&ca=g.ca,&cb=g.cb,&cc=g.cc;bool has_uv=g.has_uv,smooth=g.smooth;
  Vec3 origin=r.o-velocity*r.time;double t=inf,u=0,v=0;Vec3 n,gn,tangent,tint{1};
  if(shape==Shape::Sphere){Vec3 oc=origin-a;double bq=dot(oc,r.d),cq=dot(oc,oc)-radius*radius,disc=bq*bq-cq;if(disc<0)return false;double sq=std::sqrt(disc);t=-bq-sq;if(t<=min_t)t=-bq+sq;if(t<=min_t||t>=h.t)return false;gn=n=(origin+r.d*t-a)/radius;u=.5+std::atan2(n.z,n.x)/(2*pi);v=std::acos(clamp(n.y,-1,1))/pi;}
  else if(shape==Shape::Disk){
   gn=n=normalize(b);double den=dot(r.d,n);if(std::abs(den)<1e-14)return false;t=dot(a-origin,n)/den;if(t<=min_t||t>=h.t)return false;
   Vec3 q=origin+r.d*t-a;if(norm2(q)>radius*radius)return false;Frame f(n);u=.5+dot(q,f.x)/(2*radius);v=.5+dot(q,f.y)/(2*radius);tangent=f.x;
  }else if(shape==Shape::Cylinder){
   Vec3 axis=normalize(b-a),oc=origin-a;double length=norm(b-a),od=dot(oc,axis),dd=dot(r.d,axis);
   Vec3 op=oc-axis*od,dp=r.d-axis*dd;double A=norm2(dp),B=dot(op,dp),C=norm2(op)-radius*radius,disc=B*B-A*C;
   if(A<1e-20||disc<0)return false;double root=std::sqrt(disc);bool found=false;
   for(double candidate:{(-B-root)/A,(-B+root)/A}){double z=od+candidate*dd;if(candidate>min_t&&candidate<h.t&&z>=0&&z<=length){t=candidate;gn=n=normalize(op+dp*t);Frame f(axis);u=.5+std::atan2(dot(n,f.y),dot(n,f.x))/(2*pi);v=z/length;tangent=normalize(cross(axis,n));found=true;break;}}
   if(!found)return false;
  }
  else {
   Vec3 e1=shape==Shape::Triangle?b-a:b,e2=shape==Shape::Triangle?c-a:c;
   Vec3 p=cross(r.d,e2);double det=dot(e1,p);if(std::abs(det)<1e-14)return false;double inv=1/det;Vec3 q=origin-a;u=dot(q,p)*inv;if(u<0||u>1)return false;Vec3 k=cross(q,e1);v=dot(r.d,k)*inv;if(v<0||v>(shape==Shape::Triangle?1-u:1))return false;t=dot(e2,k)*inv;if(t<=min_t||t>=h.t)return false;gn=normalize(cross(e1,e2));n=shape==Shape::Triangle&&smooth?normalize(na*(1-u-v)+nb*u+nc*v):gn;if(dot(n,gn)<0)n=-n;tangent=normalize(e1);
   if(shape==Shape::Triangle)tint=ca*(1-u-v)+cb*u+cc*v;
   if(shape==Shape::Triangle&&has_uv){Vec3 uv=ta*(1-u-v)+tb*u+tc*v;Vec3 d1=tb-ta,d2=tc-ta;double detuv=d1.x*d2.y-d1.y*d2.x;if(std::abs(detuv)>1e-14)tangent=normalize((e1*d2.y-e2*d1.y)/detuv);u=uv.x;v=uv.y;}
  }
  h.t=t;h.p=r.at(t);h.tint=tint;h.tangent=norm2(tangent)>0?tangent:Frame(n).x;h.front=dot(r.d,gn)<0;h.gn=h.front?gn:-gn;h.n=h.front?n:-n;if(dot(h.n,r.d)>0)h.n=h.gn;h.material=material;h.object=object;h.u=u;h.v=v;return true;
 }
 void sample(RNG&r,double time,Vec3&p,Vec3&n)const{
  const auto g=geometry();const auto &a=g.a,&b=g.b,&c=g.c,&na=g.na,&nb=g.nb,&nc=g.nc,&velocity=g.velocity,&ta=g.ta,&tb=g.tb,&tc=g.tc,&ca=g.ca,&cb=g.cb,&cc=g.cc;bool has_uv=g.has_uv,smooth=g.smooth;
  if(shape==Shape::Sphere){n=uniform_sphere(r);p=a+n*radius;}
  else if(shape==Shape::Disk){n=normalize(b);auto d=concentric_disk(r.uniform(),r.uniform());Frame f(n);p=a+(f.x*d[0]+f.y*d[1])*radius;}
  else if(shape==Shape::Cylinder){Vec3 axis=normalize(b-a);Frame f(axis);double angle=2*pi*r.uniform();n=f.x*std::cos(angle)+f.y*std::sin(angle);p=a+(b-a)*r.uniform()+n*radius;}
  else if(shape==Shape::Triangle){double s=std::sqrt(r.uniform()),t=r.uniform();p=a*(1-s)+b*(s*(1-t))+c*(s*t);n=normalize(cross(b-a,c-a));}
  else {p=a+b*r.uniform()+c*r.uniform();n=normalize(cross(b,c));}p+=velocity*time;
 }
};
struct BVHNode {Bounds bounds;int left=-1,right=-1,start=0,count=0;};
// Exact, thread-local reuse across spectral wavelengths. No quantization:
// distinct rays or clipping intervals never share a cached intersection.
struct RayHitCache {
 uint64_t generation=0;Ray ray;double minimum=0,maximum=0;Hit hit;bool found=false;
 bool matches(uint64_t id,const Ray&r,double lo,double hi)const{
  return generation==id&&minimum==lo&&maximum==hi&&ray.time==r.time&&
   ray.o.x==r.o.x&&ray.o.y==r.o.y&&ray.o.z==r.o.z&&ray.d.x==r.d.x&&ray.d.y==r.d.y&&ray.d.z==r.d.z;
 }
};
inline uint64_t ray_hash(const Ray&r){
 auto bits=[](double x){uint64_t value;std::memcpy(&value,&x,8);return value;};
 return mix64(bits(r.o.x)^mix64(bits(r.o.y))^mix64(bits(r.o.z))^bits(r.d.x)^mix64(bits(r.d.y))^mix64(bits(r.d.z)));
}
inline uint64_t next_bvh_generation(){static std::atomic<uint64_t> counter{0};return ++counter;}
class BVH {
 uint64_t generation=0;
 const std::vector<Primitive>* primitives=nullptr;
 std::vector<int> order;std::vector<BVHNode> nodes;
 double time_min=0,time_max=1;
 int build_node(int begin,int end){
  int idx=int(nodes.size());nodes.emplace_back();Bounds all,centers;for(int i=begin;i<end;i++){Bounds b=(*primitives)[order[i]].bounds(time_min,time_max);all.grow(b);centers.grow(b.center());}nodes[idx].bounds=all;int n=end-begin;
  if(n<=4){nodes[idx].start=begin;nodes[idx].count=n;return idx;}
  constexpr int bins=12;double cost=inf;int bestaxis=-1,bestsplit=-1;
  for(int ax=0;ax<3;ax++){
   double extent=centers.hi[ax]-centers.lo[ax];if(extent<1e-12)continue;
   std::array<Bounds,bins> box;std::array<int,bins> count{};
   for(int i=begin;i<end;i++){auto b=(*primitives)[order[i]].bounds(time_min,time_max);int bi=std::min(bins-1,int((b.center()[ax]-centers.lo[ax])*bins/extent));box[bi].grow(b);count[bi]++;}
   for(int cut=0;cut<bins-1;cut++){Bounds l,r;int nl=0,nr=0;for(int j=0;j<=cut;j++)if(count[j]){l.grow(box[j]);nl+=count[j];}for(int j=cut+1;j<bins;j++)if(count[j]){r.grow(box[j]);nr+=count[j];}if(nl&&nr){double c=l.area()*nl+r.area()*nr;if(c<cost){cost=c;bestaxis=ax;bestsplit=cut;}}}
  }
  int mid=(begin+end)/2;
  if(bestaxis>=0){double extent=centers.hi[bestaxis]-centers.lo[bestaxis];auto pivot=std::partition(order.begin()+begin,order.begin()+end,[&](int id){int bin=std::min(bins-1,int(((*primitives)[id].bounds(time_min,time_max).center()[bestaxis]-centers.lo[bestaxis])*bins/extent));return bin<=bestsplit;});mid=int(pivot-order.begin());}
  if(mid==begin||mid==end||bestaxis<0){mid=(begin+end)/2;Vec3 e=centers.hi-centers.lo;int ax=e.x>e.y?(e.x>e.z?0:2):(e.y>e.z?1:2);std::nth_element(order.begin()+begin,order.begin()+mid,order.begin()+end,[&](int a,int b){return (*primitives)[a].bounds(time_min,time_max).center()[ax]<(*primitives)[b].bounds(time_min,time_max).center()[ax];});}
  int left=build_node(begin,mid),right=build_node(mid,end);nodes[idx].left=left;nodes[idx].right=right;return idx;
 }
 public:
 void build(const std::vector<Primitive>& p,double open=0,double close=1){generation=next_bvh_generation();if(!std::isfinite(open)||!std::isfinite(close))throw std::runtime_error("Nonfinite BVH shutter interval");time_min=std::min(open,close);time_max=std::max(open,close);primitives=&p;order.resize(p.size());std::iota(order.begin(),order.end(),0);nodes.clear();nodes.reserve(p.size()*2);if(!p.empty())build_node(0,int(p.size()));}
 size_t node_count()const{return nodes.size();}
 bool hit(const Ray&r,Hit& h,double min_t=1e-6)const{
  if(nodes.empty())return false;
#ifndef CYBR_DISABLE_RAY_CACHE
  static thread_local std::array<RayHitCache,128> ray_cache;
  auto&cached=ray_cache[ray_hash(r)%ray_cache.size()];double maximum=h.t;
  if(cached.matches(generation,r,min_t,maximum)){if(cached.found)h=cached.hit;return cached.found;}
#endif
  int stack[128],top=0;stack[top++]=0;bool found=false;
  while(top){const auto&node=nodes[stack[--top]];if(!node.bounds.hit(r,h.t))continue;if(node.count){for(int i=0;i<node.count;i++){int id=order[node.start+i];if((*primitives)[id].hit(r,min_t,h)){h.primitive=id;found=true;}}}else{if(top+2>=128)throw std::runtime_error("BVH traversal stack exhausted");const auto &l=nodes[node.left],&rr=nodes[node.right];double a,b,c,d;bool hl=l.bounds.interval(r,h.t,a,b),hr=rr.bounds.interval(r,h.t,c,d);if(hl&&hr){if(a<c){stack[top++]=node.right;stack[top++]=node.left;}else{stack[top++]=node.left;stack[top++]=node.right;}}else if(hl)stack[top++]=node.left;else if(hr)stack[top++]=node.right;}}
#ifndef CYBR_DISABLE_RAY_CACHE
  cached.generation=generation;cached.ray=r;cached.minimum=min_t;cached.maximum=maximum;cached.found=found;if(found)cached.hit=h;
#endif
  return found;
 }
 bool occluded(const Ray&r,double distance)const{
  if(nodes.empty())return false;
  int stack[128],top=0;stack[top++]=0;Hit h;h.t=distance;
  while(top){const auto&node=nodes[stack[--top]];if(!node.bounds.hit(r,distance))continue;
   if(node.count){for(int i=0;i<node.count;i++)if((*primitives)[order[node.start+i]].hit(r,1e-6,h))return true;}
   else{if(top+2>=128)throw std::runtime_error("BVH shadow stack exhausted");stack[top++]=node.left;stack[top++]=node.right;}
  }
  return false;
 }
};
struct Camera {
 Vec3 origin{0,2,6},target{0,1,0},up{0,1,0};double fov=40,aperture=0,focus=6,shutter_open=0,shutter_close=0;
 bool orthographic=false,spherical=false;double ortho_scale=4;
 Ray generate(double x,double y,int width,int height,RNG&r)const{
  Vec3 f=normalize(target-origin),right=normalize(cross(f,up)),top=cross(right,f);double aspect=double(width)/height;
  double px=(2*x/width-1)*aspect,py=1-2*y/height;
  Ray ray;ray.time=shutter_open+(shutter_close-shutter_open)*r.uniform();
  if(spherical){double a=(x/width-.5)*2*pi,t=y/height*pi;ray.o=origin;ray.d=normalize(f*(std::cos(a)*std::sin(t))+right*(std::sin(a)*std::sin(t))+top*std::cos(t));return ray;}
  if(orthographic){ray.o=origin+right*(px*ortho_scale*.5)+top*(py*ortho_scale*.5);ray.d=f;return ray;}
  double s=std::tan(fov*pi/360);ray.o=origin;ray.d=normalize(f+right*(px*s)+top*(py*s));
  if(aperture>0){Vec3 p=origin+ray.d*(focus/dot(ray.d,f));auto disk=concentric_disk(r.uniform(),r.uniform());ray.o+=right*(disk[0]*aperture)+top*(disk[1]*aperture);ray.d=normalize(p-ray.o);}return ray;
 }
};
}

