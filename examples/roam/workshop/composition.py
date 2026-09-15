"""Rigid placement of complete mechanisms, preserving their interface metadata."""
from kernel import *
def append(dst,src,prefix,offset=(0,0,0),rotation=('Z',0),inherited=()):
 axis,angle=rotation;a=np.array(AXES[axis],float);theta=math.radians(angle);K=np.array([[0,-a[2],a[1]],[a[2],0,-a[0]],[-a[1],a[0],0]]);R=np.eye(3)+math.sin(theta)*K+(1-math.cos(theta))*(K@K)
 def placed(q):return q.rotate((0,0,0),AXES[axis],angle).translate(offset)
 def tags(items):return [prefix+t for t in items]+list(inherited)
 def axis_after(ax):
  v=R@AXES[ax];i=int(np.argmax(np.abs(v)));assert abs(abs(v[i])-1)<1e-6;return 'XYZ'[i],float(v[i])
 for n,d in src.dofs.items():
  ax,sign=axis_after(d['axis']);dst.dofs[prefix+n]={**d,'axis':ax,'origin':list(R@d['origin']+offset),'factor':d['factor']*sign,'control':prefix+('feed' if n in ['feed','screw'] else n)}
 names=[]
 for n,v in src.parts.items():
  name=prefix+n;names.append(name);dst.parts[name]={**v,'shape':placed(v['shape']),'motion':tags(v['motion'])}
 for c in src.connections:dst.connections.append({**c,'a':prefix+c['a'],'b':prefix+c['b']})
 for t in src.threads:dst.threads.append({**t,'a':prefix+t['a'],'b':prefix+t['b'],'mask':placed(t['mask']),'motion':tags(t['motion'])})
 dst.sources.extend(s for s in src.sources if s not in dst.sources)
 return names

def fixture_plate(m,prefix,lower,upper,center_z,motion,xy=(20,16)):
 """Four real stand-offs around the captive feed, plus an accessible fixture grid."""
 plate=prefix+'Fixture_plate';q=box(150,150,12,(0,0,center_z+6),4)
 for x in [-50,0,50]:
  for y in [-50,0,50]:q=q.cut(cyl(3.3,14,(x,y,center_z-1)))
 m.add(plate,q,0,motion,'Replaceable printed fixture grid; no machining-load rating','Print flat')
 bottom=m.parts[lower]['shape'].BoundingBox().zmin;cap_top=m.parts[upper]['shape'].BoundingBox().zmax;standoffs=[];fasteners=[]
 for x in [-xy[0],xy[0]]:
  for y in [-xy[1],xy[1]]:
   post=prefix+f'P{x}_{y}';m.add(post,ring(7,2.2,center_z-cap_top,(x,y,cap_top)),1,motion,'Solid metal spacer around the fixture through-bolt','Cut spacer stock',upper,fit=0);standoffs.append(post)
   m.cut(upper,cyl(2.2,center_z+13-bottom,(x,y,bottom-.1)))
   m.cut(lower,cyl(4.1,4.8,(x,y,bottom)))
   name=prefix+f'F{x}_{y}';before=set(m.parts);m.bolt(name,lower,plate,(x,y,bottom+4.8),center_z+12-bottom-4.8);fasteners+=list(set(m.parts)-before)
   m.connect(post,plate,'Supporting face',0)
 return plate,standoffs,fasteners
