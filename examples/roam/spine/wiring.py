"""Analytic swept routing volumes. They are routing studies, not flex-rated cables.

The vertical U-loop has constant path length. Other articulated spans expose their
computed route length so slack requirements are visible, not silently treated as
stretching cable. Endpoints reserve connector access; actual devices are unknown.
"""
import numpy as np
import cadquery as cq
from functools import lru_cache
RADIUS=3.5
def pt(M,p):return (M@np.r_[p,1])[:3]
def curves(points):
    p=[np.asarray(x,float) for x in points];out=[]
    for i in range(len(p)-1):
        a=p[i-1] if i else 2*p[i]-p[i+1];b=p[i+2] if i+2<len(p) else 2*p[i+1]-p[i]
        out.append([p[i],p[i]+(p[i+1]-a)/6,p[i+1]-(b-p[i])/6,p[i+1]])
    return out
def samples(cs,n=20):
    rows=[]
    for a,b,c,d in cs:
        for t in np.linspace(0,1,n,endpoint=False):rows.append((1-t)**3*a+3*(1-t)**2*t*b+3*(1-t)*t*t*c+t**3*d)
    rows.append(cs[-1][-1]);return np.asarray(rows)
def pipe(cs):
    return cached_pipe(tuple(np.asarray(cs).ravel()))
@lru_cache(maxsize=64)
def cached_pipe(key):
    cs=np.asarray(key).reshape(-1,4,3)
    edges=[cq.Edge.makeBezier([cq.Vector(*p) for p in c]) for c in cs]
    wire=cq.Wire.assembleEdges(edges);t=cs[0][1]-cs[0][0]
    plane=cq.Plane(origin=cq.Vector(*cs[0][0]),normal=cq.Vector(*t))
    profile=cq.Workplane(plane).circle(RADIUS).val()
    s=cq.Solid.sweep(profile,[],wire,makeSolid=True,isFrenet=False)
    if not s.isValid() or len(s.Solids())!=1:raise ValueError('Invalid cable sweep')
    return s
def routes(m,q,solid=True,sides=None):
    fs,_=m.poses(q);out={};data={}
    for side in sides or ['left','right','tray']:
        C=fs[side+'_carriage'];A=fs[side+'_arm1'];B=fs[side+'_arm2'];H=fs[side+'_head'];P=fs[side+'_pitch'];E=fs[side+'_roll']
        L1,L2=(260,240) if side=='tray' else (280,260)
        # Rigid-arm conduits and generous external service bows clear end tongues.
        points=[pt(C,[-130,110,-190]),pt(C,[-130,110,-140]),pt(C,[-130,155,-145]),pt(C,[-130,160,-240]),pt(A,[0,100,-240]),pt(A,[55,24,-240]),pt(A,[45,30,-55]),pt(A,[55,24,0]),pt(A,[L1-55,24,0]),pt(A,[L1-30,50,-55]),pt(A,[L1,65,-80]),pt(B,[30,55,-55]),pt(B,[45,24,-55]),pt(B,[45,24,-15]),pt(B,[50,24,0]),pt(B,[65,24,0]),pt(B,[L2-55,24,0]),pt(B,[L2-15,55,-70]),pt(H,[-90,25,-40]),pt(P,[-130,65,35]),pt(P,[-150,100,0])]
        points += [pt(E,[-95,-35,-45]),pt(E,[-95,-80,-45]),pt(E,[240,-73,45])] if side=='tray' else [pt(E,[-95,-110,25]),pt(E,[-245,-145,28])]
        name=side+'_articulated_harness_route';cs=curves(points);v=samples(cs)
        data[name]={'radius_mm':RADIUS,'bezier':[[p.tolist() for p in c] for c in cs],'length_mm':float(np.linalg.norm(np.diff(v,axis=0),axis=1).sum()),'role':'Nominal installation corridor; service slack and connector selection pending'}
        if solid:out[name]=pipe(cs)
        # Constant-length 1500 mm U-loop on the outside of the spine, R=30 mm.
        top=1460.;end=q[side][0]-190;r=30.;bottom=(top+end+(np.pi-2)*r-1500)/2;z=bottom+r
        # Convert fixed world height into the moving carriage frame.
        anchors=[[-130,50,top-q[side][0]],[-130,50,z-q[side][0]],[-130,80,bottom-q[side][0]],[-130,110,z-q[side][0]],[-130,110,-190]]
        world=[pt(C,p) for p in anchors];k=.5522847498307936
        # Two quarter-circle Beziers, vertical runs represented as straight cubics.
        def line(a,b):return [a,a+(b-a)/3,a+2*(b-a)/3,b]
        a,b,c,d,e=world;V=C[:3,1];Z=C[:3,2]
        cs=[line(a,b),[b,b-Z*(k*r),c-V*(k*r),c],[c,c+V*(k*r),d-Z*(k*r),d],line(d,e)]
        name=side+'_spine_service_loop';v=samples(cs)
        data[name]={'radius_mm':RADIUS,'bezier':[[p.tolist() for p in c] for c in cs],'length_mm':float(np.linalg.norm(np.diff(v,axis=0),axis=1).sum()),'minimum_nominal_bend_radius_mm':30,'role':'1500 mm routed U-loop; quarter-circle CAD approximation'}
        if solid:out[name]=pipe(cs)
    return out,data
