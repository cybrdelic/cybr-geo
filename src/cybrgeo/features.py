"""Reusable mechanical construction primitives, native millimetres.
Involute flanks are parametric; generated root transitions are not cutter-certified.
"""
import math
import numpy as np

def involute_profile(teeth:int,module:float,backlash:float=.03,nflank:int=10):
    """Transverse involute profile with tooth-thickness allowance at pitch circle.
    Root arcs are visualization fillets, not cutter-envelope root certification.
    """
    rp=teeth*module/2; rb=rp*math.cos(math.radians(20)); rr=rp-1.25*module;ra=rp+module
    def inv(r):
        if r<=rb:return 0.
        q=math.sqrt((r/rb)**2-1);return q-math.atan(q)
    h=math.pi/(2*teeth)-backlash/(2*rp)
    ip=inv(rp); pts=[]
    for j in range(teeth):
        c=2*math.pi*j/teeth; hr=h+ip
        pts.append((rr,c-hr-.010))
        pts.append((rr+.10,c-hr-.002))
        for r in np.linspace(max(rb,rr+.10),ra,nflank):pts.append((r,c-(h+ip-inv(r))))
        ha=h+ip-inv(ra)
        for q in np.linspace(-ha,ha,6)[1:]:pts.append((ra,c+q))
        for r in np.linspace(ra,max(rb,rr+.10),nflank)[1:]:pts.append((r,c+h+ip-inv(r)))
        pts.extend([(rr+.10,c+hr+.002),(rr,c+hr+.010)])
        nxt=c+2*math.pi/teeth-hr-.010
        for q in np.linspace(c+hr+.010,nxt,5)[1:-1]:pts.append((rr,q))
    return np.asarray(pts,float)



def bolt_circle(count, radius, phase_degrees=0):
    if count < 1 or radius <= 0: raise ValueError("positive count and radius required")
    return [(radius*math.cos(math.radians(phase_degrees)+2*math.pi*i/count),
             radius*math.sin(math.radians(phase_degrees)+2*math.pi*i/count)) for i in range(count)]

def annulus(outer_diameter, inner_diameter, width, origin=(0,0,0), plane='YZ'):
    import cadquery as cq
    if not 0 <= inner_diameter < outer_diameter or width <= 0:raise ValueError('invalid annulus dimensions')
    w=cq.Workplane(plane,origin=origin).circle(outer_diameter/2)
    if inner_diameter:w=w.circle(inner_diameter/2)
    return w.extrude(width).val()

def involute_spur_gear(teeth,module,width,bore=0,origin=(0,0,0),phase=0,backlash=.10):
    import cadquery as cq
    if teeth<8 or min(module,width)<=0:raise ValueError('invalid gear parameters')
    p=involute_profile(teeth,module,backlash,nflank=12)
    xy=np.c_[p[:,0]*np.cos(p[:,1]+phase),p[:,0]*np.sin(p[:,1]+phase)]
    result=cq.Workplane('YZ',origin=origin).polyline(xy.tolist()).close().extrude(width)
    if bore:
        o=(origin[0]-1,origin[1],origin[2])
        result=result.cut(cq.Workplane('YZ',origin=o).circle(bore/2).extrude(width+2))
    return result
