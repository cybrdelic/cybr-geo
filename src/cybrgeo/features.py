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
    # For high tooth counts the root lies ABOVE the base circle.  Extending
    # the base-circle tooth width down to that root, plus a fixed .010 rad
    # transition on each side, made adjacent root loops cross (e.g. 72T/m1).
    # Start at the actual involute radius and keep each transition inside its
    # own tooth sector.  These short root blends remain visualization geometry.
    start_radius=max(rb,rr+.10)
    flank_half=h+ip-inv(start_radius)
    root_half=min(flank_half+.008,math.pi/teeth*.94)
    for j in range(teeth):
        c=2*math.pi*j/teeth
        pts.append((rr,c-root_half))
        pts.append((rr+.06,c-root_half+.0015))
        for r in np.linspace(start_radius,ra,nflank):pts.append((r,c-(h+ip-inv(r))))
        ha=h+ip-inv(ra)
        for q in np.linspace(-ha,ha,6)[1:]:pts.append((ra,c+q))
        for r in np.linspace(ra,start_radius,nflank)[1:]:pts.append((r,c+h+ip-inv(r)))
        pts.extend([(rr+.06,c+root_half-.0015),(rr,c+root_half)])
        nxt=c+2*math.pi/teeth-root_half
        for q in np.linspace(c+root_half,nxt,5)[1:-1]:pts.append((rr,q))
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
