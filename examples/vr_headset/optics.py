"""Exact geometric Snell ray checks for the authored lens (not human testing)."""
import math
import numpy as np
from spec import Spec


def unit(v):
    v=np.asarray(v,float)
    return v/np.linalg.norm(v)


def sphere_hit(origin,direction,center,radius):
    oc=origin-center
    b=np.dot(oc,direction)
    disc=b*b-np.dot(oc,oc)+radius*radius
    if disc < 0:
        return None
    roots=sorted([-b-math.sqrt(disc),-b+math.sqrt(disc)])
    for t in roots:
        if t > 1e-6:
            return origin+t*direction
    return None


def refract(direction,normal,n1,n2):
    # Normal must face the incident medium.
    if np.dot(direction,normal)>0:
        normal=-normal
    eta=n1/n2
    cosi=-np.dot(normal,direction)
    k=1-eta*eta*(1-cosi*cosi)
    if k < 0:
        return None
    return unit(eta*direction+(eta*cosi-math.sqrt(k))*normal)


def trace_from_eye(s,angle_degrees=0.,pupil_x=0.):
    """Trace pupil -> eye surface -> screen surface -> phone plane.

    A reverse optical path defines which screen pixel corresponds to a viewing
    direction. Aperture rejection and total internal reflection are failures,
    never silently replaced by a pinhole ray.
    """
    R=s.lens_radius
    c=R-s.lens_thickness/2
    y=s.focus_offset
    origin=np.array([pupil_x,y-s.lens_thickness/2-s.eye_relief,0.])
    direction=np.array([math.sin(math.radians(angle_degrees)),
                        math.cos(math.radians(angle_degrees)),0.])
    eye_center=np.array([0.,y+c,0.])
    screen_center=np.array([0.,y-c,0.])
    p=sphere_hit(origin,direction,eye_center,R)
    if p is None or math.hypot(p[0],p[2])>s.lens_diameter/2:
        return None
    direction=refract(direction,unit(p-eye_center),1.,s.lens_ior)
    q=sphere_hit(p+direction*1e-5,direction,screen_center,R)
    if q is None or math.hypot(q[0],q[2])>s.lens_diameter/2:
        return None
    direction=refract(direction,unit(q-screen_center),s.lens_ior,1.)
    if direction is None or direction[1]<=0:
        return None
    screen=q+direction*((s.screen_y-q[1])/direction[1])
    return {"eye_surface":p.tolist(),"screen_surface":q.tolist(),
            "screen_x_mm":float(screen[0]),"exit_direction":direction.tolist()}


def optical_report(s):
    records=[]
    for angle in np.linspace(-45,45,181):
        ray=trace_from_eye(s,float(angle))
        # Binocular septum is x=0; each eye uses its own display half.
        # The nose-side bound can limit the angle before the outer screen edge.
        left_x=-s.ipd/2+(ray["screen_x_mm"] if ray else 0)
        visible=bool(ray and -s.screen_width/2 <= left_x <= -.7)
        records.append({"angle_deg":float(angle),"left_eye_visible":visible,"ray":ray})
    visible=[r["angle_deg"] for r in records if r["left_eye_visible"]]
    branch=[]
    previous=-1.
    for angle in np.linspace(0,45,601):
        ray=trace_from_eye(s,float(angle))
        if ray is None or ray["screen_x_mm"]<=previous:
            break
        previous=ray["screen_x_mm"]
        branch.append(float(angle))
    return {"monotonic_inverse_map_half_angle_deg":max(branch),
            "monotonic_inverse_map_screen_radius_mm":previous,
            "model":"two spherical refracting interfaces, authored IOR, nominal pupil",
            "paraxial":s.report(),"left_eye_horizontal_visible_degrees":
            [min(visible),max(visible)] if visible else None,
            "sampling_step_degrees":.5,
            "limits":["monochromatic geometric optics","no chromatic dispersion",
                      "no eye box / MTF qualification","nominal lens, not measured vendor optic",
                      "browser demo uses this prescription; measured lenses need calibration"],
            "rays":records}
