"""Numerical checks independent of the recipe's rendered appearance."""
from pathlib import Path
import sys
import math
import pytest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"examples"/"vr_headset"))
from spec import Spec
from optics import trace_from_eye,refract
from mechanism_lab.core import Material
from mechanism_lab.v9 import principled

def test_thick_lens_prescription():
    s=Spec();n=s.lens_ior;R=s.lens_radius;t=s.lens_thickness
    inverse=(n-1)*(2/R-(n-1)*t/(n*R*R))
    assert 1/inverse==pytest.approx(45,abs=1e-10)
    assert s.lens_edge_thickness>1
    assert .5<s.report()["paraxial_virtual_image_distance_mm"]/1000<2

def test_normal_incidence_and_reversibility():
    s=Spec()
    ray=trace_from_eye(s,0)
    assert ray["screen_x_mm"]==pytest.approx(0,abs=1e-12)
    theta=math.radians(25)
    d=np.array([math.sin(theta),math.cos(theta),0.])
    normal=np.array([0.,-1.,0.])
    transmitted=refract(d,normal,1,1.49)
    assert 1.49*transmitted[0]==pytest.approx(d[0],abs=1e-12)
    reverse=refract(-transmitted,-normal,1.49,1)
    assert np.allclose(reverse,-d,atol=1e-12)

def test_total_internal_reflection():
    a=math.radians(60)
    assert refract(np.array([math.sin(a),math.cos(a),0]),
                   np.array([0,-1,0]),1.49,1) is None

def test_symmetric_monotonic_ray_mapping():
    s=Spec()
    points=[]
    for angle in np.linspace(0,25,51):
        a=trace_from_eye(s,float(angle))
        b=trace_from_eye(s,-float(angle))
        assert a is not None and b is not None
        assert a["screen_x_mm"]==pytest.approx(-b["screen_x_mm"],abs=1e-8)
        points.append(a["screen_x_mm"])
    assert np.all(np.diff(points)>0)
    assert trace_from_eye(s,80) is None

@pytest.mark.parametrize("kwargs",[{"ipd":50},{"focus_offset":3},{"lens_ior":1},
                                   {"lens_diameter":150},{"phone_width":float("nan")}])
def test_invalid_design_inputs_rejected(kwargs):
    with pytest.raises(ValueError):
        Spec(**kwargs)

def test_v9_transmission_is_explicit_and_backward_compatible():
    opaque=Material("clear sounding name",(.5,.5,.5))
    assert principled(opaque,"test")["type"]=="principled"
    clear=Material("optical PMMA",(.99,.99,.99),rough=.002,ior=1.49,transmission=1)
    bsdf=principled(clear,"test")
    assert bsdf=={"type":"dielectric","int_ior":1.49,"ext_ior":1.0}
    restored=Material(**clear.as_dict())
    assert restored.transmission==1
    assert principled(restored,"test")==bsdf
    assert np.allclose(restored.color,clear.color)
    with pytest.raises(ValueError):
        principled(Material("bad",(.5,.5,.5),transmission=2),"test")

def test_mitsuba_accepts_refractive_materials():
    import mitsuba as mi
    mi.set_variant("llvm_ad_rgb")
    for rough,transmission in [(.002,1),(.1,1),(.1,.5)]:
        bsdf=mi.load_dict(principled(Material("lens",(.9,.9,.9),
                          rough=rough,transmission=transmission),"check"))
        assert bsdf is not None
