import numpy as np
import pytest

NAMES=['nitinol_compact','nitinol_high_force','nitinol_antagonistic','nitinol_rotary','nitinol_cartridge']

@pytest.mark.parametrize('name',NAMES)
def test_family_variant_builds_and_has_render_views(name):
    from mechanism_lab.registry import factory
    from mechanism_lab.core import validate
    build,motion=factory(name)
    a=build()
    assert a.name==name
    assert motion is not None
    assert len(a.parts)>40
    assert {'hero','opposite','side','top','end','bundle','electrical','section','exploded','hot'}<=set(a.views)
    assert validate(a)['parts']==len(a.parts)

@pytest.mark.parametrize('name',NAMES)
def test_family_variant_motion_stays_rigid(name):
    from mechanism_lab.registry import factory
    build,_=factory(name);a=build()
    for t in [0,.6,1.2,3.0,5.3]:
        for p in a.parts:
            T=a.pose(p,t,.35)
            assert np.isfinite(T).all()
            assert np.allclose(T[3],[0,0,0,1])
            assert np.allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-8)
            assert abs(np.linalg.det(T[:3,:3])-1)<1e-8

def test_family_is_mechanically_distinct():
    from mechanism_lab.registry import factory
    built={n:factory(n)[0]() for n in NAMES}
    assert built['nitinol_compact'].metadata['variant']['fiber_count']==8
    assert built['nitinol_high_force'].metadata['variant']['fiber_count']==24
    assert any(p.motion=='left_fiber' for p in built['nitinol_antagonistic'].parts)
    assert any(p.motion=='rotor' for p in built['nitinol_rotary'].parts)
    assert any(p.group=='enclosure' for p in built['nitinol_cartridge'].parts)
