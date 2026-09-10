from functools import lru_cache
import numpy as np
import pytest

NAMES=['nitinol_compact','nitinol_high_force','nitinol_antagonistic','nitinol_rotary','nitinol_cartridge']

@lru_cache(maxsize=None)
def built(name):
    from mechanism_lab.registry import factory
    return factory(name)[0]()

@pytest.mark.parametrize('name',NAMES)
def test_family_variant_builds_and_has_render_views(name):
    from mechanism_lab.registry import factory
    from mechanism_lab.core import validate
    a=built(name);_,motion=factory(name)
    assert a.name==name
    assert motion is not None
    assert len(a.parts)>40
    assert {'hero','opposite','side','top','end','bundle','electrical','section','exploded','hot'}<=set(a.views)
    assert validate(a)['parts']==len(a.parts)

@pytest.mark.parametrize('name',NAMES)
def test_family_variant_motion_stays_rigid(name):
    a=built(name)
    for t in [0,.6,1.2,3.0,5.3]:
        for p in a.parts:
            T=a.pose(p,t,.35)
            assert np.isfinite(T).all()
            assert np.allclose(T[3],[0,0,0,1])
            assert np.allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-8)
            assert abs(np.linalg.det(T[:3,:3])-1)<1e-8

def test_family_is_mechanically_distinct():
    family={n:built(n) for n in NAMES}
    assert family['nitinol_compact'].metadata['variant']['fiber_count']==8
    assert family['nitinol_high_force'].metadata['variant']['fiber_count']==24
    assert any(p.motion=='left_fiber' for p in family['nitinol_antagonistic'].parts)
    assert any(p.motion=='rotor' for p in family['nitinol_rotary'].parts)
    assert any(p.group=='enclosure' for p in family['nitinol_cartridge'].parts)
