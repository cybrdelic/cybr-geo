import numpy as np
import pytest

from mechanism_lab.core import Assembly,Material,Part,View
from mechanism_lab.truth import assert_renderable,truth_report
from mechanism_lab.exporters import export_meshbin


def part(name='p',provenance='designed-concept'):
    v=np.array([[0.,0.,0.],[1.,0.,0.],[0.,1.,0.]])
    f=np.array([[0,1,2]],dtype=np.int64)
    n=np.array([[0.,0.,1.]]*3)
    return Part(name,v,f,n,provenance=provenance)


def assembly(*parts,material=None):
    return Assembly('truth_fixture',list(parts),[material or Material('plain',(0.4,0.4,0.4),.9,.25)],views={'hero':View()})


def test_clean_original_design_auto_labels_as_concept():
    a=assembly(part())
    report=assert_renderable(a,'auto')
    assert report['passed']
    assert report['resolved_intent']=='concept'
    assert report['tier_counts']=={'designed':1}


def test_preserved_reference_geometry_remains_authoritative():
    a=assembly(part('preserved','preserved-v3-geometry'))
    report=assert_renderable(a,'auto')
    assert report['passed']
    assert report['resolved_intent']=='reference'
    assert report['tier_counts']=={'authoritative':1}


def test_estimated_internal_is_blocked_by_auto_gate():
    a=assembly(part('known','manufacturer-cad'),part('guessed','inferred-internal'))
    with pytest.raises(ValueError,match='guessed'):
        assert_renderable(a,'auto')
    report=truth_report(a,'inspection')
    assert report['passed']
    assert report['tier_counts']['estimated']==1


def test_allow_estimates_is_explicit_but_unknown_still_blocks():
    a=assembly(part('estimate','photo-estimate'))
    assert assert_renderable(a,'reference',allow_estimates=True)['passed']
    b=assembly(part('mystery','legacy'))
    with pytest.raises(ValueError,match='mystery'):
        assert_renderable(b,'reference',allow_estimates=True)


def test_material_contract_carries_explicit_optical_fields():
    m=Material('anodized',(0.03,0.035,0.04),.75,.32,ior=1.48,coat=.08,coat_rough=.22,anisotropy=.15,microfinish='bead-blasted',material_source='design spec')
    d=m.as_dict()
    assert d['ior']==1.48
    assert d['coat']==.08
    assert d['microfinish']=='bead-blasted'
    assert d['material_source']=='design spec'


def test_meshbin_does_not_invent_machining_pattern_from_metallicity(tmp_path):
    a=assembly(part(),material=Material('polished metal',(0.5,0.5,0.5),1.0,.12,microfinish='none'))
    path=tmp_path/'scene.meshbin';export_meshbin(a,path)
    fields=path.with_suffix('.materials').read_text().split()
    assert fields[-1]=='0'


def test_meshbin_uses_only_explicit_microfinish(tmp_path):
    a=assembly(part(),material=Material('turned metal',(0.5,0.5,0.5),1.0,.12,microfinish='turned'))
    path=tmp_path/'scene.meshbin';export_meshbin(a,path)
    fields=path.with_suffix('.materials').read_text().split()
    assert fields[-1]=='1'


def test_hero_views_default_to_perspective():
    view=View()
    assert view.projection=='perspective'
    assert view.focal_length_mm>0
    assert view.sensor_width_mm>0
