import pytest

from mechanism_lab.core import validate
from mechanism_lab.models import morphic_wrist


@pytest.fixture(scope='module')
def assembly():
    return morphic_wrist.build()


def test_builds_and_validates(assembly):
    report = validate(assembly)
    assert report['model'] == 'morphic_wrist'


def test_is_complex_by_geometry_not_repeated_gears(assembly):
    # The benchmark is intentionally a mixed analytic assembly with dozens of
    # distinct construction operations, not a tooth-count complexity proxy.
    assert len(assembly.parts) >= 90
    analytic = [p for p in assembly.parts if p.cad is not None]
    routed_mesh = [p for p in assembly.parts if p.cad is None]
    assert len(analytic) >= 55
    assert len(routed_mesh) >= 32
    assert not any('gear' in p.name.lower() for p in assembly.parts)


def test_required_modern_techniques_are_declared_and_present(assembly):
    expected = {
        'technique:loft',
        'technique:hollow-shell',
        'technique:boolean-csg',
        'technique:pattern',
        'technique:conformal-channel',
        'technique:3d-spline-sweep',
        'technique:brep-lattice',
        'technique:additive-manufacturing',
        'technique:freeform-loft',
        'technique:drafted-extrusion',
        'technique:true-helix',
        'technique:brep-sweep',
        'technique:revolve',
        'technique:multi-material-overmold',
        'technique:mixed-mesh-routing',
    }
    tags = {tag for p in assembly.parts for tag in p.tags}
    assert expected <= tags
    assert len(assembly.metadata['geometry_techniques']) >= 16


def test_showcase_parts_remain_real_analytic_brep(assembly):
    names = {
        'MW_01_Freeform_exoskeleton',
        'MW_02_Conformal_channel_liner_01',
        'MW_03_BCC_lattice_core',
        'MW_04_Organic_load_spine',
        'MW_06_Drafted_service_collar',
        'MW_07_Helical_service_thread',
        'MW_08_Freeform_yaw_cradle',
        'MW_09_Lofted_output_yoke',
        'MW_14_Revolved_encoder_hub',
        'MW_18_Spline_tendon_01',
        'MW_19_Spline_flexure_01',
        'MW_24_Lofted_elastomer_boot',
        'MW_26_Drafted_connector_shell',
    }
    table = {p.name: p for p in assembly.parts}
    assert names <= table.keys()
    for name in names:
        part = table[name]
        assert part.cad is not None, name
        assert part.cad.isValid(), name


def test_fine_sma_geometry_is_continuous_and_truth_labeled(assembly):
    fibers = [p for p in assembly.parts if p.group == 'sma_fibers']
    assert len(fibers) == morphic_wrist.SPEC.sma_banks * morphic_wrist.SPEC.sma_fibers_per_bank
    assert all(p.provenance == 'source-guided-concept' for p in fibers)
    assert all(len(p.vertices) > 20 and len(p.faces) > 20 for p in fibers)


def test_views_match_nitinol_quality_floor(assembly):
    for name in ('hero', 'cutaway', 'geometry_macro', 'lattice_macro', 'exploded'):
        v = assembly.views[name]
        assert v.projection == 'perspective'
        assert v.focal_length_mm >= 60
        assert v.light_size >= 1.5
        assert v.background_strength <= .80
        assert v.environment_strength >= .20
    assert 'shell' in assembly.views['cutaway'].hide
    assert 'overmold' in assembly.views['cutaway'].hide


def test_no_fake_motion_claim(assembly):
    assert 'No SMA contraction' in assembly.metadata['motion_model']
    assert assembly.motion_function is None
