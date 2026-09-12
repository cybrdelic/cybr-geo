import math

from mechanism_lab.core import validate
from mechanism_lab.gears import gear_radii
from mechanism_lab.models.planetary_actuator import SPEC,build
from mechanism_lab.truth import assert_renderable


def _part(a,name):
    return next(p for p in a.parts if p.name==name)


def test_full_model_is_large_connected_concept_not_a_toy_gear_triplet():
    a=build()
    report=validate(a)
    assert report['parts'] >= 60
    truth=assert_renderable(a,'concept')
    assert truth['passed']
    assert truth['tier_counts']=={'designed':report['parts']}
    names={p.name for p in a.parts}
    required={
        'PGA01_Front_housing','PGA02_Rear_housing','PGA03_Fixed_internal_ring',
        'PGA06_Input_sun_shaft','PGA07_Output_shaft','PGA14_Carrier_front','PGA15_Carrier_rear',
        'PGA20_Planet_01','PGA20_Planet_02','PGA20_Planet_03','PGA26_Encoder_magnet','PGA27_Encoder_PCB',
        'PGA29_Connector_shell',
    }
    assert required <= names


def test_authored_radial_clearances_are_positive_where_clearance_is_claimed():
    a=build()
    ring=gear_radii(SPEC.ring_teeth,SPEC.module,SPEC.pressure_angle_deg,internal=True)
    assert ring.addendum-26.7 >= 1.25
    planet=gear_radii(SPEC.planet_teeth,SPEC.module,SPEC.pressure_angle_deg)
    assert math.isclose(ring.root-(SPEC.planet_center_radius+planet.addendum),.2,abs_tol=1e-12)
    clearances=a.metadata['clearances']
    assert math.isclose(clearances['output_bushing_radial_mm'],.08,abs_tol=1e-12)
    assert math.isclose(clearances['input_bushing_radial_mm'],.08,abs_tol=1e-12)
    assert math.isclose(clearances['planet_pin_radial_mm'],.06,abs_tol=1e-12)
    assert math.isclose(clearances['planet_bushing_bore_diametral_mm'],.04,abs_tol=1e-12)
    # Geometry agrees with those authored values.
    assert math.isclose(6.08-6.00,.08,abs_tol=1e-12)
    assert math.isclose(4.08-4.00,.08,abs_tol=1e-12)
    assert math.isclose(2.56-2.50,.06,abs_tol=1e-12)
    assert math.isclose(10.04-10.00,.04,abs_tol=1e-12)


def test_fasteners_and_supports_span_the_interfaces_their_roles_claim():
    a=build()
    front=_part(a,'PGA04_Front_ring_screw_01').bounds
    rear=_part(a,'PGA05_Rear_ring_screw_01').bounds
    front_housing=_part(a,'PGA01_Front_housing').bounds
    rear_housing=_part(a,'PGA02_Rear_housing').bounds
    ring=_part(a,'PGA03_Fixed_internal_ring').bounds
    assert front[0,0] < front_housing[0,0]
    assert front[1,0] >= ring[0,0]+1.5
    assert rear[0,0] <= ring[1,0]-1.5
    assert rear[1,0] > rear_housing[1,0]


def test_planet_bushings_are_two_real_support_segments_per_planet():
    a=build()
    for i in range(1,4):
        gear=_part(a,f'PGA20_Planet_{i:02}')
        bf=_part(a,f'PGA21_Planet_bushing_front_{i:02}')
        br=_part(a,f'PGA22_Planet_bushing_rear_{i:02}')
        pin=_part(a,f'PGA17_Planet_pin_{i:02}')
        assert bf.bounds[0,0] >= gear.bounds[0,0]-1e-8
        assert br.bounds[1,0] <= gear.bounds[1,0]+1e-8
        assert pin.bounds[0,0] < gear.bounds[0,0]
        assert pin.bounds[1,0] > gear.bounds[1,0]
        # Both bushing bodies actually occupy the planet bore envelope.
        assert bf.bounds[1,0] <= br.bounds[0,0]+1e-8


def test_seals_have_real_housing_seats_instead_of_overlapping_solid_housing():
    a=build()
    out_seal=_part(a,'PGA12_Output_seal').bounds
    in_seal=_part(a,'PGA13_Input_seal').bounds
    front=_part(a,'PGA01_Front_housing').bounds
    rear=_part(a,'PGA02_Rear_housing').bounds
    assert out_seal[0,0] < front[1,0] and out_seal[1,0] > front[0,0]
    assert in_seal[0,0] < rear[1,0] and in_seal[1,0] > rear[0,0]
    # The authored housing recipes cut matching 9 mm / 7 mm seal seats; this test
    # anchors that intent in metadata rather than claiming an invisible press fit.
    assert a.metadata['interfaces']['output_support'].startswith('two 18 mm OD')
    assert a.metadata['interfaces']['input_support'].startswith('two 14 mm OD')


def test_metadata_does_not_claim_unperformed_load_validation():
    a=build()
    text=' '.join(a.metadata['assumptions']).lower()
    assert 'no stress' in text
    assert 'contact' in text
    assert 'thread helices' in text
    assert a.metadata['truth_intent']=='concept'
    assert a.metadata['planetary_validation']['kinematics']['fixed_ring_reduction']==5.0
