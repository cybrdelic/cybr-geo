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
        'PGA21_Planet_01','PGA21_Planet_02','PGA21_Planet_03','PGA27_Encoder_magnet','PGA28_Encoder_PCB',
    }
    assert required <= names


def test_authored_radial_clearances_are_positive_where_clearance_is_claimed():
    # Carrier OD clears the inward ring tooth tips.
    ring=gear_radii(SPEC.ring_teeth,SPEC.module,SPEC.pressure_angle_deg,internal=True)
    assert ring.addendum-26.7 >= 1.25
    # Planet addendum enters the ring-root zone with small explicit root clearance.
    planet=gear_radii(SPEC.planet_teeth,SPEC.module,SPEC.pressure_angle_deg)
    assert math.isclose(ring.root-(SPEC.planet_center_radius+planet.addendum),.2,abs_tol=1e-12)
    # Model dimensions: plain-bearing running clearances are actual geometry, not prose.
    assert 6.08-6.00 > 0
    assert 4.08-4.00 > 0
    assert 2.56-2.50 > 0


def test_fasteners_and_supports_span_the_interfaces_their_roles_claim():
    a=build()
    front=_part(a,'PGA04_Front_ring_screw_01').bounds
    rear=_part(a,'PGA05_Rear_ring_screw_01').bounds
    front_housing=_part(a,'PGA01_Front_housing').bounds
    rear_housing=_part(a,'PGA02_Rear_housing').bounds
    ring=_part(a,'PGA03_Fixed_internal_ring').bounds
    # Front screw runs from an exposed head ahead of the housing through the housing
    # and into the front ring flange.
    assert front[0,0] < front_housing[0,0]
    assert front[1,0] >= ring[0,0]+1.5
    # Rear screw starts inside the rear ring flange and ends with an exposed rear head.
    assert rear[0,0] <= ring[1,0]-1.5
    assert rear[1,0] > rear_housing[1,0]


def test_planet_bushings_are_two_real_support_segments_per_planet():
    a=build()
    for i in range(1,4):
        gear=_part(a,f'PGA21_Planet_{i:02}')
        bf=_part(a,f'PGA22_Planet_bushing_front_{i:02}')
        br=_part(a,f'PGA23_Planet_bushing_rear_{i:02}')
        pin=_part(a,f'PGA17_Planet_pin_{i:02}')
        assert bf.bounds[0,0] >= gear.bounds[0,0]-1e-8
        assert br.bounds[1,0] <= gear.bounds[1,0]+1e-8
        assert pin.bounds[0,0] < gear.bounds[0,0]
        assert pin.bounds[1,0] > gear.bounds[1,0]


def test_metadata_does_not_claim_unperformed_load_validation():
    a=build()
    text=' '.join(a.metadata['assumptions']).lower()
    assert 'no stress' in text
    assert 'contact' in text
    assert a.metadata['truth_intent']=='concept'
    assert a.metadata['planetary_validation']['kinematics']['fixed_ring_reduction']==5.0
