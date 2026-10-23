"""Derive mating pin/bore geometry from an authored service and variation budget.

The two variation allowances are design targets for participating parts. They
do not assert that a shop can hold them, or that an assembled bearing has been
tested. Those claims require the independent qualification evidence path.
"""
from __future__ import annotations

from dataclasses import dataclass
import math


def _positive(value, label):
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) or value <= 0):
        raise ValueError(f'{label} must be finite and positive')
    return float(value)


def _nonnegative(value, label):
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) or value < 0):
        raise ValueError(f'{label} must be finite and nonnegative')
    return float(value)


@dataclass(frozen=True)
class CoaxialShaftHubSeat:
    """Reject a nominal shaft/hub mismatch before either part is placed.

    This is a mating-diameter design constraint, not evidence that a key,
    clamp, bearing, or vendor hub can transfer the requested torque. The
    minimum and maximum gaps are authored design targets, not measured
    process capabilities.
    """

    shaft_diameter_mm: float
    hub_bore_diameter_mm: float
    minimum_nominal_radial_gap_mm: float
    maximum_nominal_radial_gap_mm: float

    def __post_init__(self):
        _positive(self.shaft_diameter_mm, 'shaft_diameter_mm')
        _positive(self.hub_bore_diameter_mm, 'hub_bore_diameter_mm')
        minimum = _nonnegative(self.minimum_nominal_radial_gap_mm,
                               'minimum_nominal_radial_gap_mm')
        maximum = _positive(self.maximum_nominal_radial_gap_mm,
                            'maximum_nominal_radial_gap_mm')
        if maximum < minimum:
            raise ValueError('Maximum nominal shaft/hub gap cannot be smaller than minimum')
        gap = self.nominal_radial_gap_mm
        if not minimum <= gap <= maximum:
            raise ValueError('Shaft/hub nominal radial gap lies outside its authored mating interval')

    @property
    def nominal_radial_gap_mm(self):
        return (self.hub_bore_diameter_mm - self.shaft_diameter_mm) / 2.

    def report(self):
        return dict(schema='cybrgeo-coaxial-shaft-hub-seat/1',
                    shaft_diameter_mm=self.shaft_diameter_mm,
                    hub_bore_diameter_mm=self.hub_bore_diameter_mm,
                    nominal_radial_gap_mm=self.nominal_radial_gap_mm,
                    minimum_nominal_radial_gap_mm=self.minimum_nominal_radial_gap_mm,
                    maximum_nominal_radial_gap_mm=self.maximum_nominal_radial_gap_mm,
                    keyed_torque_capacity_verified=False,
                    process_capability_verified=False,
                    scope='Nominal mating diameters only; no keyed torque, axial retention, supplier fit or measured process capability')


@dataclass(frozen=True)
class PinBoreFit:
    """One nominal bore with a positive reserve beyond declared part variation."""

    pin_diameter_mm: float
    required_service_radial_gap_mm: float
    pin_declared_radial_variation_mm: float
    bore_declared_radial_variation_mm: float
    design_reserve_radial_gap_mm: float

    def __post_init__(self):
        for name in self.__dataclass_fields__:
            _positive(getattr(self, name), name)
        if not math.isfinite(self.nominal_bore_diameter_mm):
            raise ValueError('Derived nominal bore diameter must be finite')

    @classmethod
    def from_document(cls, pin_diameter_mm, document):
        fields = {'required_service_radial_gap_mm',
                  'pin_declared_radial_variation_mm',
                  'bore_declared_radial_variation_mm',
                  'design_reserve_radial_gap_mm'}
        if not isinstance(document, dict) or set(document) != fields:
            raise ValueError('Pin/bore fit needs service gap, two declared variations and a positive design reserve')
        return cls(pin_diameter_mm, **document)

    @property
    def nominal_radial_gap_mm(self):
        return (self.required_service_radial_gap_mm
                + self.pin_declared_radial_variation_mm
                + self.bore_declared_radial_variation_mm
                + self.design_reserve_radial_gap_mm)

    @property
    def nominal_diametral_clearance_mm(self):
        return 2 * self.nominal_radial_gap_mm

    @property
    def nominal_bore_diameter_mm(self):
        return self.pin_diameter_mm + self.nominal_diametral_clearance_mm

    @property
    def worst_declared_radial_gap_mm(self):
        return (self.nominal_radial_gap_mm
                - self.pin_declared_radial_variation_mm
                - self.bore_declared_radial_variation_mm)

    def report(self):
        return dict(schema='cybrgeo-pin-bore-fit-design/1',
                    pin_diameter_mm=self.pin_diameter_mm,
                    nominal_bore_diameter_mm=self.nominal_bore_diameter_mm,
                    nominal_radial_gap_mm=self.nominal_radial_gap_mm,
                    required_service_radial_gap_mm=self.required_service_radial_gap_mm,
                    pin_declared_radial_variation_mm=self.pin_declared_radial_variation_mm,
                    bore_declared_radial_variation_mm=self.bore_declared_radial_variation_mm,
                    design_reserve_radial_gap_mm=self.design_reserve_radial_gap_mm,
                    worst_declared_radial_gap_mm=self.worst_declared_radial_gap_mm,
                    process_capability_verified=False,
                    scope='Constructed nominal pin/bore geometry, authored variation budget and positive design reserve only; no measured process capability')


@dataclass(frozen=True)
class DirectedFaceGap:
    """Construct ordered, parallel mating faces with a positive design reserve.

    ``first`` and ``second`` identify physical parts, rather than a tolerance
    applied to a generic clearance check after construction. Both authored
    variations may close the gap. Coordinates are along the caller's declared
    increasing axis; the caller must also check that the constructed parts
    remain attached to their supports.
    """

    required_service_gap_mm: float
    first_declared_variation_mm: float
    second_declared_variation_mm: float
    design_reserve_mm: float

    def __post_init__(self):
        for name in self.__dataclass_fields__:
            _positive(getattr(self, name), name)
        if not math.isfinite(self.nominal_gap_mm):
            raise ValueError('Derived nominal face gap must be finite')

    @classmethod
    def from_document(cls, required_service_gap_mm, document):
        fields = {'first_declared_variation_mm',
                  'second_declared_variation_mm', 'design_reserve_mm'}
        if not isinstance(document, dict) or set(document) != fields:
            raise ValueError('Face gap needs two declared variations and a positive design reserve')
        return cls(required_service_gap_mm, **document)

    @property
    def nominal_gap_mm(self):
        return (self.required_service_gap_mm + self.first_declared_variation_mm
                + self.second_declared_variation_mm + self.design_reserve_mm)

    @property
    def worst_declared_gap_mm(self):
        return self.nominal_gap_mm - self.first_declared_variation_mm - self.second_declared_variation_mm

    def second_face_after(self, first_face_mm):
        if isinstance(first_face_mm, bool) or not isinstance(first_face_mm, (int, float)) or not math.isfinite(first_face_mm):
            raise ValueError('First face coordinate must be finite')
        return float(first_face_mm) + self.nominal_gap_mm

    def first_face_before(self, second_face_mm):
        if isinstance(second_face_mm, bool) or not isinstance(second_face_mm, (int, float)) or not math.isfinite(second_face_mm):
            raise ValueError('Second face coordinate must be finite')
        return float(second_face_mm) - self.nominal_gap_mm

    def require_second_face_at_or_beyond(self, first_face_mm, second_face_mm):
        """Reject a proposed placement that spends the service/variation reserve."""
        if (isinstance(first_face_mm, bool) or not isinstance(first_face_mm, (int, float))
                or not math.isfinite(first_face_mm) or isinstance(second_face_mm, bool)
                or not isinstance(second_face_mm, (int, float))
                or not math.isfinite(second_face_mm)):
            raise ValueError('Both directed face coordinates must be finite')
        if float(second_face_mm) - float(first_face_mm) + 1e-12 < self.nominal_gap_mm:
            raise ValueError('Proposed face placement consumes the authored service gap, variation or design reserve')
        return float(second_face_mm)

    def declared_design_variations_mm(self):
        return dict(first_mm=self.first_declared_variation_mm,
                    second_mm=self.second_declared_variation_mm)

    def report(self):
        return dict(schema='cybrgeo-directed-face-gap-design/1',
                    required_service_gap_mm=self.required_service_gap_mm,
                    first_declared_variation_mm=self.first_declared_variation_mm,
                    second_declared_variation_mm=self.second_declared_variation_mm,
                    design_reserve_mm=self.design_reserve_mm,
                    nominal_gap_mm=self.nominal_gap_mm,
                    worst_declared_gap_mm=self.worst_declared_gap_mm,
                    process_capability_verified=False,
                    scope='Ordered nominal face construction with authored variation and positive reserve; no measured process capability or support-strength proof')
