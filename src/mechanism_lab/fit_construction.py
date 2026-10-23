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
