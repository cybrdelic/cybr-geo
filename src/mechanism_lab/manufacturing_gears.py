"""Native sampled-involute pinion/rack construction and explicit SI scaling.

The flank, tip and radial-root construction is adapted directly from
cybrdelic/cybr-arx6-compact at 85bd6d2ce5b7c567d08b3ffccf2a9be46005aabf,
revisions/p6b-print-first/source/build_printed_gear_trials.py. Defaults preserve
those actual prototype solids. This module follows GEO's GPL-2.0-only license.

These are geometric test articles, not qualified gears. No generated trochoid,
undercut/contact-ratio analysis, rolling-contact solution or load rating is
implied. The SI conversions express pitch kinematics and an explicitly assumed
forward efficiency; they are not another simulator or an inherited controller.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import math


SOURCE_COMMIT = '85bd6d2ce5b7c567d08b3ffccf2a9be46005aabf'
SOURCE_PATH = 'revisions/p6b-print-first/source/build_printed_gear_trials.py'
SOURCE_SHA256 = '7d8493b46fa73b21e39441c7b41170d9dd5bbfbd570f4cb466e9e0c24e5209e1'


def _finite(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{label} must be a finite number')
    try:
        value = float(value)
    except OverflowError as error:
        raise ValueError(f'{label} must be representable as a finite number') from error
    if not math.isfinite(value):
        raise ValueError(f'{label} must be a finite number')
    return value


def _positive(value, label):
    value = _finite(value, label)
    if value <= 0:
        raise ValueError(f'{label} must be positive')
    return value


def _integer(value, minimum, label):
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
        raise ValueError(f'{label} must be an integer >= {minimum}')
    return value


def _involute(radius, base_radius):
    # Same source expression; callers use radii on or outside the base circle.
    return math.sqrt(max(0., (radius/base_radius)**2-1))-math.acos(min(1., base_radius/radius))


def _efficiency(value):
    value = _positive(value, 'Forward efficiency')
    if value > 1:
        raise ValueError('Forward efficiency must be <= 1')
    return value


@dataclass(frozen=True)
class InvoluteRackPair:
    """One tooth specification produces both solids and pitch conversion.

    Geometry is local millimetres: pinion axis +Z through the origin, rack
    travel +X with its pitch line at Y=0; both extrude from Z=0. Returned solids
    are separate test articles, not positioned in a meshing assembly. The
    source algorithm requires the root circle below the base circle. Its
    sampled flanks and straight radial roots do not cover arbitrary gear forms.

    ``minimum_wall_mm`` is an author's nominal geometric requirement, not a
    claim that this thickness can be printed or carry a specified load.
    """
    module_mm: float = 1.2
    teeth: int = 20
    pressure_angle_deg: float = 20.
    face_width_mm: float = 12.
    tooth_thickness_reduction_mm: float = .10
    pinion_bore_mm: float = 4.1
    rack_teeth: int = 16
    rack_back_depth_mm: float = 6.
    minimum_wall_mm: float = 1.
    flank_samples: int = 18
    tip_samples: int = 8
    root_samples: int = 7

    def __post_init__(self):
        for key in ('module_mm', 'face_width_mm', 'rack_back_depth_mm', 'minimum_wall_mm'):
            _positive(getattr(self, key), key)
        _integer(self.teeth, 3, 'teeth')
        _integer(self.rack_teeth, 1, 'rack_teeth')
        for key in ('flank_samples', 'tip_samples', 'root_samples'):
            _integer(getattr(self, key), 2, key)
        angle = _finite(self.pressure_angle_deg, 'pressure_angle_deg')
        if not 0 < angle < 90:
            raise ValueError('pressure_angle_deg must be between 0 and 90')
        for key in ('tooth_thickness_reduction_mm', 'pinion_bore_mm'):
            if _finite(getattr(self, key), key) < 0:
                raise ValueError(f'{key} must be nonnegative')
        try:
            p = self._dimensions()
        except (ArithmeticError, ValueError) as error:
            raise ValueError('Derived gear dimensions must be finite and representable') from error
        if not all(math.isfinite(value) for value in p.values()):
            raise ValueError('Derived gear dimensions must be finite')
        if not 0 < p['root_radius_mm'] < p['base_radius_mm'] < p['tip_radius_mm']:
            raise ValueError('Sampled radial-root construction requires 0 < root radius < base radius < tip radius')
        if not 0 < p['tip_half_angle_rad'] < p['base_half_angle_rad'] < math.pi/self.teeth:
            raise ValueError('Pinion needs positive tooth tips and root valleys')
        if not 0 < p['rack_tip_half_width_mm'] < p['rack_root_half_width_mm'] < p['pitch_mm']/2:
            raise ValueError('Rack needs positive tooth tips and root valleys')
        if self.pinion_bore_mm/2+self.minimum_wall_mm > p['root_polygon_apothem_mm']:
            raise ValueError('Pinion bore does not retain the declared minimum wall below its roots')
        if self.rack_back_depth_mm-1.25*self.module_mm < self.minimum_wall_mm:
            raise ValueError('Rack backing does not retain the declared minimum wall below its roots')

    def _dimensions(self):
        rp = self.module_mm*self.teeth/2
        rb = rp*math.cos(math.radians(self.pressure_angle_deg))
        ra, rf = rp+self.module_mm, rp-1.25*self.module_mm
        half = (math.pi*self.module_mm/2-self.tooth_thickness_reduction_mm)/(2*rp)
        phib = half+_involute(rp, rb)
        phia = phib-_involute(ra, rb)
        hp = (math.pi*self.module_mm/2-self.tooth_thickness_reduction_mm)/2
        return dict(pitch_radius_mm=rp, base_radius_mm=rb, tip_radius_mm=ra,
                    root_radius_mm=rf,
                    # The actual polygon root edge lies inside its reference
                    # circle; enforce the wall against the solid, not the circle.
                    root_polygon_apothem_mm=rf*math.cos((2*math.pi/self.teeth-2*phib)/(2*(self.root_samples+1))),
                    base_half_angle_rad=phib, tip_half_angle_rad=phia,
                    pitch_mm=math.pi*self.module_mm,
                    rack_length_mm=self.rack_teeth*math.pi*self.module_mm,
                    rack_tip_half_width_mm=hp-self.module_mm*math.tan(math.radians(self.pressure_angle_deg)),
                    rack_root_half_width_mm=hp+1.25*self.module_mm*math.tan(math.radians(self.pressure_angle_deg)))

    @property
    def pitch_radius_m(self):
        return self.module_mm*self.teeth*.0005

    def rack_force_for_pinion_torque(self, torque_Nm, *, efficiency=1.):
        """Signed forward force; efficiency is an assumption, not a rating."""
        return _finite(_finite(torque_Nm, 'Pinion torque')*_efficiency(efficiency)/self.pitch_radius_m, 'Derived rack force')

    def pinion_torque_for_rack_force(self, force_N, *, efficiency=1.):
        return _finite(_finite(force_N, 'Rack force')*self.pitch_radius_m/_efficiency(efficiency), 'Derived pinion torque')

    def rack_travel_for_pinion_angle(self, angle_rad):
        return _finite(_finite(angle_rad, 'Pinion angle')*self.pitch_radius_m, 'Derived rack travel')

    def pinion_angle_for_rack_travel(self, travel_m):
        return _finite(_finite(travel_m, 'Rack travel')/self.pitch_radius_m, 'Derived pinion angle')

    def scaling_from(self, *, module_mm, teeth):
        """Compare pitch scaling only; no substitute bore or drive is invented."""
        count = _finite(_integer(teeth, 3, 'Reference teeth'), 'Reference teeth')
        old_radius = _positive(_positive(module_mm, 'Reference module')*count*.0005, 'Reference pitch radius')
        ratio = _positive(self.pitch_radius_m/old_radius, 'Pitch radius ratio')
        inverse = _positive(1/ratio, 'Inverse pitch radius ratio')
        return dict(reference_pitch_radius_m=old_radius, pitch_radius_ratio=ratio,
                    torque_ratio_for_same_rack_force=ratio,
                    rack_force_ratio_at_same_pinion_torque=inverse,
                    rack_travel_ratio_per_pinion_radian=ratio)

    def to_source(self):
        return dict(schema='cybrgeo-involute-rack-pair/1', parameters=asdict(self),
                    dimensions_mm_and_radians=self._dimensions(), pitch_radius_m=self.pitch_radius_m,
                    source=dict(repository='cybrdelic/cybr-arx6-compact', commit=SOURCE_COMMIT,
                                path=SOURCE_PATH, sha256=SOURCE_SHA256),
                    scope='Separate native tooth test articles; no meshing assembly or robot integration',
                    limitations=[
                        'Flanks and circular tip/root arcs are polygonally sampled; roots have straight radial transitions, not generated trochoids.',
                        'No rolling interference, undercut, contact-ratio, stress, wear, life or measured backlash qualification.',
                        'Bore and minimum walls are declared geometry, not qualified shaft fits or printed strength.',
                        'Pitch scaling must be reconciled with the actual bridge, jaws, transmission and controller.',
                        'SI force/torque conversion assumes the stated forward efficiency; no back-drive loss, compliance or contact dynamics model.'])

    def native_shapes(self):
        """Build source-derived analytic solids for shared NativeConstruction."""
        import cadquery as cq
        p = self._dimensions()
        rb, ra, rf = p['base_radius_mm'], p['tip_radius_mm'], p['root_radius_mm']
        phib, phia = p['base_half_angle_rad'], p['tip_half_angle_rad']
        points = []

        def add(radius, angle):
            point = cq.Vector(radius*math.cos(angle), radius*math.sin(angle), 0)
            if not points or (point-points[-1]).Length > 1e-8:
                points.append(point)

        for k in range(self.teeth):
            a = 2*math.pi*k/self.teeth
            add(rf, a-phib)
            add(rb, a-phib)
            for j in range(1, self.flank_samples+1):
                r = rb+(ra-rb)*j/self.flank_samples
                add(r, a-(phib-_involute(r, rb)))
            for j in range(1, self.tip_samples+1):
                add(ra, a-phia+2*phia*j/self.tip_samples)
            for j in range(1, self.flank_samples+1):
                r = ra-(ra-rb)*j/self.flank_samples
                add(r, a+(phib-_involute(r, rb)))
            add(rf, a+phib)
            end = 2*math.pi*(k+1)/self.teeth-phib
            # Source: seven interior samples over eight intervals. The next
            # tooth contributes the last endpoint; do not change that polygon.
            for j in range(1, self.root_samples+1):
                add(rf, a+phib+(end-a-phib)*j/(self.root_samples+1))
        pinion = cq.Solid.extrudeLinear(cq.Wire.makePolygon(points, close=True), [], cq.Vector(0, 0, self.face_width_mm))
        if self.pinion_bore_mm:
            pinion = pinion.cut(cq.Solid.makeCylinder(self.pinion_bore_mm/2, self.face_width_mm+2, cq.Vector(0, 0, -1)))
        pinion = pinion.clean()

        pitch = p['pitch_mm']
        length = self.rack_teeth*pitch
        ha, hf = p['rack_tip_half_width_mm'], p['rack_root_half_width_mm']
        outline = [cq.Vector(-length/2, -self.rack_back_depth_mm, 0), cq.Vector(-length/2, -1.25*self.module_mm, 0)]
        for i in range(self.rack_teeth):
            x = -length/2+(i+.5)*pitch
            outline += [cq.Vector(x-hf, -1.25*self.module_mm, 0), cq.Vector(x-ha, self.module_mm, 0),
                        cq.Vector(x+ha, self.module_mm, 0), cq.Vector(x+hf, -1.25*self.module_mm, 0)]
        outline += [cq.Vector(length/2, -1.25*self.module_mm, 0), cq.Vector(length/2, -self.rack_back_depth_mm, 0)]
        rack = cq.Solid.extrudeLinear(cq.Wire.makePolygon(outline, close=True), [], cq.Vector(0, 0, self.face_width_mm)).clean()
        result = dict(pinion=pinion, rack=rack)
        for name, shape in result.items():
            if not shape.isValid() or len(shape.Solids()) != 1 or shape.Volume() <= 0:
                raise ValueError(f'Native {name} construction is not one valid positive-volume solid')
        return result
