"""Source-derived tension-spring anchor demand and native pin/clevis binding.

The peak is an exact extremum of the ideal Hookean model over the declared
joint interval. It is a required interface *demand*, never an allowable pin,
clevis, spring-hook, fastener or fatigue capacity.
"""
from __future__ import annotations

import math

from .actuation import TensionSpring


def _finite(value, label, *, positive=False):
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) or (positive and value <= 0)):
        raise ValueError(f'{label} must be finite' + (' and positive' if positive else ''))
    return float(value)


def tension_anchor_demand(config, joint_range_rad, *, pin_diameter_mm,
                          axis=(0, 0, -1)):
    """Bound force on each separate anchor pin over a full rotational range.

    Rotating the child point about the spring joint axis makes squared eye
    separation ``A+B cos(q)+C sin(q)``. Its maxima lie at the range ends or
    ``atan2(C,B)+2*pi*k``. We evaluate those extrema with the very same spring
    state function used by the dynamic controller.
    """
    if (not isinstance(joint_range_rad, (list, tuple)) or len(joint_range_rad) != 2):
        raise ValueError('Spring anchor requires a two-ended joint range')
    lo, hi = (_finite(value, 'Spring joint limit') for value in joint_range_rad)
    if lo >= hi:
        raise ValueError('Spring anchor joint range must be increasing')
    diameter = _finite(pin_diameter_mm, 'Anchor pin diameter', positive=True)
    count = config.get('spring_count') if isinstance(config, dict) else None
    if isinstance(count, bool) or not isinstance(count, int) or count < 1:
        raise ValueError('One or more physical anchor pins are required')
    spring = TensionSpring(config, axis)
    axis, child, parent = spring.axis, spring.child, spring.parent
    dot = sum(a*b for a, b in zip(axis, child))
    perpendicular = [b-a*dot for a, b in zip(axis, child)]
    cross = [axis[1]*child[2]-axis[2]*child[1],
             axis[2]*child[0]-axis[0]*child[2],
             axis[0]*child[1]-axis[1]*child[0]]
    # Only the phase of the harmonic is needed: the constant term cancels.
    b = -2*sum(p*v for p, v in zip(parent, perpendicular))
    c = -2*sum(p*v for p, v in zip(parent, cross))
    candidates = [lo, hi]
    if b or c:
        phase = math.atan2(c, b)
        first = math.ceil((lo-phase)/math.tau)
        last = math.floor((hi-phase)/math.tau)
        if first <= last:
            candidates.append(phase+math.tau*first)
    # A coincident eye has no defined force direction, even when it occurs
    # between the endpoints and is not the force maximum.
    if b or c:
        trough = math.atan2(c, b) + math.pi
        first = math.ceil((lo-trough)/math.tau)
        last = math.floor((hi-trough)/math.tau)
        if first <= last:
            candidates.append(trough+math.tau*first)
    states = [(q, spring.state(q)) for q in candidates]
    peak_q, peak = max(states, key=lambda item: item[1]['force_each_N'])
    return dict(schema='cybrgeo-spring-anchor-demand/1',
        joint_range_rad=[lo, hi], peak_joint_angle_rad=peak_q,
        peak_eye_length_m=peak['length_m'],
        peak_ideal_force_each_N=peak['force_each_N'],
        required_capacity_per_separate_pin_N=peak['force_each_N'],
        spring_count=count, nominal_pin_diameter_mm=diameter,
        pin_clevis_capacity_verified=False,
        scope='Full declared joint range of the ideal tension-only spring; each spring has separate anchor pins. No pin/clevis bearing, shear, bending, fastener, fatigue or tested spring capacity follows from this demand.')


def member_anchor_positions_mm(config, member_offsets_m, *, child_body_pos_m,
                               child_body_quat_wxyz, joint_pos_m, axis=(0, 0, -1)):
    """Derive each physical eye location from the controlled joint frame.

    The spring source anchors are in the child's unrotated hinge frame.
    Member offsets must run along the hinge axis, so every member has the
    same ideal force curve; a transverse offset would need its own force
    model. The fixed point is transformed into the parent body's coordinates
    at the hinge reference pose, while the moving point remains child-local.
    """
    spring = TensionSpring(config, axis)
    count = config.get('spring_count')
    if isinstance(count, bool) or not isinstance(count, int) or count < 1:
        raise ValueError('Spring member count must be a positive integer')
    if member_offsets_m is None and count == 1:
        member_offsets_m = [[0., 0., 0.]]
    if (not isinstance(member_offsets_m, list) or len(member_offsets_m) != count):
        raise ValueError('Each controlled spring member needs one joint-frame offset')
    body_pos = _vector3(child_body_pos_m, 'Child-body position')
    joint_pos = _vector3(joint_pos_m, 'Joint position')
    quaternion = _vector4(child_body_quat_wxyz, 'Child-body quaternion')
    norm = math.sqrt(sum(value*value for value in quaternion))
    if not math.isclose(norm, 1., rel_tol=0., abs_tol=1e-8):
        raise ValueError('Child-body quaternion must be unit length')
    w, x, y, z = quaternion

    def rotate(vector):
        # MuJoCo's wxyz quaternion maps child-local vectors to parent-local.
        cross = [y*vector[2]-z*vector[1], z*vector[0]-x*vector[2],
                 x*vector[1]-y*vector[0]]
        twice = [2*value for value in cross]
        second = [y*twice[2]-z*twice[1], z*twice[0]-x*twice[2],
                  x*twice[1]-y*twice[0]]
        return [vector[i]+w*twice[i]+second[i] for i in range(3)]

    rows = []
    seen = set()
    for index, raw_offset in enumerate(member_offsets_m):
        offset = _vector3(raw_offset, 'Spring member offset')
        if (index == 0 and any(abs(value) > 1e-12 for value in offset)):
            raise ValueError('First spring member must use the source anchor coordinates')
        axial = sum(a*b for a, b in zip(offset, spring.axis))
        transverse = [offset[i]-axial*spring.axis[i] for i in range(3)]
        if (any(abs(value) > 1e-12 for value in transverse)
                or tuple(offset) in seen):
            raise ValueError('Spring member offsets must be distinct and parallel to the hinge axis')
        seen.add(tuple(offset))
        fixed_in_child = [joint_pos[i]+spring.parent[i]+offset[i] for i in range(3)]
        moving_in_child = [joint_pos[i]+spring.child[i]+offset[i] for i in range(3)]
        fixed_rotated = rotate(fixed_in_child)
        rows.append(dict(member_index=index,
            # Canonicalize binary arithmetic at a sub-picometre CAD scale so
            # 144 mm - 40 mm and 1000*(.144-.04) share one native identity.
            fixed=[round(1000*(body_pos[i]+fixed_rotated[i]), 12) for i in range(3)],
            moving=[round(1000*value, 12) for value in moving_in_child]))
    return rows


def _vector3(value, label):
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f'{label} requires three finite coordinates')
    return [_finite(item, label) for item in value]


def _vector4(value, label):
    if not isinstance(value, (list, tuple)) or len(value) != 4:
        raise ValueError(f'{label} requires four finite coordinates')
    return [_finite(item, label) for item in value]


def verify_native_anchor(pin, clevis, expected_position_mm, *,
                         pin_diameter_mm, pin_length_mm,
                         nominal_bore_diameter_mm):
    """Require the *placed* native pin to occupy a real nominal clevis bore.

    This binds a force-demand receipt to CAD at the same body-local anchor.
    Nominal B-rep clearance is not a production tolerance or rated capacity.
    """
    pin = pin.val() if hasattr(pin, 'val') else pin
    clevis = clevis.val() if hasattr(clevis, 'val') else clevis
    position = [_finite(value, 'Anchor position') for value in expected_position_mm]
    if len(position) != 3 or not pin.isValid() or not clevis.isValid():
        raise ValueError('Anchor requires a valid placed pin, clevis and 3D position')
    diameter = _finite(pin_diameter_mm, 'Pin diameter', positive=True)
    length = _finite(pin_length_mm, 'Pin length', positive=True)
    bore = _finite(nominal_bore_diameter_mm, 'Clevis bore diameter', positive=True)
    if bore <= diameter:
        raise ValueError('Nominal clevis bore must exceed pin diameter')
    # NativeConstruction tessellates placements for display before checking
    # this interface. CadQuery's default BoundingBox then incorporates that
    # mesh's deflection into the dimensions (a 4 mm pin can read 4.0059 mm).
    # Bound the underlying B-rep without triangulation; no design tolerance
    # is relaxed and the solid's actual nominal size remains authoritative.
    from OCP.Bnd import Bnd_Box
    from OCP.BRepBndLib import BRepBndLib
    bounds = Bnd_Box()
    BRepBndLib.AddOptimal_s(pin.wrapped, bounds, False, False)
    xmin, ymin, zmin, xmax, ymax, zmax = bounds.Get()
    center = [(xmin+xmax)/2, (ymin+ymax)/2, (zmin+zmax)/2]
    if (not all(math.isclose(value, target, rel_tol=0, abs_tol=1e-6)
                for value, target in zip(center, position))
            or not math.isclose(xmax-xmin, diameter, rel_tol=0, abs_tol=1e-6)
            or not math.isclose(ymax-ymin, diameter, rel_tol=0, abs_tol=1e-6)
            or not math.isclose(zmax-zmin, length, rel_tol=0, abs_tol=1e-6)
            or len(pin.Solids()) != 1
            or not math.isclose(pin.Volume(), math.pi*(diameter/2)**2*length,
                                rel_tol=1e-6, abs_tol=1e-6)):
        raise ValueError('Placed native spring eye pin differs from its anchor position or dimensions')
    overlap = float(pin.intersect(clevis).Volume())
    distance = float(pin.distance(clevis))
    nominal_gap = (bore-diameter)/2
    if (not math.isfinite(overlap) or not math.isfinite(distance)
            or overlap > 1e-5 or distance < nominal_gap-1e-5
            or distance > nominal_gap+1e-5):
        raise ValueError('Placed native spring eye pin does not mate with the authored clevis bore')
    return dict(body_local_anchor_mm=position, pin_diameter_mm=diameter,
                pin_length_mm=length,
                clevis_bore_diameter_mm=bore, measured_nominal_radial_gap_mm=distance,
                intersection_volume_mm3=overlap,
                scope='Nominal placed B-rep pin/clevis fit only; process variation and physical capacity unmeasured')
