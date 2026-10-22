"""Invariant IRIS leaf crop derived from the source's circles and canted slab.

The shader clips material against a fixed world cylinder at every frame. This
module instead cuts one rigid blank that fits that cylinder throughout a
declared hinge interval. It preserves the source profile and cant; the crop is
an explicit engineering adaptation, not a claim about the original animation.
"""
from __future__ import annotations

import math

import cadquery as cq
import numpy as np

from mechanism_lab.shape_dag import ShapeEvaluator, from_trees
from .design import SOURCE, leaf_tree


def _range(hinge_range_rad):
    if len(hinge_range_rad) != 2:
        raise ValueError('A leaf hinge interval requires two angles')
    low, high = map(float, hinge_range_rad)
    if not all(math.isfinite(v) for v in (low, high)) or not 0 < high-low < math.pi:
        raise ValueError('A finite ordered hinge interval shorter than pi is required')
    return low, high


def swept_radius_mm(xy, hinge_radius_mm, hinge_range_rad=(0., .9)):
    """Exact maximum world radius of one local XY point over the interval.

    The maximum of x*cos(q)+y*sin(q) occurs at an endpoint or at the point's
    polar angle (modulo 2*pi). This measures continuous motion, not a time grid.
    """
    low, high = _range(hinge_range_rad)
    x, y = map(float, xy)
    hinge = float(hinge_radius_mm)
    if not all(math.isfinite(v) for v in (x, y, hinge)) or hinge <= 0:
        raise ValueError('Finite XY coordinates and a positive hinge radius are required')
    maximum = max(x*math.cos(low)+y*math.sin(low), x*math.cos(high)+y*math.sin(high))
    angle = math.atan2(y, x)
    k = math.ceil((low-angle)/(2*math.pi))
    if angle + k*2*math.pi <= high:
        maximum = math.hypot(x, y)
    return math.sqrt(max(0., hinge*hinge+x*x+y*y+2*hinge*maximum))


def build_leaf(scene_unit_mm=50., hinge_range_rad=(0., .9), envelope_radius_mm=None):
    """Return a native fixed leaf and its explicitly bounded design record.

    Endpoint disks enforce the endpoint maxima. Within the angular wedge
    [low, high], the remaining maximum is H+|p|, so removing wedge material
    outside radius R-H enforces all intermediate angles exactly. No sampled
    trim, posed material removal, or prescribed dynamics is used.
    """
    low, high = _range(hinge_range_rad)
    scale = float(scene_unit_mm)
    if isinstance(scene_unit_mm, bool) or not math.isfinite(scale) or scale <= 0:
        raise ValueError('A finite positive scene-unit conversion is required')
    hinge = SOURCE['leaf']['hinge_radius']*scale
    radius = SOURCE['leaf']['world_clip_radius']*scale if envelope_radius_mm is None else float(envelope_radius_mm)
    if not math.isfinite(radius) or radius <= hinge:
        raise ValueError('Envelope radius must be finite and exceed the hinge radius')
    original = ShapeEvaluator(from_trees({'leaf': leaf_tree(scale)})).evaluate('leaf')
    bounds = original.BoundingBox()
    zmin, zmax = bounds.zmin-scale, bounds.zmax+scale
    height = zmax-zmin
    shape = original
    for angle in (low, high):
        center = (-hinge*math.cos(angle), -hinge*math.sin(angle), zmin)
        shape = shape.intersect(cq.Solid.makeCylinder(radius, height, center))
    # The triangle's distant chord lies strictly outside the entire blank.
    reach = 4*max(math.hypot(x,y) for x in (bounds.xmin,bounds.xmax)
                  for y in (bounds.ymin,bounds.ymax))/math.cos((high-low)/2)
    wedge = (cq.Workplane('XY').polyline([(0.,0.),
        (reach*math.cos(low),reach*math.sin(low)),
        (reach*math.cos(high),reach*math.sin(high))]).close()
        .extrude(height).translate((0.,0.,zmin)).val())
    inner = cq.Solid.makeCylinder(radius-hinge, height, (0.,0.,zmin))
    shape = shape.cut(wedge.cut(inner)).clean()
    if not shape.isValid() or len(shape.Solids()) != 1 or shape.Volume() <= 0:
        raise ValueError('Continuous-envelope trim did not produce one valid rigid leaf')
    vertices, _ = shape.tessellate(.01, .025)
    measured = max(swept_radius_mm(v.toTuple()[:2], hinge, (low,high)) for v in vertices)
    if measured > radius+1e-6:
        raise ValueError('Native cropped boundary exceeds the analytically defined envelope')
    metadata = dict(schema='cybrgeo-iris-rigid-leaf/1', source=SOURCE['source'],
        adaptation='One fixed crop of original two-circle/canted-slab material; no per-pose world clipping',
        scene_unit_mm=scale, hinge_radius_mm=hinge, hinge_range_rad=[low,high],
        radial_envelope_mm=radius, exact_constraint='H^2+x^2+y^2+2*H*max_q(x*cos(q)+y*sin(q)) <= R^2',
        construction='Intersection of endpoint disks, minus the outward angular wedge beyond R-H',
        original_volume_mm3=float(original.Volume()), fixed_volume_mm3=float(shape.Volume()),
        vertical_thickness_mm=SOURCE['leaf']['vertical_thickness']*scale,
        cant=SOURCE['leaf']['cant'], boundary_continuous_radius_max_mm=measured,
        boundary_tessellation_tolerance_mm=.01,
        qualification='Exact nominal radial-envelope construction; no supplier fit, foil stiffness, fatigue, tolerance stack or hardware qualification')
    return shape, metadata


def cut_fixed_shaft_keepouts(shape, *, scene_unit_mm=50., hinge_range_rad=(0., .9),
                            shaft_radius_mm=1., clearance_mm=.1):
    """Remove the exact continuous swept envelopes of the eight other shafts.

    The shafts are fixed to the housing. In one leaf's local frame each shaft
    centre follows a circular arc as that leaf turns. The disk swept along an
    arc is an annular sector plus its two round ends. These actual CAD cuts
    leave clearance for every angle in the declared interval, without changing
    the shape or its mass during motion. The leaf's own central attachment is
    retained; its fit/attachment is a separate authored interface.
    """
    low, high = _range(hinge_range_rad)
    scale, shaft, clearance = map(float, (scene_unit_mm, shaft_radius_mm, clearance_mm))
    if not all(math.isfinite(v) for v in (scale,shaft,clearance)) or scale <= 0 or shaft <= 0 or clearance < 0:
        raise ValueError('Positive scale/shaft and nonnegative finite clearance are required')
    radius = shaft+clearance
    hinge = SOURCE['leaf']['hinge_radius']*scale
    count = SOURCE['leaf']['count']
    bounds = shape.BoundingBox()
    zmin, height = bounds.zmin-scale, bounds.zmax-bounds.zmin+2*scale
    result, arcs = shape, []
    for index in range(1,count):
        azimuth = index*2*math.pi/count
        dx,dy = hinge*(math.cos(azimuth)-1), -hinge*math.sin(azimuth)
        distance, angle = math.hypot(dx,dy), math.atan2(dy,dx)
        if distance <= radius:
            raise ValueError('Shaft envelopes overlap the leaf pivot')
        first,last = angle+low,angle+high
        reach = 4*(distance+radius)/math.cos((last-first)/2)
        wedge = (cq.Workplane('XY').polyline([(0.,0.),
            (reach*math.cos(first),reach*math.sin(first)),
            (reach*math.cos(last),reach*math.sin(last))]).close()
            .extrude(height).translate((0.,0.,zmin)).val())
        annulus = cq.Solid.makeCylinder(distance+radius,height,(0.,0.,zmin)).cut(
            cq.Solid.makeCylinder(distance-radius,height,(0.,0.,zmin)))
        sector = annulus.intersect(wedge)
        ends = [cq.Solid.makeCylinder(radius,height,(distance*math.cos(a),distance*math.sin(a),zmin))
                for a in (first,last)]
        keepout = sector.fuse(*ends).clean()
        result = result.cut(keepout).clean()
        arcs.append(dict(fixed_pivot_index=index,local_arc_radius_mm=distance,
                         local_arc_range_rad=[first,last]))
    if not result.isValid() or len(result.Solids()) != 1 or result.Volume() <= 0:
        raise ValueError('Shaft keepouts disconnect the rigid leaf; revise the actual interface geometry')
    pivot_z = SOURCE['leaf']['height']*scale-SOURCE['leaf']['vertical_thickness']*scale/2
    if not result.isInside((0.,0.,pivot_z),1e-8):
        raise ValueError('Continuous shaft keepouts removed the leaf pivot attachment')
    return result, dict(schema='cybrgeo-iris-shaft-keepouts/1',
        method='Exact annular sectors plus round ends: fixed circular shafts swept in rotating leaf coordinates',
        hinge_range_rad=[low,high], shaft_radius_mm=shaft, radial_clearance_mm=clearance,
        other_fixed_pivot_count=count-1, arcs=arcs, initial_volume_mm3=float(shape.Volume()),
        final_volume_mm3=float(result.Volume()), connected_solids=len(result.Solids()),
        own_pivot_material_retained=True,
        qualification='Nominal rigid geometry only; foil stiffness, pivot fastening and fit require separate qualification')


def projected_face_gap_mm(points_xy, hinge_radians, first_index, second_index, scene_unit_mm=50.):
    """Minimum vertical face gap over a convex projected intersection polygon.

    The caller supplies actual projected-overlap vertices. Each slab height is
    affine in XY, hence its extrema occur at vertices; a sign crossing means
    intersecting mid-surfaces. This is a nominal leaf-face check, not bearing or
    mounting clearance and not a whole-assembly contact model.
    """
    points = np.asarray(points_xy, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2 or not len(points) or not np.isfinite(points).all():
        raise ValueError('Finite convex overlap polygon vertices are required')
    radius = SOURCE['leaf']['hinge_radius']*scene_unit_mm
    def height(index):
        phi = index*2*math.pi/SOURCE['leaf']['count']
        local = points-[radius*math.cos(phi),-radius*math.sin(phi)]
        y = local[:,0]*math.sin(phi+hinge_radians)+local[:,1]*math.cos(phi+hinge_radians)
        return -SOURCE['leaf']['cant']*y
    difference = height(second_index)-height(first_index)
    minimum = 0. if difference.min() <= 0 <= difference.max() else float(np.min(np.abs(difference)))
    return minimum-SOURCE['leaf']['vertical_thickness']*scene_unit_mm
