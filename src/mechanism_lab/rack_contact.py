"""Exact geometric contact decomposition for planar rack-drive diagnostics.

These helpers preserve the supplied profile/rectilinear CAD, rather than using
one convex hull or a bounding box for a notched component. They do not provide
motor ratings, friction data, flexibility, wear or process qualification.
"""
import math
import numpy as np
from shapely.geometry import Polygon
from shapely.ops import triangulate,unary_union


def convex_profile_parts(profile,*,angular_sectors=1):
    polygon=Polygon(profile)
    if not polygon.is_valid or polygon.area<=0:raise ValueError('Valid positive-area profile required')
    regions=[polygon]
    if angular_sectors>1:
        radius=3*max(np.linalg.norm(p) for p in profile)
        regions=[]
        for k in range(angular_sectors):
            a=(k-.5)*math.tau/angular_sectors;b=(k+.5)*math.tau/angular_sectors
            regions.append(polygon.intersection(Polygon([(0,0),(radius*math.cos(a),radius*math.sin(a)),(radius*math.cos(b),radius*math.sin(b))])))
    result=[]
    for region in regions:
        pieces=[t for t in triangulate(region) if region.covers(t.representative_point()) and t.difference(region).area<1e-10]
        changed=True
        while changed:
            changed=False
            for i in range(len(pieces)):
                for j in range(i):
                    if not pieces[i].touches(pieces[j]):continue
                    merged=pieces[i].union(pieces[j])
                    if isinstance(merged,Polygon) and merged.convex_hull.area-merged.area<1e-11:
                        pieces[j]=merged;pieces.pop(i);changed=True;break
                if changed:break
        result.extend(pieces)
    difference=polygon.symmetric_difference(unary_union(result)).area
    if difference>1e-8:raise ValueError('Convex contact decomposition changed the supplied profile')
    return result,difference


def rectilinear_solid_boxes(shape):
    """Return exact disjoint box cells for axis-aligned cut-stock CAD, in mm.

    Rejects nonrectilinear geometry rather than silently overfilling a notch.
    This is useful for a real relieved pad whose bounding box falsely jams a rack.
    """
    import cadquery as cq
    if not shape.isValid() or len(shape.Solids())!=1:raise ValueError('One valid solid required')
    for face in shape.Faces():
        normal=np.abs(face.normalAt().toTuple())
        if face.geomType()!='PLANE' or not np.isclose(normal.max(),1.,atol=1e-8):raise ValueError('Contact stock is not axis-aligned rectilinear CAD')
    vertices=np.asarray([v.toTuple() for v in shape.Vertices()])
    axes=[np.unique(np.round(vertices[:,i],9)) for i in range(3)];cells=[]
    solid=shape.Solids()[0]
    for x0,x1 in zip(axes[0][:-1],axes[0][1:]):
        for y0,y1 in zip(axes[1][:-1],axes[1][1:]):
            for z0,z1 in zip(axes[2][:-1],axes[2][1:]):
                lo=np.array([x0,y0,z0]);hi=np.array([x1,y1,z1]);mid=(lo+hi)/2
                if solid.isInside(cq.Vector(*mid),1e-8):cells.append((lo,hi))
    volume=sum(float(np.prod(hi-lo)) for lo,hi in cells)
    error=abs(volume-shape.Volume())
    if error>max(1e-7,shape.Volume()*1e-9):raise ValueError('Box contact partition differs from native solid volume')
    return cells,error
