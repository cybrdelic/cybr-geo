"""Construct coarser mating rack/pinion CAD from retained source interfaces.

The recipe preserves shaft/hub/retention cuts, jaw mounting tabs and guide-bore
locations. Reciprocal carriage and finger-pad windows derive from analytic sweeps of the
new rack primitives. Geometric ligament limits are declared design constraints,
not process allowables, tooth ratings or hardware qualification.
"""
from copy import deepcopy
import math
import numpy as np
from .shape_dag import ShapeEvaluator,from_tree


def _nodes(tree):
    yield tree
    for child in tree.get('children',[]):yield from _nodes(child)


def _evaluate(tree):return ShapeEvaluator(from_tree(tree)).evaluate().clean()


def _rectangle(x0,x1,y0,y1):return np.array([[x0,y0],[x1,y0],[x1,y1],[x0,y1]])


def _prism(points,z0,z1):
    import cadquery as cq
    return cq.Workplane('XY').polyline(np.asarray(points).tolist()).close().extrude(z1-z0).val().translate((0,0,z0))


def build_coarse_transmission(pinion_tree,rack_trees,carriage_shapes,*,module=.5,source_module=.3,
        pinion_teeth=20,q_limits_mm=(17.5,38.5),clearance_mm=.3,minimum_ligament_mm=2.,
        guide_centers_yz=((-26.2,-10.),(-26.2,10.)),guide_bore_radius_mm=3.,
        retainer_groove=(1.2,3.6,9.2,9.65),profile_only=False,
        jaw_origin_y_mm=None,workpiece_rear_y_mm=None,pad_shapes=None):
    """Return actual transmission CAD, contact profiles and dimensional evidence.

    ``rack_trees``, ``carriage_shapes`` and required native ``pad_shapes`` keys
    are upper/lower. The supplied
    source rack's tab is retained verbatim. Nominal rack pitch span is preserved
    by choosing an integer tooth count. No mating jaw frame is moved.
    """
    if not profile_only:
        import cadquery as cq
        if pad_shapes is None or set(pad_shapes)!={"upper","lower"}:
            raise ValueError("Native transmission requires both actual finger pads; carriage-only clearance is insufficient")
    from shapely.geometry import Polygon
    from shapely.ops import unary_union
    if not source_module<module or min(module,clearance_mm,minimum_ligament_mm)<=0:
        raise ValueError('Require larger module and positive declared clearances/ligaments')
    scale=module/source_module;shift=(module-source_module)*pinion_teeth/2
    source_polygons=[n for n in _nodes(rack_trees['upper']) if n['op']=='polygon']
    count=len(source_polygons);new_count=round(count/scale)
    if not math.isclose(new_count*module,count*source_module,abs_tol=1e-9):raise ValueError('Integer rack tooth count must preserve source pitch span')
    tooth=np.asarray(source_polygons[0]['args'][0],float);tooth[:,0]-=tooth[:,0].mean();tooth*=scale
    pitch=math.pi*module;span=count*math.pi*source_module
    half_height=float(source_polygons[0]['args'][1]);half_width=span/2
    # Existing carriage slot begins at x=1.8mm. Keep the offset connection and
    # full end tooth 0.3mm behind it, retaining the original bolted tab.
    original_end=2.;neck_end=original_end-clearance_mm-.2
    transforms={key:np.asarray(tree['args'],float).reshape(4,4) for key,tree in rack_trees.items()}
    base_end=half_width-(original_end-neck_end)
    last_center=(new_count-1)*pitch/2
    train_shift=min(0.,base_end-(last_center+tooth[:,0].max())-.01)
    profiles={};primitives={};racks={};tabs={}
    for key,tree in rack_trees.items():
        T=transforms[key]
        if tree['op']!='transform' or tree['children'][0]['op']!='union':raise ValueError('Expected explicit retained rack placement and tab union')
        z0=float(T[2,3]);z1=z0+2*half_height
        def body(poly):return np.asarray(poly)@T[:2,:2].T+T[:2,3]
        roots=tooth[:,1].min()-shift;bottom=roots-2.6
        pieces=[_rectangle(-half_width+train_shift,base_end,bottom,roots)]
        teeth=[]
        for i in range(new_count):
            polygon=tooth+[(i-(new_count-1)/2)*pitch+train_shift,-shift]
            pieces.append(polygon);teeth.append(body(polygon).tolist())
        source_tab_edge=half_width
        source_old_neck_start=source_tab_edge-original_end
        # Offset web lies behind the unchanged carriage inner edge; old neck
        # remains in its existing y/z mounting slot and meets the original tab.
        pieces += [_rectangle(source_old_neck_start-1.5,base_end,bottom,-.375),
                   _rectangle(source_old_neck_start,source_tab_edge,-2.975,-.375)]
        body_pieces=[body(p) for p in pieces]
        primitives[key]=body_pieces
        if not profile_only and key=='upper':
            if jaw_origin_y_mm is None or workpiece_rear_y_mm is None:raise ValueError('Native transmission requires explicit workpiece backplane and jaw body origin')
            workpiece_gap=workpiece_rear_y_mm-(max(p[:,1].max() for p in body_pieces)+jaw_origin_y_mm)
            if workpiece_gap<clearance_mm-1e-8:raise ValueError(f'New upper rack violates workpiece backplane clearance: {workpiece_gap} mm')
        if not profile_only:
            native=[_prism(p,z0,z1) for p in body_pieces]
            tab_tree=deepcopy(tree);tab_tree['children']=[deepcopy(tree['children'][0]['children'][1])]
            tab=_evaluate(tab_tree);tabs[key]=tab
            rack=native[0].fuse(*native[1:],tab).clean()
            if not rack.isValid() or len(rack.Solids())!=1:raise ValueError('New rack is disconnected or invalid')
            racks[key]=rack
        profiles[key]=dict(teeth=teeth,collision_polygons=[p.tolist() for p in body_pieces],
                           retained_tab_from_source=True,body_local_z_mm=[z0,z1])
    pinion=deepcopy(pinion_tree);polygons=[n for n in _nodes(pinion) if n['op']=='polygon']
    if len(polygons)!=1:raise ValueError('Expected one gear flank profile, not an inferred gear')
    polygons[0]['args'][0]=(np.asarray(polygons[0]['args'][0])*scale).tolist()
    pinion_xy=polygons[0]['args'][0]
    def polygon_frame(node,T):
        if node['op']=='transform':T=T@np.asarray(node['args'],float).reshape(4,4)
        if node['op']=='polygon':return T
        for child in node.get('children',[]):
            result=polygon_frame(child,T)
            if result is not None:return result
    profile_frame=polygon_frame(pinion,np.eye(4))
    pinion_half_height=float(polygons[0]['args'][1])
    pinion_contact_z=[float(profile_frame[2,3]-pinion_half_height),float(profile_frame[2,3]+pinion_half_height)]
    if profile_only:return dict(pinion_xy_mm=pinion_xy,racks=profiles,module_mm=module,pitch_radius_mm=module*pinion_teeth/2,rack_teeth=new_count,train_phase_shift_mm=train_shift)
    gear=_evaluate(pinion)
    ri,ro,z0,z1=retainer_groove;center=np.asarray(pinion_tree['args']).reshape(4,4)[:3,3]
    groove=cq.Solid.makeCylinder(ro,z1-z0,cq.Vector(center[0],center[1],z0)).cut(cq.Solid.makeCylinder(ri,z1-z0+2,cq.Vector(center[0],center[1],z0-1)))
    gear=gear.cut(groove).clean()
    if not gear.isValid() or len(gear.Solids())!=1:raise ValueError('New pinion is disconnected or invalid')
    carriages={};cutters={};measurements={}
    for key,opposite,sign in [('upper','lower',-1),('lower','upper',1)]:
        source=carriage_shapes[key];bb=source.BoundingBox()
        # The opposite rack's retained tab never enters this carriage over the
        # declared jaw interval; otherwise this external-profile sweep is invalid.
        tab=tabs[opposite].BoundingBox();translations=sorted(2*sign*q for q in q_limits_mm)
        tab_low=tab.xmin+translations[0];tab_high=tab.xmax+translations[1]
        if tab_high>=bb.xmin and tab_low<=bb.xmax:raise ValueError('Opposite tab enters carriage; native subtraction sweep required')
        swept=[]
        for polygon in primitives[opposite]:
            ends=np.vstack([polygon+[translations[0],0],polygon+[translations[1],0]])
            swept.append(Polygon(ends).convex_hull)
        footprint=unary_union(swept).buffer(clearance_mm,join_style=2)
        geoms=list(footprint.geoms) if hasattr(footprint,'geoms') else [footprint]
        pieces=[_prism(np.asarray(p.exterior.coords)[:-1],-half_height-clearance_mm,half_height+clearance_mm) for p in geoms]
        cutter=pieces[0].fuse(*pieces[1:]).clean() if len(pieces)>1 else pieces[0]
        updated=source.cut(cutter).clean()
        if not updated.isValid() or len(updated.Solids())!=1:raise ValueError('Reciprocal window disconnects carriage')
        local_window_min=min(p.bounds[1] for p in geoms)
        rear_web=local_window_min-bb.ymin
        bore_gaps=[]
        for y,z in guide_centers_yz:
            bore=cq.Solid.makeCylinder(guide_bore_radius_mm,bb.xlen+2,cq.Vector(bb.xmin-1,y,z),cq.Vector(1,0,0))
            bore_gaps.append(float(cutter.distance(bore)))
        measurement=dict(rear_web_mm=rear_web,guide_bore_ligaments_mm=bore_gaps,
            minimum_declared_ligament_mm=minimum_ligament_mm,removed_volume_mm3=source.Volume()-updated.Volume(),
            valid=updated.isValid(),solid_count=len(updated.Solids()))
        measurements[key]=measurement
        if rear_web<minimum_ligament_mm-1e-8 or min(bore_gaps)<minimum_ligament_mm-1e-8:
            raise ValueError('Declared nominal carriage ligament violated: '+str(measurement))
        carriages[key]=updated;cutters[key]=cutter
    pads={};pad_measurements={}
    if pad_shapes is not None:
        for key,source in pad_shapes.items():
            # Reuse the SAME opposite-rack swept clearance as the carriage;
            # also clear the rack fixed to this jaw, without moving its tab.
            own=unary_union([Polygon(p) for p in primitives[key]]).buffer(clearance_mm,join_style=2)
            geoms=list(own.geoms) if hasattr(own,'geoms') else [own]
            own_pieces=[_prism(np.asarray(p.exterior.coords)[:-1],-half_height-clearance_mm,half_height+clearance_mm) for p in geoms]
            cutter=cutters[key].fuse(*own_pieces).clean()
            updated=source.cut(cutter).clean()
            if not updated.isValid() or len(updated.Solids())!=1:
                raise ValueError('Rack passage disconnects finger pad')
            # The actual object can contact only forward of its rear plane.
            # Preserve ALL source pad material in that half-space, not merely
            # a sampled face or an approximate contact-area claim.
            bb=source.BoundingBox();rear=workpiece_rear_y_mm-jaw_origin_y_mm
            region=cq.Solid.makeBox(bb.xlen+2,max(1.,bb.ymax-rear+1),bb.zlen+2,cq.Vector(bb.xmin-1,rear,bb.zmin-1))
            old_patch=source.intersect(region);new_patch=updated.intersect(region)
            if old_patch.Volume()<=0:raise ValueError('Workpiece contact region does not intersect source pad')
            patch_loss=old_patch.cut(new_patch).Volume()
            if patch_loss>1e-7:raise ValueError('Rack passage removes workpiece contact material')
            pads[key]=updated
            pad_measurements[key]=dict(removed_volume_mm3=source.Volume()-updated.Volume(),
                retained_contact_region_mm3=new_patch.Volume(),contact_region_loss_mm3=patch_loss,
                valid=updated.isValid(),solid_count=len(updated.Solids()))
    return dict(pinion=gear,racks=racks,carriages=carriages,window_cutters=cutters,pads=pads,
        contact_profiles=dict(pinion_xy_mm=pinion_xy,pinion_body_local_z_mm=pinion_contact_z,racks=profiles),
        measurements=dict(q_limits_mm=list(q_limits_mm),module_mm=module,pinion_teeth=pinion_teeth,pitch_radius_mm=module*pinion_teeth/2,
            rack_teeth=new_count,pitch_span_mm=span,train_phase_shift_mm=train_shift,carriages=measurements,pads=pad_measurements,
            nominal_workpiece_plane_gap_mm=workpiece_gap,workpiece_rear_y_mm=workpiece_rear_y_mm,jaw_origin_y_mm=jaw_origin_y_mm,
            tooth_tip_width_mm=float(abs(tooth[2,0]-tooth[1,0])),tooth_root_width_mm=float(abs(tooth[3,0]-tooth[0,0])),
            shaft_and_mounting_interfaces_retained=True,hardware_qualified=False,
            scope='Native mating geometry and declared nominal ligament screen; not printed tooth strength, process or motor capacity'))
