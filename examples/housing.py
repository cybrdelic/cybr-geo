"""General operations construct a machined actuator housing, dimensions in mm.
Not a prefab class: each state is inspectable analytic geometry and triangulation.
This is a geometric design example, not a certified thermomechanical component.
"""
from pathlib import Path
from cybrgeo.solid import Solid
from cybrgeo import save_snapshot, export_obj


def build():
    states=[]
    # The base and circular body genuinely fuse into one volumetric solid.
    flange=Solid.box((84,70,8)).fillet(7,'|Z')
    states.append(('01_flange',flange))
    barrel=Solid.cylinder(29,54).move((0,0,4))
    housing=flange.union(barrel)
    states.append(('02_barrel_union',housing))
    # A blind cavity leaves a 4 mm radial wall and a 6 mm floor.
    cavity=Solid.cylinder(25,60).move((0,0,2))
    housing=housing.subtract(cavity)
    states.append(('03_blind_cavity',housing))
    # Through-bore for the output shaft, then a recessed bearing seat.
    shaft=Solid.cylinder(7,20).move((0,0,-10))
    housing=housing.subtract(shaft)
    states.append(('04_shaft_bore',housing))
    bearing=Solid.cylinder(12,4).move((0,0,-4))
    housing=housing.subtract(bearing)
    states.append(('05_bearing_recess',housing))
    # Four through fixing holes with counterbores; nothing merely intersects.
    for x in (-33,33):
        for y in (-25,25):
            bore=Solid.cylinder(2.6,14).move((x,y,-7))
            counterbore=Solid.cylinder(4.4,3).move((x,y,1))
            housing=housing.subtract(bore,counterbore)
            states.append((f'06_mount_{x}_{y}',housing))
    # Rounded axial ventilation slots are subtracted through the barrel wall.
    # Temporary cutters are supplied too, for true operation visualization.
    cutters=[]
    for angle in range(0,360,45):
        slot=Solid.box((12,5,24)).fillet(2,'|X').move((27,0,32))
        slot=slot.rotate((0,0,0),(0,0,1),angle)
        cutters.append(slot)
        housing=housing.subtract(slot)
        states.append((f'07_vent_{angle}',housing))
    return housing,states,cutters


if __name__=='__main__':
    output=Path('housing_output');output.mkdir(exist_ok=True)
    final,states,cutters=build()
    for name,solid in states:
        mesh=solid.mesh(tolerance=.08)
        save_snapshot(mesh,output/(name+'.npz'))
    mesh=final.mesh(tolerance=.03)
    export_obj(mesh,output/'housing.obj')
    for index,cutter in enumerate(cutters):
        save_snapshot(cutter.mesh(),output/f'cutter_{index}.npz')
    print({'states':len(states),'volume_mm3':final.volume,'solid_count':final.solid_count,'mesh':mesh.report()})
