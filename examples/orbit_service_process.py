"""ORBIT's shop-order service plan. Geometry is supplied by orbit_inspection_wrist.

Service units: complete ball bearings, bonded finger pads, bush-lined carriages,
and the palm with its dressed harness. No balls fly through races and no cable
is passed sideways through a bore. The harness is fed axially with the palm.
"""
from __future__ import annotations
import math
import numpy as np
from mechanism_lab.assembly_process import AssemblyProcess,Move,ToolAccess


def procedure(assembly):
    parts={p.name:p for p in assembly.parts};names=list(parts);moves=[]
    state={n:np.eye(4) for n in names};slot=0
    def select(*prefixes):return tuple(n for n in names if n.startswith(prefixes))
    def bounds(group):
        points=[]
        for n in group:
            p=parts[n];T=state[n];v=p.vertices@T[:3,:3].T+T[:3,3]
            points.extend((v.min(0),v.max(0)))
        return np.array([np.min(points,0),np.max(points,0)])
    def move(title,group,delta,duration=.55,axis=(1,0,0),turns=0,pivot=(0,0,0),tool=None,note=''):
        if not group:return
        op=Move(f'{len(moves)+1:03d}',title,tuple(group),tuple(delta),duration,tuple(axis),turns,tuple(pivot),
                (moves[-1].id,) if moves else (),tool,note)
        moves.append(op)
        for n in group:state[n]=op.transform(1)@state[n]
    def park(title,group,large=False):
        nonlocal slot
        # An explicit clear-height transfer, followed by placement on the bench.
        # Final locations are distinct; removed parts remain collision obstacles.
        b=bounds(group);move('Lift '+title,group,(0,0,160-b[0,2]),.25)
        if large:
            target=np.array([-260+(slot%4)*170.,-280-(slot//4)*170.,0.]);slot+=1
        else:
            target=np.array([-330+(slot%12)*50.,260+(slot//12)*60.,0.]);slot+=1
        b=bounds(group);c=b.mean(0)
        move('Transfer '+title,group,(target[0]-c[0],target[1]-c[1],0),.25)
        b=bounds(group);move('Place '+title,group,(0,0,-1.99-b[0,2]),.25,note='Manually supported, then placed with 0.01 mm tessellation clearance above the bench; no gravity simulation.')
    def take(title,group,axis,distance,large=False):
        move(title,group,np.asarray(axis)*distance,.9,axis=axis);park(title,group,large)
    def screw(name,axis,pitch=.5,engagement=5.,small=False,bend=None):
        group=(name,);axis=np.asarray(axis,float);b=bounds(group);pivot=b.mean(0)
        # Axis-aligned driver insertion ends just outside the socket mouth.
        tip=pivot+axis*(np.dot(b[1]-b[0],np.abs(axis))*.5+.1)
        tool=ToolAccess(tuple(tip),tuple(axis),.5 if small else 1.15,
                        4. if bend else 100.,8.,60. if bend else 35.,bend,
                        'Short-leg hex key; reposition between strokes' if bend else 'Straight hex driver')
        move('Unscrew '+name,group,axis*engagement,.75,axis,engagement/pitch,pivot,tool,
             'Turn and withdraw at nominal pitch; smooth fastener thread envelope, not resolved helical bolt flanks.')
        distance=110. if name.startswith('P00_') else 55. if name.startswith('H03_') else 35. if name.startswith('P03_') else 18.
        move('Withdraw '+name,group,axis*distance,.3,axis=axis)
        park(name,group)

    # The flange screws are accessed from the front, above/below the jaw rails.
    for n in select('P00_'):screw(n,(1,0,0),engagement=6)
    front=select('P01_','P02_','P03_','P04_','L','J','C')
    take('Withdraw the complete gripper and feed the harness through the shaft',front,(1,0,0),210,True)

    screw('G07_2_Flat_engaging_grub_screw',(0,0,1),engagement=3,small=True)
    take('Remove input shaft collar',select('G06_'),(1,0,0),35)
    screw('G07_1_Flat_engaging_grub_screw',(0,0,1),engagement=3,small=True)
    input_unit=select('G03_','G05_','G07_0_')
    take('Withdraw input shaft with knob',input_unit,(-1,0,0),100,True)
    for n in select('H03_'):screw(n,(1,0,0),engagement=7)
    for side in (-1,1):
        ticks=tuple(n for n in select('M03_') if np.sign(parts[n].bounds.mean(0)[1])==side)
        half=select(f'H02_{side}_')+ticks+(select('G04_1_') if side==1 else ())
        take('Release split bearing cover '+str(side),half,(0,side,0),70,True)
    take('Slide the gear shell axially over the flange',select('H01_'),(1,0,0),140,True)
    for n in select('W06_'):screw(n,(0,0,1),engagement=5)
    for side in (1,-1):
        group=select(f'W05_{side}_')
        move('Separate rear clamp collar '+str(side),group,(0,0,side*25),.7)
        move('Clear the shaft end',group,(-65,0,0),.4)
        park('rear clamp collar',group)
    take('Remove rear thrust washer',select('W04_'),(-1,0,0),24)
    spindle=select('W01_','W02_','W03_','W07_','G01_','R02_')
    take('Withdraw spindle, gear and front bearing together',spindle,(1,0,0),100,True)
    take('Remove input pinion',select('G02_'),(1,0,0),45)
    take('Withdraw rear bearing as one cartridge',select('R01_'),(-1,0,0),35)
    take('Withdraw rear input bushing',select('G04_0_'),(-1,0,0),22)
    for n in select('B04_'):screw(n,(0,0,1),pitch=.7,engagement=6)
    take('Lift the empty pedestal from its mounting shoe',select('B03_'),(0,0,1),115,True)

    # Strip the spindle on the bench. Bearing cartridges stay intact.
    take('Remove rear gear spacer',select('W07_Rear_'),(-1,0,0),30)
    take('Slide the wrist gear off the keyed shaft',select('G01_'),(-1,0,0),55,True)
    take('Lift the parallel drive key from its keyway',select('W02_'),(0,1,0),12)
    take('Remove front gear spacer',select('W07_Front_'),(-1,0,0),55)
    take('Remove the complete front bearing over the free shaft end',select('R02_'),(-1,0,0),70)
    take('Remove front thrust washer',select('W03_'),(-1,0,0),70)
    screw('G07_0_Flat_engaging_grub_screw',(0,0,1),engagement=8,small=True)
    take('Slide the rotary knob off the input shaft',select('G05_'),(-1,0,0),20)

    # Gripper bench service: release axial restraints, withdraw guides, then
    # supports. Nuts are unthreaded only after carriages can leave their seats.
    screw('L04_Flat_engaging_knob_screw',(1,0,0),engagement=8,small=True)
    take('Remove gripper knob',select('L03_'),(0,1,0),15)
    screw('L07_Screw_collar_lock',(1,0,0),engagement=3,small=True)
    take('Remove lead screw axial collar',select('L06_'),(0,-1,0),15)
    for side in (-1,1):take('Remove screw thrust washer',select(f'L08_{side}_'),(0,side,0),15)
    for n in select('L05_'):screw(n,(1,0,0),engagement=6,small=True)
    for n in select('L01_'):take('Withdraw ground guide rod',(n,),(0,1,0),110,True)
    for side in (-1,1):
        for n in select(f'P03_{side}_'):screw(n,(1,0,0),engagement=4)
        take('Remove guide end support with its bushing',select(f'P02_{side}_',f'P04_{side}_'),(0,side,0),22)
    for side in (-1,1):
        for end in (-1,1):
            for n in select(f'J08_{side}_{end}_'):
                screw(n,(0,end,0),pitch=.35,engagement=3.6,small=True,bend=(0,0,1))
            take('Remove slotted nut retaining plate radially',select(f'J07_{side}_{end}_'),(1,0,0),12)
        jaw=select(f'J04_{side}_',f'J06_{side}_',f'J02_{side}_')
        take('Slide the bush-lined finger off its nut and screw',jaw,(0,side,0),85,True)
    for side in (-1,1):
        nut=select(f'J03_{side}_');center=bounds(nut).mean(0)
        # Both handed nuts require +rotation about +Y to move OUTWARD.
        # The negative-Y nut is left-handed, the positive-Y nut right-handed.
        move('Unthread '+('left' if side<0 else 'right')+' handed bronze nut',nut,(0,side*27,0),1.3,
             axis=(0,1,0),turns=9,pivot=center,
             note='Actual helical CAD, 3 mm lead, 9 turns for 27 mm translation.')
        take('Slide the disengaged nut off the plain journal',nut,(0,side,0),25)
    take('Lift out the opposed lead screw',select('L02_'),(1,0,0),35,True)
    # Every remaining service unit is already resting at its bench station.
    retained={
        'rear bearing cartridge':list(select('R01_')),
        'front bearing cartridge':list(select('R02_')),
        'palm and dressed harness':list(select('P01_','C')),
        'mounting shoe with bonded feet and markings':list(select('B01_','B02_','M02_','M04_')),
    }
    for side in (-1,1):
        retained[f'bonded and bush-lined finger {side}']=list(select(f'J04_{side}_',f'J06_{side}_',f'J02_{side}_'))
    plan=AssemblyProcess('ORBIT / reversible service sequence',moves,retained,
        list(select('B01_','B02_','M02_','M04_')),
        ['Manual support/placement is prescribed; gravity and hand forces are not simulated.',
         'The timeline is compressed for inspection; operation durations are not measured workshop labor times.',
         'Ball bearings, bonded pads, press-fitted bushings and dressed harness stay as service units.',
         'Fastener thread envelopes are nominal; lead screw and nut threads are actual helical CAD.',
         'Tool access envelopes establish approach clearance, not tightening torque or full wrench-swing qualification.',
         'Sampled nominal clearances do not qualify manufacturing tolerances, preload or production assembly.'])
    return plan.bind(assembly)
