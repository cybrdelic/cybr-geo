"""Split, serviceable linear stage with a mechanically retained feed system."""
from kernel import *

def linear_stage(id='table',length=180,travel=60):
 m=Machine(id,'ROAM / screw-fed linear stage')
 m.requirements={'travel_mm':travel,'guide_diameter_mm':12,'guide_centers_mm':64,'feed':'M8 x 1.25 nominal purchased threaded rod','bearing':'LM12UU envelope 12 x 21 x 30 mm','purpose':'Positioning stage; no cutting-load rating'}
 m.sources=[{'url':'https://jp.misumi-ec.com/vona2/detail/221000091803/?HissuCode=LM12UU','supports':'THK LM12UU envelope: bore 12, diameter 21, length 30 mm. Internal bearing balls/seals are not invented.'},{'url':'https://www.emarketplace.in.skf.com/industrial/bearings?search=608-RSH','supports':'608 bearing envelope 8 x 22 x 7 mm; modeled as one purchased cartridge.'}]
 m.dof('feed','linear','X',-travel/2,travel/2)
 m.dof('screw','rotary','X',-travel/2,travel/2,(0,0,22),factor=-360/1.25)
 base=m.add('B01_Ribbed_base',box(length+40,140,8,(0,0,-4),4),1,role='Drilled aluminum base; datum face Z=0',make='Cut and drill plate stock')
 for x in [-length/2-10,length/2+10]:
  for y in [-55,55]:m.cut(base,cyl(3.3,10,(x,y,-9)))
 # Sculpted end pedestals meet the base, with separate, removable bearing caps.
 for side in [-1,1]:
  x=side*(length/2-10);n=f'E{side}_Guide_pedestal';q=box(20,100,44,(x,0,22),2)
  for y in [-32,32]:
   start=(-length/2+5,y,22) if side<0 else (length/2-21,y,22)
   q=q.cut(cyl(6.1,16,start,'X'))
  q=q.cut(cyl(4.2,22,(x-11,0,22),'X'))
  seat=(-length/2-.01,0,22) if side<0 else (length/2-7,0,22)
  q=q.cut(cyl(11.1,7.01,seat,'X'))
  m.add(n,q,0,role='Printed end support with blind guide seats and bearing shoulder',parent=base)
  for y in [-20,20]:m.bolt(f'F{side}_{y}_Foot_bolt',base,n,(x,y,-8),52,diam=4)
  cap=f'C{side}_Bearing_retainer';cx=side*(length/2+2)
  m.add(cap,box(4,100,44,(cx,0,22),1).cut(cyl(4.2,6,(cx-3,0,22),'X')),0,role='Separate bearing-retention plate; remove to service feed bearing',parent=n)
  bp=(-length/2,0,22) if side<0 else (length/2-7,0,22)
  m.add(f'R{side}_608_cartridge',ring(11,4,7,bp,'X'),2,role='608 cartridge envelope; no invented internal races',make='Purchase 608 bearing',parent=n)
  for y in [-43,43]:
   for z in [12,32]:
    start=(-length/2-4,y,z) if side<0 else (length/2-20,y,z)
    m.bolt(f'K{side}_{y}_{z}_Cap_bolt',cap if side<0 else n,n if side<0 else cap,start,24,'X')
 for y in [-32,32]:m.add(f'G{y}_Ground_guide',cyl(6,length-10,(-length/2+5,y,22),'X'),2,role='Ground metal rod retained between opposed blind seats',make=f'Cut 12 mm ground shaft to {length-10} mm',parent='E-1_Guide_pedestal')
 lower=box(60,100,16,(0,0,14),3);upper=box(60,100,22,(0,0,33),3)
 for y in [-32,32]:
  seat=cyl(10.6,30.2,(-15.1,y,22),'X');clear=cyl(6.2,62,(-31,y,22),'X')
  lower=lower.cut(seat).cut(clear);upper=upper.cut(seat).cut(clear)
  m.add(f'L{y}_LM12UU_cartridge',ring(10.5,6,30,(-15,y,22),'X'),2,['feed'],'Purchased linear cartridge in a split retained seat','Purchase LM12UU',f'G{y}_Ground_guide',fit=0)
 nut_cutter=prism(6,15.25,6.7,(-3.35,0,22),'X')
 clear=cyl(4.2,62,(-31,0,22),'X');lower=lower.cut(nut_cutter).cut(clear);upper=upper.cut(nut_cutter).cut(clear)
 m.add('S01_Lower_carriage',lower,0,['feed'],'Split carriage lower seat; bearings insert from above',parent='L-32_LM12UU_cartridge')
 m.add('S02_Upper_carriage',upper,0,['feed'],'Removable carriage cap with bearing and nut shoulders',parent='S01_Lower_carriage',fit=0)
 for x in [-22,22]:
  for y in [-43,43]:m.bolt(f'SB{x}_{y}', 'S01_Lower_carriage','S02_Upper_carriage',(x,y,6),38)
 nut=prism(6,13/math.cos(math.pi/6),6.5,(-3.25,0,22),'X').cut(cyl(3.3,8.5,(-4.25,0,22),'X'))
 m.add('N01_Captive_feed_nut',nut,2,['feed'],'M8 nut retained against rotation and translation by split shoulders','Purchase nominal M8 nut', 'S01_Lower_carriage')
 # The cylindrical thread envelope is declared, bounded, and requires engagement.
 screw=cyl(4,length+45,(-length/2-12,0,22),'X')
 for start,l in [(-length/2-12,7.1),(length/2+4.9,18)]:screw=screw.cut(box(l,8,12,(start+l/2,-7.8,22)))
 m.add('D01_Feed_screw',screw,2,['screw'],'M8 feed rod; nominal thread envelope, flats at collar and handwheel','Purchase rod; file two set-screw flats',parent='R-1_608_cartridge',fit=0)
 m.threads.append(dict(a='D01_Feed_screw',b='N01_Captive_feed_nut',mask=ring(4.001,3.299,6.502,(-3.251,0,22),'X'),motion=['feed'],min_volume=.1,description='Nominal M8 feed engagement; nut and rod flanks are not modeled'))
 for side in [-1,1]:
  wp=(-length/2-4.8,0,22) if side<0 else (length/2+4,0,22)
  m.add(f'T{side}_Thrust_washer',ring(8,4.2,.8,wp,'X'),2,role='Separate axial thrust washer against retainer face',make='Purchase washer',parent=f'C{side}_Bearing_retainer',fit=0)
  p=(-length/2-9,0,22) if side<0 else (length/2+4.9,0,22)
  q=ring(8,4.2,4.1,p,'X') if side<0 else handwheel(18,4.2,16,p,'X')
  name='D02_Stop_collar' if side<0 else 'D03_Feed_handwheel'
  m.add(name,q,2 if side<0 else 0,['screw'],'Flat-locking axial retainer' if side<0 else 'Fluted input wheel; retained by flat-engaging screw','Original collar / printed wheel',parent='D01_Feed_screw',fit=.21)
  x=p[0]+(2.05 if side<0 else 8);y=-8 if side<0 else -18
  m.socket(name+'_lock',name,(x,y,22),'Y',diam=3,length=-3.8-y)
  m.connect(name,f'T{side}_Thrust_washer','Axial stop',.11)
 m.service=[dict(part='S02_Upper_carriage',remove_first=[n for n in m.parts if n.startswith('SB')],axis='Z',distance_mm=45,step_mm=1,description='Remove the four through-fastener sets, then lift the complete split cap.')]
 m.notes=['A positioning stage is not a milling machine qualification.','Feed rod uses declared nominal M8 envelope; internal helical flanks are not detailed in this version.','Base and metal hardware are not printable. Guide/carriage/caps and input wheel are clear PETG.','Assemble bearing cartridges and feed nut between split carriage halves before joining end pedestals.']
 return m

if __name__=='__main__':
 m=linear_stage();m.export(ROOT/'outputs/roam-workshop/machines/table-stage')
 sys.stdout.flush();os._exit(0)
