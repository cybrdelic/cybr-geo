"""Manually driven precision drilling head on a reversible vertical feed.

Purchased collet-extension body is a declared dimensional envelope; unsupported
vendor internals are not invented. This is not a powered metal drill-press rating.
"""
from vertical import *
def build():
 m,plate,attachment=upright('drill','ROAM / manual precision drill press');mo=['V_feed'];spin=['spindle','V_feed'];cx=-110
 m.dof('spindle','rotary','Z',0,360,(cx,0,0));m.dofs['spindle']['control']='spindle'
 # Sculpted connected bridge and hollow two-bearing quill housing.
 m.fuse(plate,box(45,46,20,(-82.5,0,120),3))
 housing=cyl(22,57,(cx,0,105)).cut(cyl(4.2,59,(cx,0,104))).cut(cyl(11.1,7.01,(cx,0,104.99))).cut(cyl(11.1,7.01,(cx,0,155)))
 m.fuse(plate,housing)
 # Finish the bearing and shaft bores through the completed joined body.
 m.cut(plate,cyl(4.2,59,(cx,0,104)))
 for z in [105,155]:m.cut(plate,cyl(11.1,7.01,(cx,0,z)))
 for z in [105,155]:m.add(f'Q01_{z}_608',ring(11,4,7,(cx,0,z)),2,mo,'Purchased 608 cartridge in a shouldered seat','Purchase 608 bearing',plate)
 caps=[]
 for z in [101,162]:
  name=f'Q02_{z}_End_cap';m.add(name,box(44,44,4,(cx,0,z+2),3).cut(cyl(6.2,6,(cx,0,z-1))),0,mo,'Removable quill bearing retainer','Print',plate,fit=0);caps.append(name)
 for x in [-12,12]:
  for y in [-12,12]:
   m.cut(plate,cyl(2.2,67,(cx+x,y,100)));m.bolt(f'Q03_{x}_{y}_Tie',caps[0],caps[1],(cx+x,y,101),65)
 # Somma's published A/B/C/D envelope: 8 mm shank, 100 total, 80 shank,
 # 12 mm nose. The collet cavity is an interface for the selected 3 mm tool.
 shank=cyl(4,80,(cx,0,100));nose=cyl(6,20,(cx,0,80));tool=shank.fuse(nose).cut(cyl(1.5,22,(cx,0,79)))
 tool=tool.cut(box(12,8,15,(cx,-7.8,173.5)))
 m.add('Q10_Collet_extension_envelope',tool,2,spin,'Somma SS8ER8100 dimensional envelope; collet internals omitted, 3 mm tool interface selected','Purchase extension/collet; validate bearing fits and file collar flats', 'Q01_105_608',fit=0)
 m.sources.append({'url':'https://www.sommatool.com/catalog/collets/straight%20shank%20er%20collet%20chuck%20extensions.asp','supports':'SS8ER8100: 8 mm shank, 100 mm overall, 80 mm shank dimension C, 12 mm nose D. Outer envelope only; modified flat and installed-bearing suitability require validation.'})
 for z in [100,162]:m.add(f'Q11_{z}_Axial_spacer',ring(6,4.2,5,(cx,0,z)),2,spin,'Spacer transfers spindle shoulder/retainer reaction to bearing face','Cut spacer; verify actual bearing inner-race contact', 'Q10_Collet_extension_envelope',fit=.21)
 m.add('Q12_Spindle_collar',ring(8,4.2,6,(cx,0,167)),2,spin,'Flat-locking axial stop for spindle','Original collar','Q10_Collet_extension_envelope',fit=.21)
 m.socket('Q13_Collar_lock','Q12_Spindle_collar',(cx,-8,170),'Y',diam=3,length=4.2)
 m.connect('Q12_Spindle_collar','Q11_162_Axial_spacer','Axial stop',0)
 m.add('Q14_Spindle_wheel',handwheel(18,4.2,12,(cx,0,173)),0,spin,'Manual spindle input; no motor is implied','Print','Q10_Collet_extension_envelope',fit=.21)
 m.socket('Q15_Wheel_lock','Q14_Spindle_wheel',(cx,-18,177),'Y',diam=3,length=14.2)
 # Representative cutting-tool envelope: no imaginary vendor flute design.
 m.add('Q20_3mm_tool_envelope',cyl(1.5,42,(cx,0,50)),2,spin,'3 mm cutting-tool envelope engaged 12 mm into the selected collet; flutes omitted','Purchase tool appropriate to material','Q10_Collet_extension_envelope',fit=0)
 workpiece(m,cx)
 tool_names=[n for n in m.parts if n.startswith(('H01','Q'))]
 m.service=[dict(parts=tool_names,remove_first=attachment,axis='X',distance_mm=-120,step_mm=2,description='Support and remove the complete head from the vertical carriage after its four fastener sets are removed.'),dict(part='V_S02_Upper_carriage',remove_first=tool_names+attachment+[n for n in m.parts if n.startswith('V_SB')],axis='X',distance_mm=-45,step_mm=1,description='With the complete head removed, unbolt and lift the split carriage cap forward.')]
 m.requirements.update(tool_capacity_mm=[.5,5],demonstration_tool_mm=3,operation='Manual spindle rotation and independent vertical feed into an existing pilot',contact_scope='Tool-envelope insertion, not modeled cutting or measured spindle performance')
 m.notes+=['The spindle is hand-driven. This avoids drawing an unspecified power drill or pretending a motor has been selected.','Supplier collet-extension dimensions define the envelope; using its shank as a spindle is an original adaptation requiring fit/runout testing.','No powered spindle, cutting load, feed force, workholding force or thermal qualification has been established.','The entire quill/head is supported as one service unit during removal.']
 return m
if __name__=='__main__':
 try:build().export(ROOT/'outputs/roam-workshop/machines/drill')
 except Exception:
  import traceback;traceback.print_exc();sys.stderr.flush();os._exit(1)
 sys.stdout.flush();os._exit(0)
