import os,json,hashlib
import cadquery as cq
from model import OUT,T,pose_cad,Assembly,MAT,cad_part
from mechanism_lab.exporters import export_glb,export_step
from verify import load,audit
EXPLODE={'left_link_1_end':0,'left_link_2_root':35,'left_elbow_bush':65,'left_elbow_thrust':15,'left_elbow_top_spacer':110,'left_elbow_bottom_spacer':-25,'left_elbow_shoulder_screw':160,'left_elbow_locknut':-60}
def run():
 m,d=load();folder=OUT/'inspection';folder.mkdir(exist_ok=True)
 shapes={p['name']:p['shape'] for p in m.parts if p['name'] in EXPLODE};shapes['left_link_1_end']=pose_cad(shapes['left_link_1_end'],T(-280,0,-18))
 mats={p['name']:p['mat'] for p in m.parts}
 for label,cut in [('joint',False),('section',True)]:
  box=cq.Workplane('XY').box(1000,1000,1000).val().translate((0,500,0))
  a=Assembly('Elbow inspection '+label,[cad_part(n,s.cut(box) if cut else s,mats[n],tolerance=.035,angular=.08) for n,s in shapes.items()],MAT)
  export_glb(a,folder/(label+'.glb'));export_step(a,folder/(label+'.step'),individual=False)
 results=[{'percent':pct,**audit({n:s.translate((0,0,EXPLODE[n]*pct/100)) for n,s in shapes.items()})} for pct in range(0,101,10)]
 info={'source_step_sha256':hashlib.sha256((OUT/'cad/workstation.step').read_bytes()).hexdigest(),'note':'Eight elbow components at the shared shaft datum, straight alignment. Complete end fittings; tube and attachment screws excluded from this inspection selection. Exact Y>=0 material removal in section. Exploded offsets are inspected separations, not a simultaneous removal procedure.','explode':EXPLODE,'parts':{p['name']:{'volume_mm3':p['shape'].Volume(),'faces':len(p['shape'].Faces()),'solids':len(p['shape'].Solids())} for p in m.parts},'exploded_cases':results}
 (folder/'manifest.json').write_text(json.dumps(info,indent=2));print(json.dumps({'joint_parts':len(shapes),'passed':all(x['passed'] for x in results)}),flush=True)
 if not all(x['passed'] for x in results):raise ValueError('Inspection assembly overlaps')
 return info
if __name__=='__main__':
 try:run();os._exit(0)
 except BaseException:
  import traceback;traceback.print_exc();os._exit(1)
