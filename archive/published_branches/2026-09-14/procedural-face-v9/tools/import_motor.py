from pathlib import Path
from cybrgeo.cad import import_step
import json,time
root=Path(__file__).resolve().parents[1]
t=time.time();a=import_step(root/'references/neo_vortex/NEO_Vortex_SPARK_Flex_8mm.STEP',axis='motor-x')
a.name='REV_NEO_Vortex_SPARK_Flex_8mm'
a.save(root/'examples/neo_vortex/geometry');a.export_glb(root/'examples/neo_vortex/geometry/motor_complete.glb')
(root/'examples/neo_vortex/geometry/validation.json').write_text(json.dumps(a.validate(),indent=2))
print('DONE',len(a.parts),time.time()-t,flush=True)
for p in a.parts:print(p.name,p.bounds.round(2).tolist(),p.material,p.metadata,flush=True)
