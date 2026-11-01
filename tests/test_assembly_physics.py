"""One existing-PhysicalModel bridge contract, not a robot simulation claim."""
import numpy as np
import pytest
from mechanism_lab.assembly_model import ComponentAssembly
from mechanism_lab.assembly_physics import bind_physical_model,UnresolvedAssemblyPhysics
from mechanism_lab.physical_model import PhysicalModel

def test_component_mass_binds_existing_physical_owner_and_missing_inputs_fail():
    assembly=ComponentAssembly([dict(id=1,body_index='link',name='solid')],{'link':None},{'0':{'link':np.eye(4)}})
    physical=PhysicalModel.from_xml('<mujoco><worldbody><body name="link"><joint name="hinge" type="hinge" axis="0 0 1"/></body></worldbody></mujoco>')
    mass=[dict(id=1,body_index='link',mass_kg=2.,com_m=[.1,0,0],inertia_com_kg_m2=np.diag([.02,.03,.04]).tolist(),provenance='explicit test fixture')]
    bound=bind_physical_model(assembly,physical,mass,{'link':'link'})
    assert bound.mass_records['link']['mass_kg']==2.
    assert bound.provenance['component_assembly_revision']==assembly.revision
    assert physical.mass_records=={}
    with pytest.raises(UnresolvedAssemblyPhysics,match='not a model'):bind_physical_model(assembly,None,mass,{'link':'link'})
    with pytest.raises(UnresolvedAssemblyPhysics,match='missing component mass'):bind_physical_model(assembly,physical,[],{'link':'link'})
