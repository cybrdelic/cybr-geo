"""Bind constructive component mass data to the existing PhysicalModel owner.

No actuator, contact, spring or transmission behavior is inferred from a replay,
part name or optical material. A complete explicit physical tree is required.
"""
from .physical_model import PhysicalModel
from .native_construction import aggregate_mass_properties


class UnresolvedAssemblyPhysics(ValueError):
    def __init__(self,missing):
        self.missing=tuple(missing)
        super().__init__('Physical model unresolved: '+'; '.join(self.missing))


def bind_physical_model(assembly,physical_model,component_masses,body_mapping):
    """Rebind an authored physical tree to this exact component revision.

    body_mapping maps physical body names to component assembly body IDs.
    All components of each selected body need explicit mechanical mass records;
    diagnostic mesh estimates retain their provenance and do not become tests.
    Simulation and replay continue through the existing PhysicalModel pipeline.
    """
    if not isinstance(physical_model,PhysicalModel):
        raise UnresolvedAssemblyPhysics(['complete authored PhysicalModel required; recorded poses are not a model'])
    model=PhysicalModel.from_source(physical_model.to_source())
    if len(set(body_mapping.values()))!=len(body_mapping):raise ValueError('Physical bodies require one-to-one assembly binding')
    inverse={body:name for name,body in body_mapping.items()}
    parents={child:parent for parent in model.root.iter() for child in parent}
    for name,body in body_mapping.items():
        node=model.bodies.get(name)
        if node is None or body not in assembly.body_parents:raise ValueError('Unknown physical/assembly body binding')
        ancestor=parents.get(node)
        while ancestor is not None and ancestor.get('name') not in body_mapping:ancestor=parents.get(ancestor)
        physical_parent=None if ancestor is None else body_mapping[ancestor.get('name')]
        expected=assembly.body_parents[body]
        while expected is not None and expected not in inverse:expected=assembly.body_parents[expected]
        if physical_parent!=expected:raise ValueError('Physical tree differs from assembly body topology')
    rows={r['id']:r for r in component_masses}
    if len(rows)!=len(component_masses):raise ValueError('Duplicate mechanical component mass')
    records={}
    for name,body in body_mapping.items():
        if name not in model.bodies or body not in assembly.body_parents:raise ValueError('Unknown physical/assembly body binding')
        ids={pid for pid,p in assembly.parts.items() if p['body_index']==body}
        if ids-set(rows):raise UnresolvedAssemblyPhysics([f'missing component mass records for body {body}: {sorted(ids-set(rows))}'])
        components=[dict(name=str(pid),mass_kg=rows[pid]['mass_kg'],com_m=rows[pid]['com_m'],
            inertia_at_com_kg_m2=rows[pid]['inertia_com_kg_m2'],provenance=rows[pid].get('provenance')) for pid in sorted(ids)]
        records[name]=aggregate_mass_properties(components,status='Assembly-native/explicit estimated component mass; not qualified hardware')
    model.bind_inertias(records,{name:name for name in records})
    model.provenance.update(component_assembly_revision=assembly.revision,
        component_body_binding={name:str(body) for name,body in body_mapping.items()},
        reference_poses_not_used_as_physics=True,hardware_qualified=False)
    return model
