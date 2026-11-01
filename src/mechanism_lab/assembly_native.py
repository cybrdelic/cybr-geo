"""Native CAD lowering from the same component records and poses as rendering."""
from pathlib import Path
import numpy as np
from .shape_dag import ShapeEvaluator,from_tree
from .core import pose_cad

class NativeAssembly:
    def __init__(self,assembly,root):
        self.assembly=assembly;self.root=Path(root);self.shapes={};self.maximum_rotation_projection=0.

    def local_shape(self,pid):
        if pid not in self.shapes:
            import cadquery as cq
            record=self.assembly.parts[pid]
            if record.get('native_units')!='mm':raise ValueError('Native component requires explicit millimetre units')
            if record.get('native_path'):
                self.shapes[pid]=cq.Shape.importBrep(str(self.root/record['native_path']))
            elif record.get('shape_tree'):
                self.shapes[pid]=ShapeEvaluator(from_tree(record['shape_tree'])).evaluate().clean()
            else:raise ValueError(f'Component {pid} has no native geometry; no mesh substitution allowed')
        shape=self.shapes[pid]
        if not shape.isValid() or not shape.Solids() or shape.Volume()<=0:raise ValueError(f'Invalid native component {pid}')
        return shape.copy()

    def placed_shape(self,pid,frame,*,relative_body=None,rack_coordinate_m=None):
        transform=(self.assembly.pose(frame,pid,rack_coordinate_m=rack_coordinate_m) if relative_body is None else
            self.assembly.local_pose(frame,pid,relative_body,rack_coordinate_m=rack_coordinate_m))
        transform=transform.copy()
        # Recorded Blender float32 matrices contain tiny scale/shear rounding.
        # CAD requires a proper rigid rotation. Project only at this boundary;
        # preserve original reference matrices in the canonical document.
        u,_,vh=np.linalg.svd(transform[:3,:3]);rotation=u@vh
        error=float(np.max(np.abs(rotation-transform[:3,:3])))
        if np.linalg.det(rotation)<0 or error>2e-6:raise ValueError('Native pose is not near-rigid')
        self.maximum_rotation_projection=max(self.maximum_rotation_projection,error)
        transform[:3,:3]=rotation;transform[:3,3]*=1000
        return pose_cad(self.local_shape(pid),transform)

    def export_step(self,path,ids,frame,*,relative_body=None,rack_coordinate_m=None,purpose='manufacturing'):
        self.assembly.require_export(purpose)
        import cadquery as cq
        model=cq.Assembly(name=Path(path).stem)
        for pid in ids:model.add(self.placed_shape(pid,frame,relative_body=relative_body,rack_coordinate_m=rack_coordinate_m),name=f'component_{pid:03}')
        model.export(str(path))
        return {'file':Path(path).name,'component_ids':list(ids),'components':len(ids),
                'reference_motion_only':True,'hardware_qualified':False}


def mesh_mass_record(record,path,*,density_kg_m3,mass_override_kg=None):
    """Explicitly estimated mass from source triangles; never a strength record."""
    import trimesh
    with np.load(path,allow_pickle=False) as data:
        mesh=trimesh.Trimesh(data['vertices'],data['faces'],process=False)
    properties=mesh.mass_properties
    mass=abs(properties['volume'])*density_kg_m3
    inertia=np.asarray(properties['inertia'])*density_kg_m3
    if mass_override_kg is not None:
        inertia*=mass_override_kg/mass;mass=mass_override_kg
    return dict(id=record['id'],body_index=record['body_index'],name=record['name'],mass_kg=mass,
        com_m=np.asarray(properties['center_mass']).tolist(),inertia_com_kg_m2=inertia.tolist(),
        provenance='source_mesh_density_estimate',density_kg_m3=density_kg_m3,hardware_qualified=False)
