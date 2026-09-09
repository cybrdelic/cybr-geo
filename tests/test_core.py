from dataclasses import replace
import json
import numpy as np
import pytest
from mechanism_lab.core import validate,save_cache,load_cache,axis_pose,pose_cad
from mechanism_lab.exporters import export_glb,export_step,export_stls


def test_independent_recipe_has_real_solids(flange):
    report=validate(flange,expensive=True)
    assert report['parts']==2
    assert all(row['analytic_valid'] and row['watertight'] for row in report['checks'])


def test_cache_preserves_mesh_bytes(flange,tmp_path):
    save_cache(flange,tmp_path)
    restored=load_cache(tmp_path)
    for before,after in zip(flange.parts,restored.parts):
        for key in ['vertices','faces','normals']:
            assert np.array_equal(getattr(before,key),getattr(after,key))
        assert after.cad is None  # cache never impersonates analytic geometry


def test_rejects_duplicate_names(flange):
    with pytest.raises(ValueError,match='Duplicate'):
        validate(replace(flange,parts=[flange.parts[0],flange.parts[0]]))


def test_rejects_nonfinite_and_indices(flange):
    p=flange.parts[0]
    vertices=p.vertices.copy();vertices[0,0]=float('nan')
    with pytest.raises(ValueError,match='nonfinite'):
        validate(replace(flange,parts=[replace(p,vertices=vertices)]))
    faces=p.faces.copy();faces[0,0]=len(vertices)+1
    with pytest.raises(ValueError,match='face indices'):
        validate(replace(flange,parts=[replace(p,faces=faces)]))


def test_native_cad_pose_matches_mesh_pose(flange):
    import cadquery as cq
    shape=flange.parts[0].cad
    T=axis_pose(.72,(13,8,-2),(3,9,1),.4)
    moved=pose_cad(shape,T)
    before=np.array(shape.Center().toTuple())
    after=np.array(moved.Center().toTuple())
    assert np.max(abs(after-(T[:3,:3]@before+T[:3,3])))<1e-8
    assert moved.isValid()


def test_step_and_stl_are_real_and_units_explicit(flange,tmp_path):
    import cadquery as cq
    import trimesh
    report=export_step(flange,tmp_path/'flange_analytic.step')
    assert len(report['analytic_components'])==2
    assert len(cq.importers.importStep(str(tmp_path/'flange_analytic.step')).val().Solids())==2
    export_stls(flange,tmp_path/'stl')
    mesh=trimesh.load_mesh(tmp_path/'stl'/f'{flange.parts[0].name}.stl')
    assert np.allclose(mesh.bounds,flange.parts[0].bounds,atol=1e-5)
    assert 'millimetres' in (tmp_path/'stl/UNITS.txt').read_text()


def test_static_glb_roundtrip_metres_yup(flange,tmp_path):
    import trimesh
    from mechanism_lab.exporters import NATIVE_TO_GLTF
    path=export_glb(flange,tmp_path/'flange.glb')
    scene=trimesh.load_scene(path,process=False)
    for p in flange.parts:
        T,geometry=scene.graph[p.name]
        assert np.max(abs(T-NATIVE_TO_GLTF@flange.pose(p)))<1e-12


def test_geometry_import_unit_contract(flange,tmp_path):
    from mechanism_lab.importers import import_geometry
    path=export_glb(flange,tmp_path/'flange.glb')
    imported=import_geometry(path)
    assert np.allclose(imported.bounds,flange.bounds,atol=1e-4)
    export_stls(flange,tmp_path/'stl')
    stl=tmp_path/'stl'/f'{flange.parts[0].name}.stl'
    with pytest.raises(ValueError,match='require units'):import_geometry(stl)
    assert np.allclose(import_geometry(stl,units='mm').bounds,flange.parts[0].bounds,atol=1e-5)


def test_plugin_and_cache_invalidation(tmp_path):
    from mechanism_lab.registry import factory,fingerprint
    recipe=tmp_path/'recipe.py'
    recipe.write_text('from mechanism_lab.models.example import build\n')
    build,motion=factory(str(recipe));assert len(build().parts)==2 and motion is None
    first=fingerprint(str(recipe));recipe.write_text(recipe.read_text()+'\n# recipe edit\n')
    assert first!=fingerprint(str(recipe))
