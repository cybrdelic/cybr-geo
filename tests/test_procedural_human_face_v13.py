"""Regression tests for CYBR GEO v13 semantic-cage human face."""
from pathlib import Path
import builtins, sys
import numpy as np
import pytest

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'examples'),str(ROOT/'tools'),str(ROOT/'src')]

from procedural_human_face_v13 import (
    build,HumanAnatomy,FaceParameters,SKIN,CORNEA
)


@pytest.fixture(scope='module')
def neutral():
    return build(FaceParameters(),quality='preview',hair=False)


def test_v13_builds_without_file_reads(monkeypatch,neutral):
    def deny(*args,**kwargs):
        raise AssertionError('v13 procedural anatomy attempted file access')
    with monkeypatch.context() as m:
        m.setattr(builtins,'open',deny)
        m.setattr(Path,'open',deny)
        again=build(FaceParameters(),quality='preview',hair=False)
    assert again.metadata['input_assets']==[]
    assert again.metadata['scan_used'] is False
    assert again.metadata['imported_anatomy_mesh'] is False
    assert again.metadata['learned_face_model'] is False
    assert len(again.parts)==len(neutral.parts)
    for a,b in zip(again.parts,neutral.parts):
        assert a.name==b.name
        assert np.array_equal(a.vertices,b.vertices)
        assert np.array_equal(a.faces,b.faces)


def test_semantic_cage_is_real_and_boundary_matches_skull():
    anatomy=HumanAnatomy(FaceParameters())
    cage=anatomy.cage
    assert cage.control_offsets.shape==(17,13)
    assert cage.control_offsets.size==221
    assert len(cage.landmarks)>=25
    assert np.allclose(cage.control_offsets[:,0],0)
    assert np.allclose(cage.control_offsets[:,-1],0)
    assert np.allclose(cage.control_offsets[0],0)
    assert np.allclose(cage.control_offsets[-1],0)

    # Landmark RBF solves its authored macro targets to close tolerance.
    predicted=cage.macro_offset(cage.landmarks[:,0],cage.landmarks[:,1])
    assert np.max(np.abs(predicted-cage.landmarks[:,2]))<.55

    face=cage.sample_patch('preview')
    rows,cols=240,241
    vv=face.vertices.reshape(rows,cols,3)
    for edge in (vv[:,0],vv[:,-1]):
        shell=anatomy.skull.front_shell(edge[:,0],edge[:,2])
        assert np.max(np.abs(edge[:,1]-shell))<1e-5


def test_v13_geometry_is_nondegenerate_and_semantic(neutral):
    names={p.name for p in neutral.parts}
    assert 'Semantic_face_subdivision_surface' in names
    assert 'Procedural_cranium_mandible_neck_shoulders' in names
    face=next(p for p in neutral.parts if p.name=='Semantic_face_subdivision_surface')
    assert face.material==SKIN
    assert 'semantic-face-surface' in face.tags
    assert len(face.faces)>50000

    for p in neutral.parts:
        assert np.isfinite(p.vertices).all(),p.name
        assert np.isfinite(p.normals).all(),p.name
        assert p.faces.min()>=0 and p.faces.max()<len(p.vertices),p.name
        tri=p.vertices[p.faces]
        area=np.linalg.norm(np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]),axis=1)
        assert np.all(area>1e-10),p.name
        used=np.unique(p.faces)
        assert np.allclose(np.linalg.norm(p.normals[used],axis=1),1,atol=2e-5),p.name


def test_anthropometric_parameters_change_real_cage_shape():
    narrow=HumanAnatomy(FaceParameters(bizygomatic_width=130,bigonial_width=98))
    wide=HumanAnatomy(FaceParameters(bizygomatic_width=150,bigonial_width=120))
    a=narrow.cage.sample_patch('preview')
    b=wide.cage.sample_patch('preview')
    assert np.ptp(b.vertices[:,0])>np.ptp(a.vertices[:,0])+12
    # The change must not be a uniform XYZ scale.
    da=np.ptp(a.vertices,axis=0)
    db=np.ptp(b.vertices,axis=0)
    ratios=db/np.maximum(da,1e-9)
    assert np.ptp(ratios)>.03


def test_eye_and_nostril_apertures_hit_real_procedural_geometry():
    import mitsuba as mi
    mi.set_variant('llvm_ad_rgb')
    from render_procedural_face_v13 import mesh_shape

    material=mi.load_dict({'type':'diffuse'})
    for closure in (0.,1.):
        p=FaceParameters(eyelid_closure=closure)
        anatomy=HumanAnatomy(p)
        assembly=build(p,quality='preview',hair=False)
        scene_dict={'type':'scene'}
        for part in assembly.parts:
            scene_dict[part.name]=mesh_shape(part,material)
        scene=mi.load_dict(scene_dict)

        for side in (-1,1):
            c=anatomy.eyes.center(side)
            ray=mi.Ray3f(mi.Point3f(float(c[0]),-160.,float(c[2])),mi.Vector3f(0,1,0))
            hit=scene.ray_intersect(ray)
            assert bool(np.asarray(hit.is_valid()).ravel()[0])
            name=hit.shape[0].id()
            if closure<.995:
                assert ('corneal_tear_surface' in name or 'sclera_globe' in name or 'iris' in name),name
            else:
                assert name=='Semantic_face_subdivision_surface',name

        cx,cz=anatomy.nose.nostril_center(1)
        hit=scene.ray_intersect(mi.Ray3f(mi.Point3f(float(cx),-160.,float(cz)),mi.Vector3f(0,1,0)))
        assert bool(np.asarray(hit.is_valid()).ravel()[0])
        assert 'nasal_vestibule' in hit.shape[0].id(),hit.shape[0].id()


def test_metadata_enforces_no_scan_no_learned_identity(neutral):
    m=neutral.metadata
    assert m['topology']=='semantic-control-cage-subdivision'
    assert m['scan_used'] is False
    assert m['imported_anatomy_mesh'] is False
    assert m['photographic_skin_textures'] is False
    assert m['learned_face_model'] is False
    assert m['image_generation'] is False
    assert m['input_assets']==[]
    assert m['cage_control_vertices']==221
    assert m['semantic_landmark_count']>=25
