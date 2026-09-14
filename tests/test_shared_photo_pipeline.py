from dataclasses import replace
import cadquery as cq
import numpy as np
from mechanism_lab.core import View
from mechanism_lab.cli import parser
from mechanism_lab.photoreal import resolve_studio
from mechanism_lab.finish_render import tonemap,save_png
from mechanism_lab.render_profiles import V9
from cybrgeo.core import Assembly,Material,from_shape
from cybrgeo.photoreal import adapt
from PIL import Image


def test_public_adapter_preserves_optics_and_explicit_materials():
    shape=cq.Workplane('XY').box(10,20,30).val()
    material=Material('coated',(.2,.3,.4),1.,.27,ior=1.6,coat=.4,anisotropy=.7,microfinish='brushed')
    a=Assembly('part',[from_shape('body',shape,metadata={'finish_axis':[0,1,0]})],[material],cad={'body':shape})
    b=adapt(a)
    assert b.materials[0].as_dict()['anisotropy']==.7
    assert b.materials[0].ior==1.6 and b.parts[0].finish_axis==(0,1,0)
    assert np.array_equal(a.parts[0].vertices,b.parts[0].vertices)
    assert b.parts[0].cad is shape
    assert b.metadata['truth_intent']=='inspection'


def test_legacy_native_studio_does_not_follow_camera_or_removing_parts():
    # The in-house native renderer remains a supported explicit backend even
    # though V9 is now the public default.
    a=adapt(Assembly('one',[from_shape('body',cq.Workplane('XY').box(10,20,30).val())]))
    fixed=resolve_studio(a,View())
    changed=resolve_studio(a,replace(fixed,az=120,target=(1000,2000,3000)))
    assert fixed.studio_target==changed.studio_target
    assert fixed.studio_az==changed.studio_az
    assert fixed.floor_z_mm==-15 and changed.floor_z_mm==-15
    assert fixed.studio_scale==.15


def test_new_render_video_and_film_defaults_use_v9():
    assert parser().parse_args(['render','model.py']).renderer=='v9'
    assert parser().parse_args(['video','model.py']).renderer=='v9'
    assert parser().parse_args(['film','model.py']).renderer=='v9'
    assert parser().parse_args(['render','model.py']).size==V9.still_size
    # Native studio defaults remain unchanged for users explicitly selecting it.
    assert View().studio_style=='product' and View().floor_gap_mm==0


def test_legacy_neutral_transfer_is_finite_monotonic_and_achromatic():
    ramp=np.repeat(np.linspace(0,100,10000)[:,None],3,axis=1)
    mapped=tonemap(ramp)
    assert mapped.dtype==np.uint8
    assert (np.diff(mapped[:,0].astype(int))>=0).all()
    assert (mapped[:,0]==mapped[:,1]).all() and mapped[0,0]==0
    assert tuple(tonemap(np.array([[.2,.4,.6]]))[0])==(124,170,203)


def test_complete_png_round_trip(tmp_path):
    values=np.random.default_rng(25).integers(0,256,(43,57,3),dtype=np.uint8)
    out=tmp_path/'test.png';save_png(Image.fromarray(values),out)
    with Image.open(out) as im:assert np.array_equal(np.asarray(im),values)


def test_native_photographic_section_is_real_capped_geometry():
    import trimesh
    from mechanism_lab.photoreal import prepare_view_geometry
    source=adapt(Assembly('cube',[from_shape('body',cq.Workplane('XY').box(10,20,30).val())]))
    clipped,view=prepare_view_geometry(source,View(section=(0,1,0)))
    part=clipped.parts[0]
    mesh=trimesh.Trimesh(part.vertices,part.faces,process=True)
    assert mesh.is_watertight and np.isclose(abs(mesh.volume),3000)
    assert view.section is None and part.cad is None
    assert np.isfinite(part.normals).all()
    assert np.isclose(np.ptp(part.vertices[:,1]),10)
