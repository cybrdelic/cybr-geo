"""Check subgrid appearance contracts, distinct from solver validation."""
from dataclasses import asdict
import numpy as np
import pytest
import trimesh

from cybr_terrain.model import Config, Layer, LAYERS, make_terrain
from cybr_terrain.materials import make_atlas
from cybr_terrain.geometry import build_geometry


def test_loaded_layer_atlas_is_reproducible_and_wet_optics_consistent(tmp_path):
    values=asdict(LAYERS['soil'])
    values['color']=list(values['color']);values['fractions']=list(values['fractions'])
    layer=Layer(**values)
    dry=make_atlas(layer,0,64)
    repeated=make_atlas(layer,0,64)
    wet=make_atlas(layer,1,64).save(tmp_path,'wet')
    assert np.array_equal(dry.albedo,repeated.albedo)
    assert np.array_equal(dry.height,repeated.height)
    assert np.all(wet.albedo<dry.albedo)
    assert np.all(wet.roughness<dry.roughness)
    assert dry.roughness.std()>.02
    # The coarse test atlas retains fine relief without the former mm-high
    # tiled clod network. Real aggregate silhouettes belong to geometry.
    assert .01<np.max(np.linalg.norm(dry.normal[:,:,:2],axis=2))<.20
    assert dry.height.std()<.0008
    assert np.allclose(np.linalg.norm(dry.normal,axis=2),1,atol=2e-6)
    assert wet.roughness_pfm.is_file()


@pytest.fixture(scope='module')
def detailed_geometry(tmp_path_factory):
    directory=tmp_path_factory.mktemp('detail-geometry')
    state=make_terrain(Config(preset='soil-profile',grid=17,extent=10,duration=1))
    # A uniform one-millimetre bulk sand dusting must retain substrate
    # material everywhere while contributing some optical pigment coverage.
    state.loose[1]=.00058
    height=state.height.copy();solids=state.solid_volumes().copy()
    geometry=build_geometry(state,directory,subdivision=2,stones=7)
    assert np.array_equal(state.height,height)
    assert np.array_equal(state.solid_volumes(),solids)
    return geometry,directory


def test_thin_sediment_dusting_retains_underlying_lithology(detailed_geometry):
    geometry,_=detailed_geometry
    meta=geometry.assembly.metadata
    assert meta['deposit_full_coverage_min_bulk_m']==.010
    assert meta['deposit_full_coverage_fraction']==0
    assert 0<meta['deposit_mean_optical_coverage']<.11
    deposited_material_start=len(geometry.state.layers)*4
    surface=[p for p in geometry.assembly.parts if p.group=='surface']
    assert surface
    assert all(p.material<deposited_material_start for p in surface)


def test_fracture_planes_and_centimetre_aggregate_budget(detailed_geometry):
    geometry,_=detailed_geometry
    meta=geometry.assembly.metadata
    assert meta['fractured_chunks']==7
    assert meta['soil_aggregates']==315
    assert meta['aggregate_radius_m']==[.004,.040]
    chunks=[p for p in geometry.assembly.parts if p.name.startswith('fractured_chunks_')]
    assert chunks
    for part in chunks:
        v=part.vertices;f=part.faces
        face_normals=np.cross(v[f[:,1]]-v[f[:,0]],v[f[:,2]]-v[f[:,0]])
        face_normals/=np.linalg.norm(face_normals,axis=1,keepdims=True)
        assert np.allclose(part.normals[f[:,0]],face_normals,atol=2e-6)
        assert np.allclose(part.normals[f[:,0]],part.normals[f[:,1]],atol=2e-6)
    assert sum(len(p.faces) for p in geometry.assembly.parts)<20000


def test_glb_pigment_rebasing_preserves_linear_products(detailed_geometry):
    geometry,directory=detailed_geometry
    path=geometry.export_glb(directory/'detail.glb')
    loaded=trimesh.load(path,force='scene')
    for part in geometry.assembly.parts:
        if part.material not in geometry.atlases:continue
        atlas=geometry.atlases[part.material]
        tint=geometry.attributes[part.name].tint
        assert np.all(atlas.albedo.max(axis=(0,1))*tint.max(axis=0)<1)
        material=loaded.geometry[part.name].visual.material
        # The exported PNG and COLOR_0 represent the same multiplication.
        pixel=np.asarray(material.baseColorTexture)[0,0,:3]/255
        linear=np.where(pixel<=.04045,pixel/12.92,((pixel+.055)/1.055)**2.4)
        colors=np.asarray(loaded.geometry[part.name].visual.vertex_attributes['color'])[:,:3]
        if colors.max()>1:colors=colors/255
        expected=atlas.albedo[0,0]*tint[0]
        assert np.allclose(linear*colors[0],expected,atol=.005)
