"""Continuous geological contacts must survive shader and export boundaries."""
import json
import struct

import numpy as np
import pytest

from cybr_terrain.geometry import build_geometry, grid_faces
from cybr_terrain.materials import Atlas, make_boundary_atlas
from cybr_terrain.model import Config, LAYERS, make_terrain
from cybr_terrain import rendering


def reference_repeat(pixels, u, v):
    """Four-texel reference, including negative UVs and wrapping seams."""
    h, w = pixels.shape[:2]
    x, y = u * w - .5, v * h - .5
    ix, iy = int(np.floor(x)), int(np.floor(y))
    fx, fy = x - ix, y - iy
    return ((1-fx)*(1-fy)*pixels[iy % h, ix % w]
            + fx*(1-fy)*pixels[iy % h, (ix+1) % w]
            + (1-fx)*fy*pixels[(iy+1) % h, ix % w]
            + fx*fy*pixels[(iy+1) % h, (ix+1) % w])


def test_partial_coating_matches_interior_microtexture_at_outer_halo():
    layer = LAYERS['soil']
    pattern = np.array([[.72, .98], [.84, 1.06]], np.float32)
    albedo = pattern[..., None] * np.asarray(layer.color)
    normal = np.broadcast_to(np.array([.12, -.08, np.sqrt(1-.12**2-.08**2)]), (2, 2, 3)).copy()
    rough = np.array([[.66, .81], [.73, .90]], np.float32)
    source = Atlas(layer, 0, albedo, normal, rough, np.zeros((2, 2)))
    # A 50% transported pigment coating must retain the bed's microtexture.
    target = .5*np.asarray(layer.color) + .5*np.array([.49, .35, .20])
    n, size = 9, 64
    wet = np.full((n, n), .31)
    pigment = np.broadcast_to(target, (n, n, 3)).copy()
    atlas = make_boundary_atlas({0: source}, {0: np.ones((n, n))}, wet, pigment, 1.25, size)
    assert atlas.repeat is False
    for row, col in [(0, 0), (13, 31), (63, 63)]:
        u, v = (col+.5)/size-.5, (row+.5)/size-.5
        expected = reference_repeat(albedo, u, v) * np.clip(target/layer.color, .30, 3.5) * (1-.48*.31)
        assert np.allclose(atlas.albedo[row, col], expected, atol=2e-7)
        assert np.allclose(atlas.normal[row, col], normal[0, 0], atol=2e-6)
        assert atlas.roughness[row, col] == pytest.approx(reference_repeat(rough, u, v), abs=2e-7)


@pytest.fixture(scope='module')
def contact_geometry(tmp_path_factory):
    folder = tmp_path_factory.mktemp('oblique-contact')
    state = make_terrain(Config(preset='soil-profile', grid=17, extent=10, duration=0))
    yy, xx = np.mgrid[:17, :17]
    # An oblique exposed contact cannot align with grid rows or columns.
    state.thickness[-1] = np.maximum(.021*(xx+.63*yy-12.7), 0)
    before = {name: getattr(state, name).copy() for name in
              ('thickness', 'loose', 'water', 'sediment', 'foundation', 'height')}
    geometry = build_geometry(state, folder, subdivision=2, stones=0, contact_resolution=128)
    for name, original in before.items():
        assert np.array_equal(getattr(state, name), original)
    return geometry, folder


def test_oblique_contact_keeps_surface_faces_and_shared_world_mapping(contact_geometry):
    geometry, _ = contact_geometry
    n = 33
    expected = {tuple(sorted(map(int, face))) for face in grid_faces(n)}
    found = []
    contact = next(p for p in geometry.assembly.parts if p.name == 'surface_contact')
    for part in geometry.assembly.parts:
        if part.group != 'surface': continue
        xy = part.vertices[:, :2]*.001
        ij = np.rint((xy/10+.5)*(n-1)).astype(int)
        indices = ij[:, 1]*n+ij[:, 0]
        found.extend(tuple(sorted(map(int, face))) for face in indices[part.faces])
    assert len(found) == len(expected) and set(found) == expected
    assert len(contact.faces) == geometry.assembly.metadata['surface_contact_faces'] > 0
    a = geometry.attributes[contact.name]
    assert np.allclose(a.uv, contact.vertices[:, :2]*.0001+.5, atol=1e-7)
    atlas = geometry.atlases[contact.material]
    assert atlas.repeat is False
    assert np.isfinite(atlas.albedo).all() and atlas.albedo.max() < 1
    assert np.allclose(np.linalg.norm(atlas.normal, axis=-1), 1, atol=2e-5)


def test_glb_clamps_all_contact_channels(contact_geometry):
    geometry, folder = contact_geometry
    data = geometry.export_glb(folder/'contacts.glb').read_bytes()
    length, kind = struct.unpack_from('<II', data, 12)
    assert kind == 0x4e4f534a
    tree = json.loads(data[20:20+length])
    material = next(m for m in tree['materials'] if m.get('name') == 'surface_contact_world')
    pbr = material['pbrMetallicRoughness']
    for channel in [pbr['baseColorTexture'], pbr['metallicRoughnessTexture'], material['normalTexture']]:
        sampler = tree['samplers'][tree['textures'][channel['index']]['sampler']]
        assert sampler['wrapS'] == sampler['wrapT'] == 33071


def test_surface_partitions_share_native_object_identity(contact_geometry, monkeypatch):
    geometry, folder = contact_geometry
    captured = []
    class Writer:
        triangles = meshlets = vertices = 0
        def __init__(self, path): self.path = path
        def __enter__(self): self.path.touch(); return self
        def __exit__(self, *args): return False
        def add(self, records):
            captured.append(records.copy()); self.triangles += len(records)
    monkeypatch.setattr(rendering, 'Meshlets', Writer)
    rendering.pack_geometry(geometry, folder/'identity.clm', {i:i for i in range(len(geometry.assembly.materials))})
    records = np.concatenate(captured)
    offset = 0
    for part in geometry.assembly.parts:
        object_ids = records[offset:offset+len(part.faces), 19]
        if part.group == 'surface': assert np.all(object_ids == 800000)
        elif part.group == 'water': assert np.all(object_ids == 900000)
        else: assert not np.isin(object_ids, [800000, 900000]).any()
        offset += len(part.faces)
    assert offset == len(records)


def test_depleted_foundation_keeps_wet_material_classification(tmp_path):
    state = make_terrain(Config(preset='soil-profile', grid=17, extent=10, duration=0))
    state.thickness[:] = 0
    state.loose[:] = 0
    state.water[:] = .01
    geometry = build_geometry(state, tmp_path, subdivision=1, stones=0, contact_resolution=32)
    surface = [p for p in geometry.assembly.parts if p.group == 'surface']
    assert surface and all(p.material == 1 for p in surface)
