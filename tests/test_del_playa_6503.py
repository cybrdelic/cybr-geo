from __future__ import annotations

import importlib.util
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "examples" / "del_playa_6503" / "recipe.py"


def load_recipe():
    spec = importlib.util.spec_from_file_location("del_playa_6503_recipe_test", SOURCE)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_del_playa_reference_assembly_is_finite_and_procedural():
    module = load_recipe()
    assembly = module.build()
    assert assembly.metadata["no_image_generation"] is True
    assert assembly.metadata["no_photogrammetry"] is True
    assert assembly.metadata["no_scan_geometry"] is True
    assert assembly.metadata["no_scan_textures"] is True
    assert len(assembly.parts) > 250
    assert sum(len(part.faces) for part in assembly.parts) > 8000
    names = {part.name for part in assembly.parts}
    assert "upper_main_shell" in names
    assert "front_rail_toprail" in names
    assert "bluff_terrain" in names
    assert any(name.startswith("front_rail_baluster_") for name in names)
    assert any(name.startswith("front_deck_board_") for name in names)
    for part in assembly.parts:
        assert np.isfinite(part.vertices).all(), part.name
        assert np.isfinite(part.normals).all(), part.name
        assert (part.faces >= 0).all(), part.name
        assert (part.faces < len(part.vertices)).all(), part.name


def test_del_playa_bounds_cover_house_bluff_and_ocean():
    assembly = load_recipe().build()
    lo, hi = assembly.bounds
    assert lo[1] < -70_000
    assert hi[1] > 10_000
    assert lo[2] < -7_000  # Estimated 7.2-m bluff; no surveyed elevation claim.
    assert hi[2] > 8_000
    assert hi[0] - lo[0] > 70_000


def test_mapped_footprint_replaces_shallow_proxy():
    module = load_recipe()
    constraints = module.site_constraints()
    poly = module.footprint_mm()
    assert constraints['footprint']['way_id'] == 42753197
    assert constraints['parcel']['apn'] == '075-223-019'
    assert np.ptp(poly[:,1]) > 32_000
    assert len(poly) == 20
    assert constraints['footprint']['status'].startswith('Community-mapped')
    # Independent linear map projection agrees with the stored authored frame.
    ll = np.asarray(constraints['footprint']['wgs84'])
    en = (ll - constraints['origin_wgs84']) * [constraints['longitude_scale_m_per_degree'], constraints['latitude_scale_m_per_degree']]
    xy = en @ np.array([constraints['local_x_east_north'], constraints['local_y_east_north']]).T
    xy[:,0] += constraints['origin_x_offset_m']
    np.testing.assert_allclose(poly*.001,xy,atol=1e-8)
    source_area = .5*abs(np.sum(poly[:,0]*np.roll(poly[:,1],-1)-poly[:,1]*np.roll(poly[:,0],-1)))
    tris = module._polygon_triangles(poly)
    ab=poly[tris[:,1]]-poly[tris[:,0]];ac=poly[tris[:,2]]-poly[tris[:,0]]
    authored_area = .5*np.sum(np.abs(ab[:,0]*ac[:,1]-ab[:,1]*ac[:,0]))
    np.testing.assert_allclose(source_area,authored_area,rtol=1e-12)


def test_scene_has_no_degenerate_faces_and_repeatable_geometry():
    assembly = load_recipe().build()
    names=[p.name for p in assembly.parts]
    assert len(names)==len(set(names))
    for p in assembly.parts:
        xyz=p.vertices[p.faces]
        area=np.linalg.norm(np.cross(xyz[:,1]-xyz[:,0],xyz[:,2]-xyz[:,0]),axis=1)
        assert np.all(area>1e-7),p.name
    assert len([p for p in assembly.parts if p.name.startswith('chainlink_')])>400
    assert any(p.name.startswith('front_deck_joist_') for p in assembly.parts)
    assert any(p.name.startswith('bluff_groundcover_') for p in assembly.parts)


def test_windows_are_holes_in_wall_geometry():
    assembly=load_recipe().build()
    opaque=np.vstack([p.vertices[p.faces] for p in assembly.parts if p.group in ('architecture','cladding')])
    # Short rays across each glazing sheet may not hit an opaque wall panel.
    for glass in [p for p in assembly.parts if p.group=='fenestration' and p.name.endswith('_glass') and not p.name.startswith('neighbor_')]:
        point=glass.vertices.mean(axis=0);direction=glass.normals[0];origin=point-direction*230
        a=opaque[:,0];e1=opaque[:,1]-a;e2=opaque[:,2]-a;pv=np.cross(direction,e2);det=np.einsum('ij,ij->i',e1,pv);ok=np.abs(det)>1e-9
        inv=np.zeros(len(det));inv[ok]=1/det[ok];tvec=origin-a;u=np.einsum('ij,ij->i',tvec,pv)*inv;q=np.cross(tvec,e1);v=q@direction*inv;t=np.einsum('ij,ij->i',e2,q)*inv
        blocked=ok&(u>=0)&(v>=0)&(u+v<=1)&(t>1e-4)&(t<460)
        assert not blocked.any(),glass.name


def test_sparse_alignment_retains_observations_and_uncertainty():
    import json
    r=json.loads((SOURCE.parent/'alignment.json').read_text())
    assert len(r['views'])==2
    assert len(r['limitations'])>=4
    assert r['views']['ocean']['reprojection_rmse_px']<25
    assert r['views']['east']['reprojection_rmse_px']<25
    for spec in r['views'].values():
        residual=[np.linalg.norm(np.array(p['observed_px'])-p['projected_px']) for p in spec['landmarks']]
        np.testing.assert_allclose(np.sqrt(np.mean(np.square(residual))),spec['reprojection_rmse_px'])
