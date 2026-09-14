"""Nominal geometry checks, not structural or skatepark safety certification."""
from __future__ import annotations
import importlib.util
import math
import sys
from pathlib import Path
import numpy as np
import pytest
from mechanism_lab.core import validate, View
from mechanism_lab.exporters import MICROFINISH_PATTERNS, export_meshbin

RECIPE=Path(__file__).resolve().parents[1]/'examples/diy_skatepark/recipe.py'
spec=importlib.util.spec_from_file_location('cybr_skatepark_recipe',RECIPE)
recipe=importlib.util.module_from_spec(spec);sys.modules[spec.name]=recipe;spec.loader.exec_module(recipe)

@pytest.fixture(scope='module')
def park():
    return recipe.build()

def test_configuration_rejects_invalid_transition():
    with pytest.raises(ValueError): recipe.ParkConfig(mini_height=2300).check()
    with pytest.raises(ValueError): recipe.ParkConfig(mini_width=4700).check()

def test_geometry_contract_and_provenance(park):
    report=validate(park,expensive=True)
    assert report['parts']==1404
    # System text-outline tessellation can differ with the installed font fallback.
    assert 150000 <= report['triangles'] <= 165000
    assert all(row['analytic_valid'] is not False for row in report['checks'])
    assert sum(row['analytic_valid'] is True for row in report['checks'])==1401
    assert report['truth']['tier_counts']=={'designed':1404}
    assert all(row['consistent_winding'] for row in report['checks'])
    assert {row['name'] for row in report['checks'] if not row['watertight']}=={
        'planter_0_leaf_blades','planter_1_leaf_blades'}

def test_circular_transition_dimensions_and_coping_reveal(park):
    for t in park.metadata['transitions']:
        R,H,theta=t['radius_mm'],t['height_mm'],t['theta_rad']
        assert abs(R*(1-math.cos(theta))-H)<1e-8
        assert abs(math.sqrt(2*R*H-H*H)-t['run_mm'])<1e-8
        endpoint=np.array((t['toe_mm']+t['sign']*t['run_mm'],t['center_y_mm'],t['base_mm']+H))
        center=np.array(t['coping_center_mm']);normal=np.array((-t['sign']*math.sin(theta),0,math.cos(theta)))
        assert abs(t['coping_radius_mm']+center[2]-endpoint[2]-t['coping_reveal_mm'])<1e-8
        assert abs(t['coping_radius_mm']+np.dot(center-endpoint,normal)-t['coping_reveal_mm'])<1e-8
        # Every tessellation vertex belongs to the exact top or underside circle.
        panels=[p for p in park.parts if p.name.startswith(t['prefix']+'_curve_panel_') and p.name.endswith('ply0')]
        assert panels
        for p in panels:
            x=(p.vertices[:,0]-t['toe_mm'])*t['sign'];z=p.vertices[:,2]-(t['base_mm']+R)
            radii=np.sqrt(x*x+z*z)
            assert np.max(np.minimum(abs(radii-R),abs(radii-(R+6))))<.005

def test_nominal_layout_separates_obstacle_envelopes(park):
    obs=park.metadata['obstacles'];assert len(obs)==7
    for i,a in enumerate(obs):
        assert -11000<=a['x0']<a['x1']<=11000
        assert -8000<=a['y0']<a['y1']<=8000
        for b in obs[i+1:]:
            overlapx=min(a['x1'],b['x1'])-max(a['x0'],b['x0'])
            overlapy=min(a['y1'],b['y1'])-max(a['y0'],b['y0'])
            assert overlapx<=0 or overlapy<=0, (a['name'],b['name'])
    treads=[p for p in park.parts if p.name.startswith('mini_access_tread_')]
    assert len(treads)==6
    assert 8000-max(p.bounds[1,1] for p in treads)>1000

def test_steel_entry_is_closed_and_joins_nominal_skin(park):
    p=next(p for p in park.parts if p.name=='street_quarter_tangent_steel_entry')
    assert p.bounds[0,2]>=-.01
    assert abs(p.bounds[1,2]-45)<1e-6
    assert np.isfinite(p.normals).all()

def test_new_finish_ids_preserve_existing_native_contract(park,tmp_path):
    assert [MICROFINISH_PATTERNS[k] for k in ['none','machined','bead-blasted','brushed','polymer','drawn-wire','copper-wire','anodized']]==list(range(8))
    assert [MICROFINISH_PATTERNS[k] for k in ['wood','concrete','grip']]==[9,10,11]
    assert View().studio_style=='product'
    assert all(v.studio_style=='outdoor' for v in park.views.values())
    export_meshbin(park,tmp_path/'scene.meshbin',photographic=True)
    lines=(tmp_path/'scene.materials').read_text().splitlines()
    assert lines[0]=='CYBR_PHOTO_MATERIALS 2' and len(lines)==1405
    assert all(len(row.split())==17 for row in lines[1:])

def test_deterministic_geometry(park):
    second=recipe.build()
    assert len(park.parts)==len(second.parts)
    for a,b in zip(park.parts,second.parts):
        assert a.name==b.name
        assert np.array_equal(a.vertices,b.vertices)
        assert np.array_equal(a.faces,b.faces)
        assert np.array_equal(a.normals,b.normals)
