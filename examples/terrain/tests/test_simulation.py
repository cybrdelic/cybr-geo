"""Behavioral checks for inventories and equilibrium, independent of rendering."""
from dataclasses import replace
import zipfile
import numpy as np
import pytest

from cybr_terrain.model import Config, State, PRESETS, make_terrain
from cybr_terrain.simulation import Simulator


def terrain(**changes):
    defaults=dict(grid=17,duration=2,rain_mm_hour=0,inlet_m3_second=0,
                  evaporation_mm_hour=0,thermal_rate=0,boundary="closed")
    defaults.update(changes)
    return make_terrain(Config(**defaults))


def test_nonflat_lake_at_rest():
    state=terrain(duration=3)
    state.layers=tuple(replace(layer,conductivity=0,erodibility=0) for layer in state.layers)
    state.water[:]=state.height.max()+.2-state.height
    initial_eta=state.height+state.water
    simulator=Simulator(state)
    report=simulator.run()
    assert report["passed"]
    np.testing.assert_allclose(state.height+state.water,initial_eta,rtol=0,atol=1e-13)
    np.testing.assert_allclose(state.qx,0,rtol=0,atol=1e-13)
    np.testing.assert_allclose(state.qy,0,rtol=0,atol=1e-13)


@pytest.mark.parametrize("preset",list(PRESETS))
def test_storm_preserves_each_grain_and_accounts_water(preset):
    state=terrain(preset=preset,extent=PRESETS[preset]["extent"],duration=8,
                  rain_mm_hour=500,inlet_m3_second=.8,boundary="open",thermal_rate=.08)
    simulator=Simulator(state)
    initial=state.solid_volumes()
    report=simulator.run()
    assert report["passed"]
    assert report["water_relative_error"]<1e-11
    np.testing.assert_allclose(state.solid_volumes()+simulator.exported_solids,initial,rtol=1e-11)
    assert simulator.ledger.outflow>0
    for field in (state.water,state.loose,state.sediment,state.thickness):
        assert field.min()>=0


def test_constant_concentration_follows_limited_water_flux():
    state=terrain()
    state.water[:]=1
    mixture=np.array([.013,.022,.035])[:,None,None]
    state.sediment[:]=mixture*state.water
    simulator=Simulator(state)
    qx=np.full_like(state.qx,.12);qy=np.zeros_like(state.qy)
    old_water=state.water.copy();old_solid=state.sediment.sum(axis=(1,2))
    dt=.3
    simulator._advect_sediment(old_water,qx,qy,np.zeros((4,state.config.grid)),dt)
    new_water=old_water.copy()
    new_water[:,:-1]-=qx*dt/state.config.dx
    new_water[:,1:]+=qx*dt/state.config.dx
    np.testing.assert_allclose(state.sediment,mixture*new_water,rtol=0,atol=1e-15)
    np.testing.assert_allclose(state.sediment.sum(axis=(1,2)),old_solid,rtol=1e-14)


def test_full_depletion_removes_real_layers_and_keeps_foundation():
    state=terrain()
    state.loose[:]=np.array([.03,.01,.002])[:,None,None]
    initial=state.solid_volumes();foundation=state.foundation.copy()
    simulator=Simulator(state)
    removed=simulator._remove(np.full_like(state.water,1000))
    np.testing.assert_allclose(removed.sum(axis=(1,2))*state.config.dx**2,initial,rtol=1e-13)
    assert state.thickness.max()<1e-14 and state.loose.max()<1e-14
    np.testing.assert_array_equal(state.foundation,foundation)
    assert simulator._remove(np.ones_like(state.water)).max()<1e-14


def test_repeated_depletion_does_not_amplify_roundoff():
    state=terrain()
    state.thickness[:]=0
    state.loose[:]=np.array([1e-9,3e-10,1e-11])[:,None,None]
    before=state.solid_volumes();simulator=Simulator(state);removed=np.zeros(3)
    for _ in range(40):
        parcel=simulator._remove(np.full_like(state.water,1e-9))
        removed+=parcel.sum(axis=(1,2))*state.config.dx**2
    np.testing.assert_allclose(removed+state.solid_volumes(),before,rtol=1e-13,atol=1e-19)
    assert state.loose.min()>=0


def test_dry_suspension_deposits_its_actual_grain_mixture():
    state=terrain();state.sediment[:]=np.array([.002,.004,.009])[:,None,None]
    before=state.solid_volumes();mixture=state.sediment.copy()
    Simulator(state)._exchange(.2)
    np.testing.assert_array_equal(state.sediment,0)
    np.testing.assert_allclose(state.loose,mixture,rtol=0,atol=1e-16)
    np.testing.assert_allclose(state.solid_volumes(),before,rtol=1e-13)


def test_repose_transfer_moves_each_grain_without_height_only_edit():
    state=terrain(thermal_rate=.3,extent=10)
    state.foundation[:]=0;state.thickness[:]=0
    center=state.config.grid//2
    state.thickness[-1,center,center]=6
    before=state.solid_volumes();height=state.height[center,center]
    Simulator(state)._thermal(.25)
    assert state.height[center,center]<height
    assert state.loose.sum()>0
    np.testing.assert_allclose(state.solid_volumes(),before,rtol=1e-12,atol=1e-13)


def test_infiltration_is_internal_transfer_and_respects_capacity():
    state=terrain(preset="watershed",duration=3)
    state.water[:]=.1
    simulator=Simulator(state);before=state.water_volume()
    report=simulator.run()
    assert report["passed"] and simulator.ledger.infiltration>0
    assert state.soil_water.max()>0
    assert np.max(state.soil_water-state.pore_capacity)<=1e-12
    assert state.water_volume()==pytest.approx(before,rel=1e-12)


def test_dry_stale_face_discharge_does_not_stall_cfl():
    state=terrain();state.qx[:]=1e12;state.qy[:]=-1e12
    assert Simulator(state).timestep()==state.config.max_dt


def test_state_roundtrip_preserves_units_and_inventory(tmp_path):
    state=terrain(rain_mm_hour=100,inlet_m3_second=.1)
    Simulator(state).run();state.save(tmp_path/"state.npz")
    with zipfile.ZipFile(tmp_path/"state.npz") as archive:
        assert archive.testzip() is None
    restored=State.load(tmp_path/"state.npz")
    assert restored.config==state.config
    assert restored.time==state.time and restored.steps==state.steps
    for field in ("foundation","thickness","loose","sediment","water","soil_water","qx","qy"):
        np.testing.assert_array_equal(getattr(restored,field),getattr(state,field))
    np.testing.assert_array_equal(restored.solid_volumes(),state.solid_volumes())


def test_failed_publication_preserves_previous_complete_state(tmp_path,monkeypatch):
    import cybr_light.runtime
    state=terrain();destination=tmp_path/"state.npz";state.save(destination)
    previous=destination.read_bytes();state.water[:]=.1
    def fail(source,destination):raise OSError("Publication interrupted")
    monkeypatch.setattr(cybr_light.runtime,"publish",fail)
    with pytest.raises(OSError,match="Publication interrupted"):state.save(destination)
    assert destination.read_bytes()==previous
    assert State.load(destination).water.max()==0


def test_truncated_state_reports_a_useful_load_error(tmp_path):
    state=terrain();path=tmp_path/"broken.npz";state.save(path)
    path.write_bytes(path.read_bytes()[:-80])
    with pytest.raises(ValueError,match="Cannot load terrain state"):State.load(path)


@pytest.mark.parametrize("changes",[
    {"grid":16},{"grid":True},{"grid":20.5},{"extent":0},{"duration":-1},
    {"rain_mm_hour":float("nan")},{"inlet_m3_second":-1},{"cfl":.46},
    {"morphological_factor":0},{"boundary":"mystery"},
    {"seed":-1},{"seed":True},{"max_steps":1.5},
])
def test_invalid_process_settings_are_rejected(changes):
    with pytest.raises(ValueError):Config(**changes)


@pytest.mark.parametrize("preset",list(PRESETS))
def test_authored_geology_is_seeded_and_exposes_interior_layers(preset):
    config=Config(preset=preset,grid=65,extent=PRESETS[preset]["extent"])
    a=make_terrain(config);b=make_terrain(config);c=make_terrain(replace(config,seed=config.seed+1))
    np.testing.assert_array_equal(a.height,b.height)
    assert not np.array_equal(a.height,c.height)
    assert len(np.unique(a.exposed[4:-4,4:-4]))>=3
    assert a.thickness.min()>=0
    assert a.validate()
