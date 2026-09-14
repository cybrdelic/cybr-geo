"""Regression gate for the actual repository-wide V9 photographic defaults."""
from __future__ import annotations
import inspect
import numpy as np

from mechanism_lab.cli import parser
from mechanism_lab.render_profiles import V9
from mechanism_lab import v9


def _args(*argv):
    return parser().parse_args(list(argv))


def test_lab_defaults_are_true_v9():
    still=_args('render','example_flange')
    assert still.renderer=='v9'==V9.renderer
    assert still.size==V9.still_size==(1100,825)
    assert still.spp==V9.still_spp==256
    assert still.depth==V9.still_depth==14

    video=_args('video','example_flange')
    assert video.renderer=='v9'
    assert video.size==V9.video_size
    assert video.spp==V9.video_spp
    assert video.depth==V9.video_depth

    film=_args('film','example_flange')
    assert film.renderer=='v9'
    assert film.size==V9.film_size
    assert film.spp==V9.film_spp
    assert film.depth==V9.film_depth


def test_v9_matches_approved_orbit_renderer_contract():
    assert V9.studio_style=='captured-workshop'
    assert V9.tone_mapping=='aces'
    assert V9.environment_asset=='small_workshop'
    assert V9.environment_resolution=='2k'
    assert V9.environment_rotation_degrees==195.0
    assert V9.environment_scale==.72
    assert V9.bench_asset=='blue_metal_plate'
    assert V9.bench_resolution=='1k'
    assert V9.exposure==1.04
    assert V9.reconstruction_filter_stddev==.42
    assert V9.reference_f_stop==16.0
    assert 'Open Image Denoise 2.5.1' in V9.denoiser
    assert 'Mitsuba 3' in inspect.getdoc(v9)


def test_v9_aces_transfer_is_finite_monotonic_and_achromatic():
    ramp=np.repeat(np.geomspace(1e-6,100,10000)[:,None],3,axis=1).astype(np.float32)
    mapped=v9._aces(ramp,V9.exposure)
    assert np.isfinite(mapped).all()
    assert (np.diff(mapped[:,0])>=-1e-7).all()
    assert np.allclose(mapped[:,0],mapped[:,1]) and np.allclose(mapped[:,1],mapped[:,2])
    assert mapped.min()>=0 and mapped.max()<=1


def test_cybrgeo_cli_defaults_to_v9_not_legacy_native():
    from cybrgeo import cli
    source=inspect.getsource(cli.main)
    assert "--backend',default='v9'" in source
    assert "choices=['v9','photoreal','pbr']" in source


def test_original_cybrgeo_api_uses_v9():
    from cybrgeo import photoreal
    assert photoreal.render.__defaults__[0:4]==(V9.still_size[0],V9.still_size[1],V9.still_spp,V9.still_depth)
    assert 'render_v9' in inspect.getsource(photoreal.render)
    assert 'render_v9_video' in inspect.getsource(photoreal.film)
