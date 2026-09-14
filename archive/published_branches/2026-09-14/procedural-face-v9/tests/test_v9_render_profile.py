"""Regression gate for the repository-wide V9 photographic defaults."""
from __future__ import annotations
import inspect

from mechanism_lab.cli import parser
from mechanism_lab.core import View
from mechanism_lab.photoreal import render_photoreal, render_photoreal_video
from mechanism_lab.render_profiles import V9


def _args(*argv):
    return parser().parse_args(list(argv))


def test_mechanism_lab_default_commands_match_v9_contract():
    still=_args('render','example_flange')
    assert still.renderer==V9.renderer
    assert still.size==V9.still_size
    assert still.spp==V9.still_spp
    assert still.depth==V9.still_depth

    video=_args('video','example_flange')
    assert video.renderer==V9.renderer
    assert video.size==V9.video_size
    assert video.spp==V9.video_spp
    assert video.depth==V9.video_depth

    film=_args('film','example_flange')
    assert film.size==V9.film_size
    assert film.spp==V9.film_spp
    assert film.depth==V9.film_depth


def test_shared_photographic_api_and_view_defaults_match_v9():
    still=inspect.signature(render_photoreal).parameters
    film=inspect.signature(render_photoreal_video).parameters
    assert still['spp'].default==V9.still_spp
    assert still['depth'].default==V9.still_depth
    assert film['spp'].default==V9.film_spp
    assert film['depth'].default==V9.film_depth
    assert V9.filter_passes==3
    assert 'V9.filter_passes' in inspect.getsource(render_photoreal)
    assert 'V9.filter_passes' in inspect.getsource(render_photoreal_video)
    assert V9.reference_still_size==(1920,1440)
    assert (V9.reference_still_spp,V9.reference_still_depth)==(192,12)
    assert V9.reference_film_size==(960,720)
    assert (V9.reference_film_spp,V9.reference_film_depth,V9.reference_fps,V9.reference_duration)==(48,10,24,4.0)
    view=View()
    assert view.tone_mapping==V9.tone_mapping
    assert view.studio_style==V9.studio_style


def test_cybrgeo_cli_keeps_photoreal_as_default():
    # cybrgeo builds its argparse surface inside main(), so inspect the tiny
    # declaration directly rather than creating a scene just to parse flags.
    from cybrgeo import cli
    source=inspect.getsource(cli.main)
    assert "--backend',default='photoreal'" in source
    assert "--samples',type=int,default=512" in source
    assert "--backend',choices=['photoreal','pbr'],default='photoreal'" in source
    assert "--samples',type=int,default=128" in source
