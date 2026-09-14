"""Named rendering contracts for CYBR GEO.

V9 is the captured-workshop Mitsuba/OIDN renderer proven by the ORBIT V9
artifact. The older in-house BVH/GGX renderer remains available as `photoreal`
for compatibility, but it is not V9.
"""
from __future__ import annotations
from dataclasses import dataclass
import urllib.parse as _urllib_parse
import urllib.request as _urllib_request

# v9.py keeps its downloader intentionally dependency-light. Expose urlparse on
# urllib.request for the renderer's cached-asset path derivation.
if not hasattr(_urllib_request,'urlparse'):
    _urllib_request.urlparse=_urllib_parse.urlparse


@dataclass(frozen=True)
class RenderContract:
    name: str
    renderer: str
    still_size: tuple[int, int]
    still_spp: int
    still_depth: int
    video_size: tuple[int, int]
    video_spp: int
    video_depth: int
    film_size: tuple[int, int]
    film_spp: int
    film_depth: int
    tone_mapping: str
    studio_style: str
    environment_asset: str
    environment_resolution: str
    environment_rotation_degrees: float
    environment_scale: float
    bench_asset: str
    bench_resolution: str
    exposure: float
    reconstruction_filter_stddev: float
    reference_f_stop: float
    denoiser: str
    # Retained only for the explicit legacy native photographic backend.
    filter_passes: int
    temporal_filter: str


V9 = RenderContract(
    name='v9',
    renderer='v9',
    still_size=(1100, 825),
    still_spp=256,
    still_depth=14,
    video_size=(1280, 720),
    video_spp=128,
    video_depth=12,
    film_size=(1920, 1080),
    film_spp=144,
    film_depth=14,
    tone_mapping='aces',
    studio_style='captured-workshop',
    environment_asset='small_workshop',
    environment_resolution='2k',
    environment_rotation_degrees=195.0,
    environment_scale=.72,
    bench_asset='blue_metal_plate',
    bench_resolution='1k',
    exposure=1.04,
    reconstruction_filter_stddev=.42,
    reference_f_stop=16.0,
    denoiser='Intel Open Image Denoise 2.5.1 high / albedo+normal guided',
    filter_passes=3,
    temporal_filter='none in V9; every encoded frame is independently path traced and guided-denoised',
)

DEFAULT_RENDER_CONTRACT = V9
