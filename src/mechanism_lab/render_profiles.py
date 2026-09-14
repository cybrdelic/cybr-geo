"""Named rendering contracts for CYBR GEO.

V9 is deliberately a contract over the shared photographic renderer rather than
another renderer fork.  Both public APIs already route their normal still/video
commands through mechanism_lab.photoreal; these values pin the defaults that
must remain true when that pipeline evolves.
"""
from __future__ import annotations
from dataclasses import dataclass


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
    filter_passes: int
    tone_mapping: str
    studio_style: str
    temporal_filter: str


V9 = RenderContract(
    name='v9',
    renderer='photoreal',
    still_size=(1600, 1100),
    still_spp=512,
    still_depth=14,
    video_size=(1280, 720),
    video_spp=128,
    video_depth=12,
    film_size=(1920, 1080),
    film_spp=144,
    film_depth=12,
    filter_passes=2,
    tone_mapping='neutral',
    studio_style='product',
    temporal_filter='geometry-reprojected current-dominant radiance reuse',
)

DEFAULT_RENDER_CONTRACT = V9
