"""Named rendering contracts for CYBR GEO.

V9 is deliberately a contract over the shared photographic renderer rather than
another renderer fork. Both public APIs route normal still/video commands through
mechanism_lab.photoreal; these values pin the defaults and the approved V9
presentation behavior so later changes cannot silently regress it.
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
    reference_still_size: tuple[int, int]
    reference_still_spp: int
    reference_still_depth: int
    reference_film_size: tuple[int, int]
    reference_film_spp: int
    reference_film_depth: int
    reference_fps: int
    reference_duration: float


V9 = RenderContract(
    name='v9',
    renderer='photoreal',
    # Repository command defaults remain deliberately higher-sample generic
    # production budgets. The V9 reference budgets below record the approved
    # ORBIT proof settings without forcing every model into that exact shot size.
    still_size=(1600, 1100),
    still_spp=512,
    still_depth=14,
    video_size=(1280, 720),
    video_spp=128,
    video_depth=12,
    film_size=(1920, 1080),
    film_spp=144,
    film_depth=12,
    filter_passes=3,
    tone_mapping='neutral',
    studio_style='product',
    temporal_filter='geometry-reprojected current-dominant radiance reuse',
    # Approved V9/ORBIT photographic proof: delivered stills were 1920x1440
    # at 192 spp / 12 bounces / three guide-aware passes. The hero used a
    # 2304x1728 native render before Lanczos delivery. Films were 960x720,
    # 48 spp, depth 10, 24 fps, four seconds, with the same three spatial passes.
    reference_still_size=(1920, 1440),
    reference_still_spp=192,
    reference_still_depth=12,
    reference_film_size=(960, 720),
    reference_film_spp=48,
    reference_film_depth=10,
    reference_fps=24,
    reference_duration=4.0,
)

DEFAULT_RENDER_CONTRACT = V9
