"""Shared quality defaults for the bundled CYBR LIGHT spectral renderer."""
from dataclasses import dataclass

@dataclass(frozen=True)
class RenderContract:
    name: str='light'
    renderer: str='light'
    still_size: tuple=(1100,825)
    still_spp: int=96
    still_depth: int=14
    video_size: tuple=(1280,720)
    video_spp: int=64
    video_depth: int=12
    film_size: tuple=(1920,1080)
    film_spp: int=128
    film_depth: int=14
    bands: int=8
    filter_passes: int=3 # explicit legacy photoreal finishing only

LIGHT=RenderContract()
DEFAULT_RENDER_CONTRACT=LIGHT
# Source compatibility for existing recipes using size/sample constants.
V9=LIGHT
