from mechanism_lab.core import View
from mechanism_lab.photoreal import _camera_distance


def test_explicit_camera_distance_is_respected():
    view=View(camera_distance_mm=360.,focal_length_mm=62.,sensor_width_mm=36.)
    assert _camera_distance(view,(1600,1000))==360.


def test_scale_can_derive_physical_camera_distance():
    view=View(scale=72.,camera_distance_mm=None,focal_length_mm=62.,sensor_width_mm=36.)
    distance=_camera_distance(view,(1600,1000))
    # A narrower 62 mm lens requires the camera to move back substantially to
    # preserve a 72 mm vertical half-framing height.
    assert 380 < distance < 410


def test_photographic_view_remains_perspective_by_default():
    view=View()
    assert view.projection=='perspective'
    assert view.focal_length_mm>0
    assert view.sensor_width_mm>0
