from mechanism_lab.core import View
from mechanism_lab.photoreal import _camera_distance,_render_scratch_paths,_cleanup_render_scratch


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


def test_film_sample_scratch_is_complete_and_deleted_immediately(tmp_path):
    mesh=tmp_path/'sample.meshbin';ppm=tmp_path/'sample.ppm'
    paths=_render_scratch_paths(mesh,ppm)
    assert set(p.name for p in paths)=={
        'sample.meshbin','sample.materials','sample.ppm','sample.ppm.pfm','sample.ppm.guides'
    }
    for p in paths:p.write_bytes(b'scratch')
    keep=tmp_path/'000001.png';keep.write_bytes(b'frame')
    _cleanup_render_scratch(mesh,ppm)
    assert not any(p.exists() for p in paths)
    # Encoded-frame inputs are intentionally not part of native sample scratch.
    assert keep.exists()


def test_film_scratch_cleanup_is_idempotent(tmp_path):
    mesh=tmp_path/'sample.meshbin';ppm=tmp_path/'sample.ppm'
    _cleanup_render_scratch(mesh,ppm)
    _cleanup_render_scratch(mesh,ppm)
