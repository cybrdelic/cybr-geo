from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from PIL import Image

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


@pytest.mark.parametrize('spp,samples,expected',[
    (48,3,[16,16,16]),
    (64,3,[22,21,21]),
    (193,5,[39,39,39,38,38]),
])
def test_film_exact_budget_streaming_and_v9_reprojection(tmp_path,monkeypatch,flange,spp,samples,expected):
    """Exercise the selected film loop, not helpers from the superseded branch.

    Native tracing and encoding are explicit test doubles. This is a control-flow
    regression test, not render-quality or physical-validation evidence.
    """
    from mechanism_lab import photoreal as photo
    from mechanism_lab import finish_render as filt
    from mechanism_lab import film_filter,media,truth

    budgets=[];scratch=[];encoded=[];reuse=[]

    def invoke(exe,assembly,view,mesh,ppm,size,sample_spp,threads,depth,seed,time_seconds=0.,explode=None):
        # Every prior temporal sample must already have released ALL native files,
        # including the newer V9 surface buffer used by temporal reprojection.
        assert not any(path.exists() for path in scratch)
        paths=[mesh,mesh.with_suffix('.materials'),ppm,Path(str(ppm)+'.pfm'),
               Path(str(ppm)+'.guides'),Path(str(ppm)+'.surfaces')]
        for path in paths:path.write_bytes(b'unit-test native scratch')
        header=np.asarray(size,dtype='<u4').tobytes()
        Path(str(ppm)+'.guides').write_bytes(header+np.zeros((size[1],size[0],9),dtype='<f4').tobytes())
        Path(str(ppm)+'.surfaces').write_bytes(header+np.zeros((size[1],size[0],4),dtype='<f4').tobytes())
        scratch.extend(paths);budgets.append(sample_spp)

    def run(command,**kwargs):
        if '-frames:v' in command:
            count=int(command[command.index('-frames:v')+1])
            directory=Path(command[command.index('-i')+1]).parent
            frames=sorted(directory.glob('[0-9][0-9][0-9][0-9][0-9][0-9].png'))
            assert len(frames)==count==2
            assert not any(path.exists() for path in scratch)
            encoded.extend(frames)
            Path(command[-1]).write_bytes(b'unit-test encoder output')
        return SimpleNamespace(returncode=0)

    def temporal(current,history,view):
        assert current['surfaces'].shape==(2,2,4)
        assert current['poses'].shape==(len(flange.parts),4,4)
        reuse.append(len(history))
        return current['radiance'],[]

    monkeypatch.setattr(photo,'compile_renderer',lambda:Path('unit-test-native-stub'))
    monkeypatch.setattr(photo,'_invoke',invoke)
    monkeypatch.setattr(photo.subprocess,'run',run)
    monkeypatch.setattr(filt,'read_pfm',lambda path:np.zeros((2,2,3),dtype=np.float32))
    monkeypatch.setattr(filt,'finish_frame',lambda *args:Image.new('RGB',(2,2)))
    monkeypatch.setattr(film_filter,'temporal_radiance',temporal)
    monkeypatch.setattr(media,'probe',lambda path:{'streams':[{'nb_read_frames':str(len(encoded))}]})
    monkeypatch.setattr(truth,'assert_renderable',lambda *args:{'resolved_intent':'concept'})
    monkeypatch.setattr(truth,'write_truth_report',lambda *args:None)
    shot=SimpleNamespace(view='hero',action='motion',duration=2/24,orbit_degrees=0.)
    report=photo.render_photoreal_video(flange,tmp_path/'film.mp4',[shot],size=(2,2),fps=24,
                                      spp=spp,shutter_samples=samples,threads=1,depth=2)
    assert budgets==expected*2
    assert sum(budgets)==spp*2
    assert reuse==[1]
    assert report['frames']==2
    assert len(encoded)==2
    assert not any(path.exists() for path in scratch)
