"""Check actual geometry alignment and rejection, not visual plausibility alone."""
from dataclasses import replace
import numpy as np
from mechanism_lab.core import View
from mechanism_lab.film_filter import camera_basis,temporal_radiance


def plane_frame(view,width=48,height=36,shift=0.):
    camera,forward,right,up=camera_basis(view,(width,height))
    x,y=np.meshgrid(np.arange(width)+.5,np.arange(height)+.5)
    rays=(forward*view.focal_length_mm
          +(2*x/width-1)[:,:,None]*right*view.sensor_width_mm/2
          +(1-2*y/height)[:,:,None]*up*view.sensor_width_mm*height/(2*width))
    points=camera+rays*(-camera[2]/rays[:,:,2])[:,:,None]
    # Linear radiance in the part's own Y coordinate has exact bilinear sampling.
    value=.3+.003*(points[:,:,1]-shift)
    radiance=np.repeat(value[:,:,None],3,axis=2).astype(np.float32)
    guide=np.zeros((height,width,9),np.float32);guide[:,:,2]=1;guide[:,:,7]=.003
    surface=np.concatenate([points,np.zeros((height,width,1))],axis=2).astype(np.float32)
    poses=np.eye(4)[None];poses[0,1,3]=shift
    return dict(radiance=radiance,guides=guide,surfaces=surface,poses=poses)


def test_exact_pose_reprojection_aligns_a_translating_surface():
    view=View(az=0,el=90,scale=20,target=(0,0,0),focal_length_mm=50)
    current=plane_frame(view);neighbor=plane_frame(view,shift=1.5)
    filtered,stats=temporal_radiance(current,[neighbor],view)
    assert stats[0]['geometry_accepted_fraction']>.80
    assert np.max(np.abs(filtered-current['radiance']))<1e-6


def test_disoccluded_parts_do_not_leak_radiance():
    view=View(az=0,el=90,scale=20,target=(0,0,0))
    current=plane_frame(view);neighbor=plane_frame(view)
    neighbor['surfaces'][:,:,3]=1
    neighbor['radiance'][:]=10
    filtered,stats=temporal_radiance(current,[neighbor],view)
    assert stats[0]['geometry_accepted_fraction']==0
    assert np.array_equal(filtered,current['radiance'])


def test_a_changed_specular_highlight_is_rejected():
    view=View(az=0,el=90,scale=20,target=(0,0,0))
    current=plane_frame(view);neighbor=plane_frame(view)
    neighbor['radiance'][:]=10
    filtered,stats=temporal_radiance(current,[neighbor],view)
    assert stats[0]['geometry_accepted_fraction']>.80
    assert np.array_equal(filtered,current['radiance'])
