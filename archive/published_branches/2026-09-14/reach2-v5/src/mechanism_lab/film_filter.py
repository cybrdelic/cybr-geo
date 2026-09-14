"""Conservative radiance reuse using actual CAD poses and primary hit points.

This filters Monte Carlo estimates. It never synthesizes a frame or changes the
current frame's geometry/visibility. Reprojection rejects disocclusions, normal
discontinuities, different parts, and changing highlights. Camera must be fixed.
"""
from __future__ import annotations
import math
import numpy as np
from .photoreal import _camera_distance


def camera_basis(view,size):
    az,el=math.radians(view.az),math.radians(view.el)
    outward=np.array([math.cos(az)*math.cos(el),math.sin(az)*math.cos(el),math.sin(el)])
    camera=np.asarray(view.target)+outward*_camera_distance(view,size)
    forward=-outward;right=np.cross(forward,[0,0,1]);right/=np.linalg.norm(right)
    up=np.cross(right,forward)
    return camera,forward,right,up


def temporal_radiance(current,neighbors,view):
    if view.projection!='perspective':raise ValueError('Temporal filter requires a perspective camera')
    radiance=current['radiance'];height,width=radiance.shape[:2]
    color=radiance.reshape(-1,3);guide=current['guides'].reshape(-1,9)
    surface=current['surfaces'].reshape(-1,4)
    ids=surface[:,3].astype(np.int32);points=surface[:,:3]
    camera,forward,right,up=camera_basis(view,(width,height))
    focal=view.focal_length_mm;sensor_width=view.sensor_width_mm;sensor_height=sensor_width*height/width
    result=color.copy();total=np.ones(len(color),np.float32)
    lum=color@np.array([.2126,.7152,.0722],np.float32)
    stats=[]
    for previous in neighbors:
        point=points.copy();normal=guide[:,:3].copy()
        for part in np.unique(ids[ids>=0]):
            selected=ids==part
            old=current['poses'][part];new=previous['poses'][part]
            rotation=new[:3,:3]@old[:3,:3].T
            translation=new[:3,3]-rotation@old[:3,3]
            point[selected]=points[selected]@rotation.T+translation
            normal[selected]=guide[selected,:3]@rotation.T
        delta=point-camera;z=delta@forward
        safe_z=np.maximum(z,1e-5)
        x=width*(.5+focal*(delta@right)/(sensor_width*safe_z))-.5
        y=height*(.5-focal*(delta@up)/(sensor_height*safe_z))-.5
        inside=(z>0)&(x>=0)&(x<width-1)&(y>=0)&(y<height-1)&(ids!=-1)
        x0=np.clip(np.floor(x),0,width-2).astype(np.int32)
        y0=np.clip(np.floor(y),0,height-2).astype(np.int32)
        fx=np.clip(x-x0,0,1);fy=np.clip(y-y0,0,1)
        reference=previous['radiance'];refguide=previous['guides'];refsurface=previous['surfaces']
        sample=np.zeros_like(color);variance=np.zeros(len(color),np.float32);valid_weight=np.zeros(len(color),np.float32)
        # World-space tolerance is tied to the pixel footprint, not object size.
        footprint=np.maximum(.06,z*sensor_width/(focal*width))
        for dx,dy in ((0,0),(1,0),(0,1),(1,1)):
            xx=x0+dx;yy=y0+dy
            g=refguide[yy,xx];s=refsurface[yy,xx]
            agrees=(s[:,3]==ids)&(np.einsum('ij,ij->i',g[:,:3],normal)>.985)
            agrees &= np.linalg.norm(s[:,:3]-point,axis=1)<footprint*1.7
            weight=((fx if dx else 1-fx)*(fy if dy else 1-fy)*agrees*inside).astype(np.float32)
            sample+=reference[yy,xx]*weight[:,None]
            variance+=g[:,7]*weight*weight;valid_weight+=weight
        accepted=valid_weight>.97
        sample/=np.maximum(valid_weight[:,None],1e-8)
        variance/=np.maximum(valid_weight*valid_weight,1e-8)
        previous_lum=sample@np.array([.2126,.7152,.0722],np.float32)
        delta_lum=previous_lum-lum
        sigma=9*(variance+guide[:,7])+.001+.006*lum*lum
        # With one neighbor the current frame keeps at least 80% of the weight. Glossy changes receive
        # rapidly decreasing weight and cannot leave a long temporal trail.
        weight=.25*np.exp(-delta_lum*delta_lum/np.maximum(sigma,1e-8))*accepted
        result+=sample*weight[:,None];total+=weight
        stats.append(dict(geometry_accepted_fraction=float(np.mean(accepted)),
                          mean_neighbor_weight=float(np.mean(weight))))
    return (result/total[:,None]).reshape(height,width,3),stats
