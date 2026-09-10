"""Geometry-only studio renderer and frame-by-frame kinematic recordings.

VTK / Mesa EGL PBR + procedural analytic studio environment + SSAO for video.
No reference-image textures, generated imagery, or still-image camera pans.
The separate pathtrace.cpp is used for offline stills.
"""
from __future__ import annotations
import os
os.environ.setdefault('LP_NUM_THREADS','4')
os.environ.setdefault('OMP_NUM_THREADS','4')
os.environ.setdefault('OPENBLAS_NUM_THREADS','1')
import sys, time, json, math, subprocess, argparse
from pathlib import Path
import numpy as np
from PIL import Image,ImageDraw,ImageFont
import vtk
from vtk.util.numpy_support import numpy_to_vtk,numpy_to_vtkIdTypeArray,vtk_to_numpy
from .core import Part, Assembly, View
from .geometry import *


FONT='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
BOLD='/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'
BG=(13,18,24);FG=(220,230,236);MUTED=(140,156,168);BLUE=(104,192,233);GOLD=(225,177,102)
AA_FACTORS={'none':1,'fxaa':1,'ssaa2':2,'ssaa3':3,'ssaa4':4}


def normalize_aa(mode):
    mode=str(mode).lower()
    if mode not in AA_FACTORS:
        raise ValueError(f'Unknown antialiasing mode {mode}; choose {tuple(AA_FACTORS)}')
    return mode


def antialias_description(mode):
    mode=normalize_aa(mode)
    if mode=='none':return 'antialiasing disabled'
    if mode=='fxaa':return 'FXAA'
    factor=AA_FACTORS[mode]
    return f'{factor}x SSAA / Lanczos resolve / FXAA'


def font(sz,bold=False):
    try:return ImageFont.truetype(BOLD if bold else FONT,sz)
    except OSError:return ImageFont.load_default(size=sz)


def studio_texture():
    """Analytic lat-long reflection cards. Float radiance, not a photograph."""
    h,w=512,1024
    u=(np.arange(w)+.5)/w*2*np.pi;v=(np.arange(h)+.5)/h*np.pi
    # VTK environment Y-up is explicitly changed below to world Z-up.
    xx=np.sin(v)[:,None]*np.cos(u)[None,:];yy=np.cos(v)[:,None]*np.ones((1,w));zz=np.sin(v)[:,None]*np.sin(u)[None,:]
    dirs=np.stack([xx,yy,zz],axis=-1)
    data=np.ones((h,w,3),np.float32)*np.array([.11,.125,.145])
    data+=np.maximum(yy,0)[...,None]*np.array([.18,.19,.2])
    def card(az,el,width,height,intensity,col):
        a,e=math.radians(az),math.radians(el);c=np.array([math.cos(e)*math.cos(a),math.sin(e),math.cos(e)*math.sin(a)])
        right=np.array([-math.sin(a),0,math.cos(a)]);up=np.cross(right,c)
        denom=np.maximum(dirs@c,1e-5)
        px=(dirs@right)/denom;py=(dirs@up)/denom
        hw,hh=math.tan(math.radians(width/2)),math.tan(math.radians(height/2))
        mask=np.clip((hw-np.abs(px))/.022,0,1)*np.clip((hh-np.abs(py))/.022,0,1)*(dirs@c>0)
        data[:]+=mask[...,None]*np.array(col)*intensity
    card(205,57,95,48,3.2,(.93,.97,1.))
    card(118,20,28,87,4.8,(1.,.98,.94))
    card(350,40,77,45,3.8,(.90,.95,1.))
    card(258,-12,30,58,1.4,(.85,.9,1.))
    image=vtk.vtkImageData();image.SetDimensions(w,h,1);image.GetPointData().SetScalars(numpy_to_vtk(np.ascontiguousarray(data.reshape(-1,3)),deep=True,array_type=vtk.VTK_FLOAT))
    tex=vtk.vtkTexture();tex.SetInputData(image);tex.InterpolateOn();tex.MipmapOn();tex.SetColorModeToDirectScalars()
    return tex


def polydata(p:Part):
    data=vtk.vtkPolyData();pts=vtk.vtkPoints();pts.SetData(numpy_to_vtk(np.ascontiguousarray(p.vertices,np.float32),deep=True));data.SetPoints(pts)
    cells=vtk.vtkCellArray();cells.SetData(numpy_to_vtkIdTypeArray(np.arange(len(p.faces)+1,dtype=np.int64)*3,deep=True),numpy_to_vtkIdTypeArray(np.asarray(p.faces,dtype=np.int64).ravel(),deep=True));data.SetPolys(cells)
    ns=numpy_to_vtk(np.ascontiguousarray(p.normals,np.float32),deep=True);ns.SetName('Normals');data.GetPointData().SetNormals(ns)
    return data


def clip_closed(data,normal=(0,1,0),origin=(0,0,0)):
    cl=vtk.vtkCleanPolyData();cl.SetInputData(data);cl.SetTolerance(1e-8);cl.Update()
    plane=vtk.vtkPlane();plane.SetNormal(*normal);plane.SetOrigin(*origin);pc=vtk.vtkPlaneCollection();pc.AddItem(plane)
    cutter=vtk.vtkClipClosedSurface();cutter.SetInputConnection(cl.GetOutputPort());cutter.SetClippingPlanes(pc);cutter.GenerateFacesOn();cutter.SetTolerance(1e-6);cutter.Update()
    normals=vtk.vtkPolyDataNormals();normals.SetInputConnection(cutter.GetOutputPort());normals.SetFeatureAngle(40);normals.SplittingOn();normals.ConsistencyOn();normals.Update()
    out=vtk.vtkPolyData();out.DeepCopy(normals.GetOutput());return out


class Studio:
    def __init__(self,assembly:Assembly,size=(1280,720),ao=True,section=None,aa='ssaa2'):
        self.assembly=assembly;parts=assembly.parts;self.parts=parts;self.actors=[];self.buffers=[]
        self.output_size=tuple(int(v) for v in size);self.aa=normalize_aa(aa);self.supersample=AA_FACTORS[self.aa]
        self.render_size=tuple(v*self.supersample for v in self.output_size)
        if min(self.output_size)<=0 or max(self.render_size)>16384:
            raise ValueError(f'Invalid AA render size {self.render_size}; resolved output is {self.output_size}')
        self.ren=vtk.vtkRenderer();self.ren.SetBackground(.060,.070,.085);self.ren.SetBackground2(.095,.11,.13);self.ren.GradientBackgroundOn()
        self.win=vtk.vtkEGLRenderWindow();self.win.SetOffScreenRendering(1);self.win.SetSize(*self.render_size);self.win.SetMultiSamples(0);self.win.SetAlphaBitPlanes(1);self.win.AddRenderer(self.ren)
        self.ren.UseImageBasedLightingOn();self.ren.AutomaticLightCreationOff();self.tex=studio_texture();self.ren.SetEnvironmentTexture(self.tex,False);self.ren.SetEnvironmentUp(0,0,1);self.ren.SetEnvironmentRight(1,0,0)
        self.ren.UseSphericalHarmonicsOn()
        # A modest direct component gives readable glancing edges without a white wash.
        for pos,inten,col in [((-200,-160,200),.5,(.88,.94,1.)),((180,150,110),.7,(1.,.93,.83)),((-130,160,20),.3,(.9,.94,1.))]:
            light=vtk.vtkLight();light.SetPosition(*pos);light.SetFocalPoint(0,0,0);light.SetLightTypeToSceneLight();light.SetPositional(False);light.SetIntensity(inten);light.SetColor(*col);self.ren.AddLight(light)
        for p in parts:
            data=polydata(p)
            if section is not None:
                data=clip_closed(data,normal=section)
            mapper=vtk.vtkPolyDataMapper();mapper.SetInputData(data);mapper.ScalarVisibilityOff()
            actor=vtk.vtkActor();actor.SetMapper(mapper);pr=actor.GetProperty();m=assembly.materials[p.material].as_dict()
            # VTK baseColor is scene-linear here; filmic tone mapping follows.
            pr.SetColor(*m['color']);pr.SetInterpolationToPBR();pr.SetMetallic(m['metal']);pr.SetRoughness(m['rough']);pr.SetCoatStrength(.08);pr.SetCoatRoughness(.23)
            self.ren.AddActor(actor);self.actors.append(actor);self.buffers.append(data)
        steps=vtk.vtkRenderStepsPass();delegate=steps;self.passes=[steps]
        if ao:
            ssao=vtk.vtkSSAOPass();ssao.SetDelegatePass(steps);ssao.SetRadius(3.8);ssao.SetBias(.035);ssao.SetKernelSize(48);ssao.BlurOn();delegate=ssao;self.passes.append(ssao)
        tone=vtk.vtkToneMappingPass();tone.SetToneMappingType(vtk.vtkToneMappingPass.GenericFilmic);tone.SetGenericFilmicDefaultPresets();tone.SetExposure(1.15);tone.SetDelegatePass(delegate);self.passes.append(tone);self.ren.SetPass(tone)
        if self.aa=='none':self.ren.UseFXAAOff()
        else:
            self.ren.UseFXAAOn();self.ren.GetFXAAOptions().SetRelativeContrastThreshold(.08)
        self.cam=self.ren.GetActiveCamera();self.cam.SetViewUp(0,0,1);self.cam.ParallelProjectionOn();self.capture=vtk.vtkWindowToImageFilter();self.capture.SetInput(self.win);self.capture.SetInputBufferTypeToRGB();self.capture.ReadFrontBufferOff()
        self.set_camera()

    def set_camera(self,az=235.,el=22.,scale=88.,target=(0,0,0),distance=500):
        a,e=math.radians(az),math.radians(el);t=np.array(target,float);eye=t+distance*np.array([math.cos(a)*math.cos(e),math.sin(a)*math.cos(e),math.sin(e)])
        self.cam.SetPosition(*eye);self.cam.SetFocalPoint(*t);self.cam.SetViewUp(0,0,1);self.cam.SetParallelScale(scale);self.ren.ResetCameraClippingRange()

    def pose(self,t=0,explode=0):
        seconds=t
        for p,a in zip(self.parts,self.actors):
            t=self.assembly.pose(p,seconds,explode);matrix=vtk.vtkMatrix4x4()
            for j in range(4):
                for k in range(4):matrix.SetElement(j,k,t[j,k])
            a.SetUserMatrix(matrix)
        self.ren.ResetCameraClippingRange()

    def visible(self,predicate):
        for p,a in zip(self.parts,self.actors):a.SetVisibility(bool(predicate(p)))
        self.ren.ResetCameraClippingRange()

    def render(self):
        self.win.Render();self.capture.Modified();self.capture.Update();im=self.capture.GetOutput();w,h,_=im.GetDimensions();a=vtk_to_numpy(im.GetPointData().GetScalars()).reshape(h,w,3)
        image=Image.fromarray(np.flipud(a).copy())
        if self.supersample>1:
            image=image.resize(self.output_size,Image.Resampling.LANCZOS)
        return image

    def close(self):self.win.Finalize()


def labelled(im,title,subtitle='',footer='',tag='CYBR MECHANISM LAB / GEOMETRY RENDER'):
    im=im.copy();d=ImageDraw.Draw(im);w,h=im.size;s=w/1600
    d.text((int(42*s),int(28*s)),tag,font=font(max(12,int(15*s))),fill=MUTED)
    d.text((int(40*s),int(56*s)),title,font=font(max(18,int(33*s)),True),fill=FG)
    if subtitle:d.text((int(42*s),int(107*s)),subtitle,font=font(max(11,int(16*s))),fill=MUTED)
    if footer:
        d.line((int(40*s),h-int(49*s),w-int(40*s),h-int(49*s)),fill=(50,65,79),width=1)
        d.text((int(42*s),h-int(35*s)),footer,font=font(max(10,int(13*s))),fill=MUTED)
    return im


def render_still(assembly, output, view_name='hero', size=(1600,1100), time_seconds=0., captions=True, aa='ssaa2'):
    view=assembly.views[view_name]
    studio=Studio(assembly,size=size,section=view.section,aa=aa)
    try:
        studio.visible(lambda p:p.group not in view.hide)
        studio.pose(time_seconds,view.explode)
        studio.set_camera(view.az,view.el,view.scale,view.target)
        im=studio.render()
        if captions:im=labelled(im,view.title or assembly.name.upper(),view.note,
                              'Actual 3D geometry / PBR inspection / '+antialias_description(aa)+' / '+str(len(assembly.parts))+' named components')
        output=Path(output);output.parent.mkdir(parents=True,exist_ok=True);im.save(output)
        return output
    finally:studio.close()
