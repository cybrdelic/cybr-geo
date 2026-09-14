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
from .core import Part, Assembly, translation


FONT='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
BOLD='/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'
BG=(13,18,24);FG=(220,230,236);MUTED=(140,156,168);BLUE=(104,192,233);GOLD=(225,177,102)

def font(sz,bold=False):return ImageFont.truetype(BOLD if bold else FONT,sz)


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
    cells=vtk.vtkCellArray();cells.SetCells(len(p.faces),numpy_to_vtkIdTypeArray(np.column_stack([np.full(len(p.faces),3),p.faces]).astype(np.int64).ravel(),deep=True));data.SetPolys(cells)
    ns=numpy_to_vtk(np.ascontiguousarray(p.normals,np.float32),deep=True);ns.SetName('Normals');data.GetPointData().SetNormals(ns)
    return data


def clip_closed(data,normal=(0,1,0),origin=(0,0,0)):
    cl=vtk.vtkCleanPolyData();cl.SetInputData(data);cl.SetTolerance(1e-8);cl.Update()
    plane=vtk.vtkPlane();plane.SetNormal(*normal);plane.SetOrigin(*origin);pc=vtk.vtkPlaneCollection();pc.AddItem(plane)
    cutter=vtk.vtkClipClosedSurface();cutter.SetInputConnection(cl.GetOutputPort());cutter.SetClippingPlanes(pc);cutter.GenerateFacesOn();cutter.SetTolerance(1e-6);cutter.Update()
    normals=vtk.vtkPolyDataNormals();normals.SetInputConnection(cutter.GetOutputPort());normals.SetFeatureAngle(40);normals.SplittingOn();normals.ConsistencyOn();normals.Update()
    out=vtk.vtkPolyData();out.DeepCopy(normals.GetOutput());return out


class Studio:
    def __init__(self,assembly:Assembly,size=(1280,720),ao=True,section=None):
        parts=assembly.parts; materials=[vars(m) for m in assembly.materials]
        self.assembly=assembly; self.parts=parts;self.actors=[];self.buffers=[]
        self.ren=vtk.vtkRenderer();self.ren.SetBackground(.060,.070,.085);self.ren.SetBackground2(.095,.11,.13);self.ren.GradientBackgroundOn()
        self.win=vtk.vtkEGLRenderWindow();self.win.SetOffScreenRendering(1);self.win.SetSize(*size);self.win.SetMultiSamples(0);self.win.SetAlphaBitPlanes(1);self.win.AddRenderer(self.ren)
        self.ren.UseImageBasedLightingOn();self.ren.AutomaticLightCreationOff();self.tex=studio_texture();self.ren.SetEnvironmentTexture(self.tex,False);self.ren.SetEnvironmentUp(0,0,1);self.ren.SetEnvironmentRight(1,0,0)
        self.ren.UseSphericalHarmonicsOn()
        # A modest direct component gives readable glancing edges without a white wash.
        for pos,inten,col in [((-200,-160,200),.5,(.88,.94,1.)),((180,150,110),.7,(1.,.93,.83)),((-130,160,20),.3,(.9,.94,1.))]:
            light=vtk.vtkLight();light.SetPosition(*pos);light.SetFocalPoint(0,0,0);light.SetLightTypeToSceneLight();light.SetPositional(False);light.SetIntensity(inten);light.SetColor(*col);self.ren.AddLight(light)
        for p in parts:
            data=polydata(p)
            if section is not None and (section=='all' or (section=='shell' and p.group in ['carrier','front_flange','rear_flange','marking'])):
                data=clip_closed(data)
            mapper=vtk.vtkPolyDataMapper();mapper.SetInputData(data);mapper.ScalarVisibilityOff()
            actor=vtk.vtkActor();actor.SetMapper(mapper);pr=actor.GetProperty();m=materials[p.material]
            # VTK baseColor is scene-linear here; filmic tone mapping follows.
            pr.SetColor(*m['color']);pr.SetInterpolationToPBR();pr.SetMetallic(m['metal']);pr.SetRoughness(m['rough']);pr.SetCoatStrength(.08);pr.SetCoatRoughness(.23)
            self.ren.AddActor(actor);self.actors.append(actor);self.buffers.append(data)
        steps=vtk.vtkRenderStepsPass();delegate=steps;self.passes=[steps]
        if ao:
            ssao=vtk.vtkSSAOPass();ssao.SetDelegatePass(steps);ssao.SetRadius(3.8);ssao.SetBias(.035);ssao.SetKernelSize(48);ssao.BlurOn();delegate=ssao;self.passes.append(ssao)
        tone=vtk.vtkToneMappingPass();tone.SetToneMappingType(vtk.vtkToneMappingPass.GenericFilmic);tone.SetGenericFilmicDefaultPresets();tone.SetExposure(1.15);tone.SetDelegatePass(delegate);self.passes.append(tone);self.ren.SetPass(tone)
        self.ren.UseFXAAOn();self.ren.GetFXAAOptions().SetRelativeContrastThreshold(.08)
        self.cam=self.ren.GetActiveCamera();self.cam.SetViewUp(0,0,1);self.cam.ParallelProjectionOn();self.capture=vtk.vtkWindowToImageFilter();self.capture.SetInput(self.win);self.capture.SetInputBufferTypeToRGB();self.capture.ReadFrontBufferOff()
        self.set_camera()

    def set_camera(self,az=235.,el=22.,scale=88.,target=(0,0,0),distance=500):
        a,e=math.radians(az),math.radians(el);t=np.array(target,float);eye=t+distance*np.array([math.cos(a)*math.cos(e),math.sin(a)*math.cos(e),math.sin(e)])
        self.cam.SetPosition(*eye);self.cam.SetFocalPoint(*t);self.cam.SetViewUp(0,0,1);self.cam.SetParallelScale(scale);self.ren.ResetCameraClippingRange()

    def pose(self, matrices=None, explode=0.):
        matrices = {} if matrices is None else matrices
        for p, actor in zip(self.parts, self.actors):
            transform = translation(p.explode * explode) @ matrices.get(p.name, np.eye(4))
            matrix=vtk.vtkMatrix4x4()
            for j in range(4):
                for k in range(4):matrix.SetElement(j,k,transform[j,k])
            actor.SetUserMatrix(matrix)
        self.ren.ResetCameraClippingRange()

    def fit(self, az=235, el=23, padding=1.35):
        bounds=self.assembly.bounds
        self.set_camera(az,el,scale=max(np.linalg.norm(bounds[1]-bounds[0])*.5*padding,1),target=bounds.mean(0))

    def __enter__(self): return self
    def __exit__(self, *args): self.close()

    def visible(self,predicate):
        for p,a in zip(self.parts,self.actors):a.SetVisibility(bool(predicate(p)))
        self.ren.ResetCameraClippingRange()

    def render(self):
        self.win.Render();self.capture.Modified();self.capture.Update();im=self.capture.GetOutput();w,h,_=im.GetDimensions();a=vtk_to_numpy(im.GetPointData().GetScalars()).reshape(h,w,3)
        return Image.fromarray(np.flipud(a).copy())

    def close(self):self.win.Finalize()


def labelled(im,title,subtitle='',footer='',tag='CYBR GEO / GEOMETRY RENDER'):
    im=im.copy();d=ImageDraw.Draw(im);w,h=im.size;s=w/1600
    d.text((int(42*s),int(28*s)),tag,font=font(max(12,int(15*s))),fill=MUTED)
    d.text((int(40*s),int(56*s)),title,font=font(max(18,int(33*s)),True),fill=FG)
    if subtitle:d.text((int(42*s),int(107*s)),subtitle,font=font(max(11,int(16*s))),fill=MUTED)
    if footer:
        d.line((int(40*s),h-int(49*s),w-int(40*s),h-int(49*s)),fill=(50,65,79),width=1)
        d.text((int(42*s),h-int(35*s)),footer,font=font(max(10,int(13*s))),fill=MUTED)
    return im

