"""Geometry-only studio renderer and frame-by-frame kinematic recordings.

The renderer never substitutes reference images, generated textures, image
billboards, or still-image pans for geometry.  Hero views use a physical-ish
perspective camera, explicit PBR material properties, grounded studio lighting,
and supersampled output. Engineering views can opt into orthographic projection.
"""
from __future__ import annotations
import os
os.environ.setdefault('LP_NUM_THREADS','4')
os.environ.setdefault('OMP_NUM_THREADS','4')
os.environ.setdefault('OPENBLAS_NUM_THREADS','1')
import math
from pathlib import Path
import numpy as np
from PIL import Image,ImageDraw,ImageFont
import vtk
from vtk.util.numpy_support import numpy_to_vtk,numpy_to_vtkIdTypeArray,vtk_to_numpy
from .core import Part, Assembly, View


FONT='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
BOLD='/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'
FG=(220,230,236);MUTED=(140,156,168)


def font(sz,bold=False):
    try:return ImageFont.truetype(BOLD if bold else FONT,sz)
    except OSError:return ImageFont.load_default(size=sz)


def studio_texture():
    """Procedural float-radiance studio; no photographic/generated environment."""
    h,w=768,1536
    u=(np.arange(w)+.5)/w*2*np.pi;v=(np.arange(h)+.5)/h*np.pi
    xx=np.sin(v)[:,None]*np.cos(u)[None,:]
    yy=np.cos(v)[:,None]*np.ones((1,w))
    zz=np.sin(v)[:,None]*np.sin(u)[None,:]
    dirs=np.stack([xx,yy,zz],axis=-1)
    # Neutral dark cyclorama with a gentle overhead lift.
    data=np.ones((h,w,3),np.float32)*np.array([.045,.052,.061])
    data+=np.maximum(yy,0)[...,None]*np.array([.12,.13,.145])

    def card(az,el,width,height,intensity,col,softness=.035):
        a,e=math.radians(az),math.radians(el)
        c=np.array([math.cos(e)*math.cos(a),math.sin(e),math.cos(e)*math.sin(a)])
        right=np.array([-math.sin(a),0,math.cos(a)]);up=np.cross(right,c)
        denom=np.maximum(dirs@c,1e-5)
        px=(dirs@right)/denom;py=(dirs@up)/denom
        hw,hh=math.tan(math.radians(width/2)),math.tan(math.radians(height/2))
        mask=np.clip((hw-np.abs(px))/softness,0,1)*np.clip((hh-np.abs(py))/softness,0,1)*(dirs@c>0)
        # smoothstep the card edges to avoid hard-box reflection artifacts
        mask=mask*mask*(3-2*mask)
        data[:]+=mask[...,None]*np.array(col)*intensity

    card(210,55,78,42,4.5,(.95,.98,1.00))
    card(115,18,24,78,3.5,(1.00,.94,.86))
    card(350,35,64,34,2.5,(.88,.94,1.00))
    card(265,-8,35,55,.9,(.78,.84,.92))
    image=vtk.vtkImageData();image.SetDimensions(w,h,1)
    image.GetPointData().SetScalars(numpy_to_vtk(np.ascontiguousarray(data.reshape(-1,3)),deep=True,array_type=vtk.VTK_FLOAT))
    tex=vtk.vtkTexture();tex.SetInputData(image);tex.InterpolateOn();tex.MipmapOn();tex.SetColorModeToDirectScalars()
    return tex


def polydata(p:Part):
    data=vtk.vtkPolyData();pts=vtk.vtkPoints()
    pts.SetData(numpy_to_vtk(np.ascontiguousarray(p.vertices,np.float32),deep=True));data.SetPoints(pts)
    cells=vtk.vtkCellArray();cells.SetData(
        numpy_to_vtkIdTypeArray(np.arange(len(p.faces)+1,dtype=np.int64)*3,deep=True),
        numpy_to_vtkIdTypeArray(np.asarray(p.faces,dtype=np.int64).ravel(),deep=True))
    data.SetPolys(cells)
    ns=numpy_to_vtk(np.ascontiguousarray(p.normals,np.float32),deep=True);ns.SetName('Normals');data.GetPointData().SetNormals(ns)
    return data


def clip_closed(data,normal=(0,1,0),origin=(0,0,0)):
    cl=vtk.vtkCleanPolyData();cl.SetInputData(data);cl.SetTolerance(1e-8);cl.Update()
    plane=vtk.vtkPlane();plane.SetNormal(*normal);plane.SetOrigin(*origin)
    pc=vtk.vtkPlaneCollection();pc.AddItem(plane)
    cutter=vtk.vtkClipClosedSurface();cutter.SetInputConnection(cl.GetOutputPort());cutter.SetClippingPlanes(pc)
    cutter.GenerateFacesOn();cutter.SetTolerance(1e-6);cutter.Update()
    normals=vtk.vtkPolyDataNormals();normals.SetInputConnection(cutter.GetOutputPort());normals.SetFeatureAngle(40)
    normals.SplittingOn();normals.ConsistencyOn();normals.Update()
    out=vtk.vtkPolyData();out.DeepCopy(normals.GetOutput());return out


def _material_property(prop, material):
    m=material.as_dict()
    prop.SetColor(*m['color']);prop.SetInterpolationToPBR()
    prop.SetMetallic(m['metal']);prop.SetRoughness(m['rough'])
    prop.SetBaseIOR(m['ior']);prop.SetCoatStrength(m['coat']);prop.SetCoatRoughness(m['coat_rough'])
    prop.SetAnisotropy(m['anisotropy']);prop.SetAnisotropyRotation(m['anisotropy_rotation'])
    prop.SetOpacity(m['opacity'])


def _floor_actor(bounds):
    center=np.mean(bounds,axis=0);extent=np.ptp(bounds,axis=0)
    z=float(bounds[0,2]-max(3.0,.10*max(extent)))
    half=max(180.,float(max(extent[:2]))*2.6,float(extent[0])*1.3)
    plane=vtk.vtkPlaneSource();plane.SetOrigin(center[0]-half,center[1]-half,z)
    plane.SetPoint1(center[0]+half,center[1]-half,z);plane.SetPoint2(center[0]-half,center[1]+half,z);plane.SetResolution(1,1);plane.Update()
    mapper=vtk.vtkPolyDataMapper();mapper.SetInputConnection(plane.GetOutputPort())
    actor=vtk.vtkActor();actor.SetMapper(mapper);p=actor.GetProperty();p.SetInterpolationToPBR();p.SetColor(.028,.032,.038)
    p.SetMetallic(.05);p.SetRoughness(.36);p.SetBaseIOR(1.48);p.SetCoatStrength(.08);p.SetCoatRoughness(.22)
    return actor,z


class Studio:
    def __init__(self,assembly:Assembly,size=(1280,720),ao=True,section=None,floor=True,supersample=1,exposure=1.0):
        self.assembly=assembly;self.parts=assembly.parts;self.actors=[];self.buffers=[]
        self.output_size=tuple(map(int,size));self.supersample=max(1,int(supersample))
        render_size=(self.output_size[0]*self.supersample,self.output_size[1]*self.supersample)
        self.ren=vtk.vtkRenderer();self.ren.SetBackground(.026,.031,.038);self.ren.SetBackground2(.065,.074,.086);self.ren.GradientBackgroundOn()
        self.win=vtk.vtkEGLRenderWindow();self.win.SetOffScreenRendering(1);self.win.SetSize(*render_size);self.win.SetMultiSamples(0);self.win.SetAlphaBitPlanes(1);self.win.AddRenderer(self.ren)
        self.ren.UseImageBasedLightingOn();self.ren.AutomaticLightCreationOff();self.tex=studio_texture();self.ren.SetEnvironmentTexture(self.tex,False)
        self.ren.SetEnvironmentUp(0,0,1);self.ren.SetEnvironmentRight(1,0,0);self.ren.UseSphericalHarmonicsOn()

        # Multiple low-intensity lights approximate broad sources in the fast
        # preview renderer; the offline tracer uses actual finite area lights.
        key_positions=[(-230,-190,250),(-200,-120,235),(-150,-205,210)]
        fill_positions=[(210,170,145),(170,210,110)]
        rim_positions=[(-160,210,70),(-210,170,115)]
        for positions,intensity,color in [
            (key_positions,.42,(.94,.98,1.0)),
            (fill_positions,.32,(1.0,.91,.80)),
            (rim_positions,.24,(.82,.90,1.0)),
        ]:
            for pos in positions:
                light=vtk.vtkLight();light.SetPosition(*pos);light.SetFocalPoint(*np.mean(assembly.bounds,axis=0));light.SetLightTypeToSceneLight()
                light.SetPositional(True);light.SetConeAngle(80);light.SetIntensity(intensity);light.SetColor(*color)
                light.SetAttenuationValues(1,0,0);self.ren.AddLight(light)

        for part in self.parts:
            data=polydata(part)
            if section is not None:data=clip_closed(data,normal=section)
            mapper=vtk.vtkPolyDataMapper();mapper.SetInputData(data);mapper.ScalarVisibilityOff()
            actor=vtk.vtkActor();actor.SetMapper(mapper);_material_property(actor.GetProperty(),assembly.materials[part.material])
            self.ren.AddActor(actor);self.actors.append(actor);self.buffers.append(data)

        self.floor_actor=None;self.floor_z=None
        if floor:
            self.floor_actor,self.floor_z=_floor_actor(assembly.bounds);self.ren.AddActor(self.floor_actor)

        steps=vtk.vtkRenderStepsPass();delegate=steps;self.passes=[steps]
        if ao:
            ssao=vtk.vtkSSAOPass();ssao.SetDelegatePass(steps)
            extent=max(np.ptp(assembly.bounds,axis=0));ssao.SetRadius(max(1.0,extent*.025));ssao.SetBias(.018);ssao.SetKernelSize(64);ssao.BlurOn()
            delegate=ssao;self.passes.append(ssao)
        tone=vtk.vtkToneMappingPass();tone.SetToneMappingType(vtk.vtkToneMappingPass.GenericFilmic);tone.SetGenericFilmicDefaultPresets();tone.SetExposure(1.05*float(exposure));tone.SetDelegatePass(delegate)
        self.passes.append(tone);self.ren.SetPass(tone)
        self.ren.UseFXAAOn();self.ren.GetFXAAOptions().SetRelativeContrastThreshold(.05);self.ren.GetFXAAOptions().SetHardContrastThreshold(.18)
        self.cam=self.ren.GetActiveCamera();self.cam.SetViewUp(0,0,1)
        self.capture=vtk.vtkWindowToImageFilter();self.capture.SetInput(self.win);self.capture.SetInputBufferTypeToRGB();self.capture.ReadFrontBufferOff()
        self.set_camera()

    def set_camera(self,az=235.,el=22.,scale=88.,target=(0,0,0),distance=None,projection='perspective',focal_length_mm=58.,sensor_width_mm=36.):
        aspect=self.output_size[0]/self.output_size[1]
        sensor_height=float(sensor_width_mm)/aspect
        vfov=2*math.atan(sensor_height/(2*float(focal_length_mm)))
        if distance is None:
            distance=float(scale)/max(math.tan(vfov/2),1e-6)*1.12
        a,e=math.radians(az),math.radians(el);t=np.array(target,float)
        eye=t+float(distance)*np.array([math.cos(a)*math.cos(e),math.sin(a)*math.cos(e),math.sin(e)])
        self.cam.SetPosition(*eye);self.cam.SetFocalPoint(*t);self.cam.SetViewUp(0,0,1)
        if projection=='orthographic':
            self.cam.ParallelProjectionOn();self.cam.SetParallelScale(float(scale))
        elif projection=='perspective':
            self.cam.ParallelProjectionOff();self.cam.SetViewAngle(math.degrees(vfov))
        else:raise ValueError(f'Unknown projection {projection!r}')
        self.ren.ResetCameraClippingRange()

    def set_view(self,view:View):
        self.set_camera(view.az,view.el,view.scale,view.target,view.camera_distance_mm,view.projection,view.focal_length_mm,view.sensor_width_mm)

    def pose(self,t=0,explode=0):
        seconds=t
        for part,actor in zip(self.parts,self.actors):
            T=self.assembly.pose(part,seconds,explode);matrix=vtk.vtkMatrix4x4()
            for j in range(4):
                for k in range(4):matrix.SetElement(j,k,T[j,k])
            actor.SetUserMatrix(matrix)
        self.ren.ResetCameraClippingRange()

    def visible(self,predicate):
        for p,a in zip(self.parts,self.actors):a.SetVisibility(bool(predicate(p)))
        self.ren.ResetCameraClippingRange()

    def render(self):
        self.win.Render();self.capture.Modified();self.capture.Update();im=self.capture.GetOutput();w,h,_=im.GetDimensions()
        a=vtk_to_numpy(im.GetPointData().GetScalars()).reshape(h,w,3);out=Image.fromarray(np.flipud(a).copy())
        if self.supersample>1:out=out.resize(self.output_size,Image.Resampling.LANCZOS)
        return out

    def close(self):self.win.Finalize()


def labelled(im,title,subtitle='',footer='',tag='CYBR MECHANISM LAB / GEOMETRY RENDER'):
    im=im.copy();d=ImageDraw.Draw(im);w,h=im.size;s=w/1600
    # very light information treatment; does not cover the product
    d.text((int(42*s),int(28*s)),tag,font=font(max(12,int(15*s))),fill=MUTED)
    d.text((int(40*s),int(56*s)),title,font=font(max(18,int(33*s)),True),fill=FG)
    if subtitle:d.text((int(42*s),int(107*s)),subtitle,font=font(max(11,int(16*s))),fill=MUTED)
    if footer:
        d.line((int(40*s),h-int(49*s),w-int(40*s),h-int(49*s)),fill=(50,65,79),width=1)
        d.text((int(42*s),h-int(35*s)),footer,font=font(max(10,int(13*s))),fill=MUTED)
    return im


def render_still(assembly,output,view_name='hero',size=(1600,1100),time_seconds=0.,captions=True,intent='auto',allow_estimates=False,supersample=2):
    from .truth import assert_renderable,write_truth_report
    report=assert_renderable(assembly,intent,allow_estimates)
    view=assembly.views[view_name]
    studio=Studio(assembly,size=size,section=view.section,floor=view.floor,supersample=supersample,exposure=view.exposure)
    try:
        studio.visible(lambda p:p.group not in view.hide);studio.pose(time_seconds,view.explode);studio.set_view(view);im=studio.render()
        if captions:
            counts=report['tier_counts'];truth=f"{report['resolved_intent'].upper()} / " + ', '.join(f"{k}:{v}" for k,v in counts.items())
            im=labelled(im,view.title or assembly.name.upper(),view.note,
                        f'Actual 3D geometry / {view.projection} PBR / {truth}',
                        tag='CYBR MECHANISM LAB / TRUTH-GATED RENDER')
        output=Path(output);output.parent.mkdir(parents=True,exist_ok=True);im.save(output)
        write_truth_report(report,output.with_suffix('.truth.json'));return output
    finally:studio.close()
