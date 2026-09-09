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
from model import *

R=ROOT/'renders';V=ROOT/'videos';P=ROOT/'parts'
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
    def __init__(self,parts:list[Part],size=(1280,720),ao=True,section=None):
        self.parts=parts;self.actors=[];self.buffers=[]
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
            actor=vtk.vtkActor();actor.SetMapper(mapper);pr=actor.GetProperty();m=MATERIALS[p.material]
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

    def pose(self,c=0,d=0,explode=0):
        for p,a in zip(self.parts,self.actors):
            t=transform(p,c,d,explode);matrix=vtk.vtkMatrix4x4()
            for j in range(4):
                for k in range(4):matrix.SetElement(j,k,t[j,k])
            a.SetUserMatrix(matrix)
        self.ren.ResetCameraClippingRange()

    def visible(self,predicate):
        for p,a in zip(self.parts,self.actors):a.SetVisibility(bool(predicate(p)))
        self.ren.ResetCameraClippingRange()

    def render(self):
        self.win.Render();self.capture.Modified();self.capture.Update();im=self.capture.GetOutput();w,h,_=im.GetDimensions();a=vtk_to_numpy(im.GetPointData().GetScalars()).reshape(h,w,3)
        return Image.fromarray(np.flipud(a).copy())

    def close(self):self.win.Finalize()


def labelled(im,title,subtitle='',footer='',tag='REFERENCE / 81 ORIGINAL MESHES'):
    im=im.copy();d=ImageDraw.Draw(im);w,h=im.size;s=w/1600
    d.text((int(42*s),int(28*s)),tag,font=font(max(12,int(15*s))),fill=MUTED)
    d.text((int(40*s),int(56*s)),title,font=font(max(18,int(33*s)),True),fill=FG)
    if subtitle:d.text((int(42*s),int(107*s)),subtitle,font=font(max(11,int(16*s))),fill=MUTED)
    if footer:
        d.line((int(40*s),h-int(49*s),w-int(40*s),h-int(49*s)),fill=(50,65,79),width=1)
        d.text((int(42*s),h-int(35*s)),footer,font=font(max(10,int(13*s))),fill=MUTED)
    return im


def test():
    ref=load_parts(GEOM/'reference_parts.npz');s=Studio(ref,(1440,1080));s.set_camera(az=235,el=20,scale=91,target=(0,0,2));start=time.time();im=s.render();print('first frame',time.time()-start);im.save(R/'pbr_test.png');start=time.time();s.pose(c=.1);s.render().save(R/'pbr_test2.png');print('second frame',time.time()-start);s.close()
    core=load_parts(GEOM/'kinematic_core_parts.npz');s=Studio(core,(1440,1000));s.visible(lambda p:'support_plate' not in p.name);s.set_camera(az=237,el=23,scale=58);s.render().save(R/'kinematic_test.png');s.close()


def stills():
    ref=load_parts(GEOM/'reference_parts.npz');core=load_parts(GEOM/'kinematic_core_parts.npz');work=load_parts(GEOM/'working_variant_parts.npz')
    # All new views are independently rendered geometry, at their output resolution.
    s=Studio(ref,(1800,1200));s.visible(lambda p:p.group not in ['carrier','marking','front_flange','rear_flange','front_hub','rear_hub']);s.set_camera(az=233,el=28,scale=70,target=(0,0,4));im=s.render();labelled(im,'INTERNAL ASSEMBLY','Exterior removed; original gear masses, hollow sleeves, bearings and clutch stack.','This reference core is a visual layout. It has no connecting pinion train.').save(R/'reference_internal.png');s.close()
    s=Studio(ref,(1800,1200),section='all');s.set_camera(az=263,el=12,scale=82,target=(0,0,6));labelled(s.render(),'LONGITUDINAL SECTION','A real geometric cut through the existing model; no invented hidden surfaces.','Capped section geometry / same original 81 components').save(R/'reference_section.png');s.close()
    s=Studio(ref,(2200,1400));s.pose(explode=1);s.set_camera(az=258,el=21,scale=181,target=(5,0,32));im=s.render();labelled(im,'EXPLODED ASSEMBLY','Carrier lifted away; hubs, flanges, races, individual clutch plates and preload spring separated.','Positions are inspection offsets, not a validated disassembly sequence.').save(R/'reference_exploded.png');s.set_camera(az=290,el=32,scale=192,target=(5,0,35));labelled(s.render(),'EXPLODED / REVERSE VIEW','The same component separation seen from the opposite end.','Unchanged meshes; alternate camera.').save(R/'reference_exploded_reverse.png');s.close()
    selected=[p for p in ref if 'Rear_clutch' in p.name or p.name=='Bronze_wave_preload_spring' or p.name.startswith('Rear_bearing') or p.name=='Rear_thrust_ring']
    s=Studio(selected,(1800,1200));
    for p in selected:
        if p.name.startswith('Rear_clutch_plate'):p.explode=np.array([(int(p.name[-2:])-4)*8.,0,0])
        elif p.name=='Bronze_wave_preload_spring':p.explode=np.array([0,-50.,18.])
        elif p.name.startswith('Rear_bearing'):p.explode=np.array([55.,0,0])
        elif p.name=='Rear_clutch_retainer':p.explode=np.array([45.,0,0])
        else:p.explode=np.array([-40.,0,0])
    s.pose(explode=1);s.set_camera(az=239,el=27,scale=81,target=(42,-6,4));labelled(s.render(),'CLUTCH / PRELOAD / BEARING','Seven discrete plates, modeled wave spring, two races, retainer and twenty balls.','Reference geometry, separated for inspection.').save(R/'clutch_bearing_detail.png');s.close()
    s=Studio(core,(1800,1200));s.visible(lambda p:'support_plate' not in p.name);s.set_camera(az=232,el=25,scale=63,target=(0,0,4));labelled(s.render(),'KINEMATIC CORE / PAIRED PINIONS','Two 48-tooth side gears; three pairs of 18-tooth helical pinions with central spur couplings.','New internal alternative. Original exterior is unchanged; load behavior is not solved.','KINEMATIC REDESIGN / 44 NEW COMPONENTS').save(R/'kinematic_core.png');s.set_camera(az=180,el=0,scale=50,target=(-3,0,0));labelled(s.render(),'KINEMATIC CORE / END VIEW','Six planet axes on a 31.02 mm orbit; three independently indexed coupling pairs.','Helical stages have matched module and opposite hands.','KINEMATIC REDESIGN / NOT THE ORIGINAL CORE').save(R/'kinematic_end.png');s.close()
    s=Studio(work,(1800,1200),section='shell');s.set_camera(az=242,el=22,scale=86,target=(0,0,7));labelled(s.render(),'ALTERNATIVE CORE IN THE ORIGINAL SHELL','Housing section reveals the new gear train inside the preserved exterior.','Ideal rigid-body kinematic model; no claim of manufacturing or torque-bias validation.','KINEMATIC REDESIGN / ORIGINAL EXTERIOR').save(R/'working_section.png');s.close()
    print('STILLS COMPLETE',flush=True)


def part_catalog():
    ref=load_parts(GEOM/'reference_parts.npz');s=Studio(ref,(760,640));index=[]
    for i,p in enumerate(ref):
        s.visible(lambda q:q.name==p.name);b=p.bounds;center=b.mean(0);dim=np.ptp(b,axis=0);rad=np.linalg.norm(dim)*.5;scale=max(rad*.90,1.)
        # Flat rings read best at a clear three-quarter view; even small machining rings receive a full view.
        s.set_camera(az=232,el=25,scale=scale*1.08,target=center);im=s.render()
        d=ImageDraw.Draw(im);d.text((25,22),f'{i+1:02d} / {len(ref):02d}',font=font(18,True),fill=BLUE)
        title=p.name.replace('_',' ');words=title.split();lines=[];line=''
        for word in words:
            if d.textlength((line+' '+word).strip(),font=font(19,True))>710:lines.append(line);line=word
            else:line=(line+' '+word).strip()
        lines.append(line)
        for j,line in enumerate(lines):d.text((25,568+j*24),line,font=font(19,True),fill=FG)
        fname=f'{i+1:02d}_{p.name}.png';im.save(P/fname)
        index.append(dict(index=i+1,name=p.name,image=fname,group=p.group,triangles=len(p.faces),size_mm=np.round(dim,4).tolist()))
        if (i+1)%10==0:print('parts',i+1,flush=True)
    s.close();(P/'index.json').write_text(json.dumps(index,indent=2))
    # Twenty distinct component families, rather than a wall of identical balls.
    reps=[0,1,2,5,6,10,12,14,15,16,17,19,21,22,28,29,30,31,32,52]
    plate=Image.new('RGB',(3040,3400),BG);d=ImageDraw.Draw(plate);d.text((40,25),'PART-BY-PART / SELECTED COMPONENT FAMILIES',font=font(38,True),fill=FG);d.text((42,85),'All 81 individual renders, including every bearing ball and surface marking, are in the package.',font=font(21),fill=MUTED)
    for j,i in enumerate(reps):
        tile=Image.open(P/index[i]['image']);plate.paste(tile,((j%4)*760,150+(j//4)*640))
    plate.save(R/'parts_overview.png')
    for start in range(0,len(ref),20):
        plate=Image.new('RGB',(3040,3420),BG);d=ImageDraw.Draw(plate);d.text((40,25),f'COMPONENT ATLAS / {start+1:02d}-{min(len(ref),start+20):02d}',font=font(40,True),fill=FG)
        for j,item in enumerate(index[start:start+20]):plate.paste(Image.open(P/item['image']),((j%4)*760,150+(j//4)*640))
        plate.save(P/f'atlas_{start//20+1:02d}.png')
    print('CATALOG COMPLETE',flush=True)

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('task',choices=['test','stills','parts']);args=parser.parse_args();globals()[{'test':'test','stills':'stills','parts':'part_catalog'}[args.task]]()
