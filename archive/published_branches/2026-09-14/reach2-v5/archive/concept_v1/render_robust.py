from pathlib import Path
import math, os
import vtk
from vtk.util import numpy_support
import numpy as np
from PIL import Image
import imageio.v2 as imageio
import trimesh

ROOT=Path('/mnt/data/torquebias_diff'); STL=ROOT/'stl'; REND=ROOT/'renders'; VID=ROOT/'videos'
REND.mkdir(exist_ok=True); VID.mkdir(exist_ok=True)

MAT={
 'housing':((0.22,0.24,0.27),0.92,0.24),
 'flange':((0.58,0.61,0.65),0.95,0.22),
 'hub':((0.45,0.48,0.52),0.95,0.24),
 'spline':((0.62,0.64,0.67),0.98,0.16),
 'bearing':((0.75,0.77,0.79),1.0,0.11),
 'plate':((0.33,0.35,0.39),0.94,0.26),
 'gear':((0.62,0.64,0.67),0.98,0.17),
 'pinion':((0.46,0.49,0.52),0.98,0.18),
 'shaft':((0.35,0.38,0.40),0.96,0.22),
 'clutch':((0.26,0.28,0.30),0.90,0.30),
 'spring':((0.70,0.42,0.16),0.80,0.27),
 'ring':((0.38,0.41,0.45),0.95,0.22),
 'bolt':((0.15,0.16,0.18),0.86,0.33),
}

def kind(name):
    if name=='housing': return 'housing'
    if 'flange' in name: return 'flange'
    if 'hub' in name: return 'hub'
    if 'output_spline' in name: return 'spline'
    if 'bearing' in name: return 'bearing'
    if 'end_plate' in name: return 'plate'
    if 'side_gear' in name: return 'gear'
    if name.startswith('pinion_') and 'axle' not in name: return 'pinion'
    if 'shaft' in name or 'axle' in name: return 'shaft'
    if 'clutch' in name: return 'clutch'
    if 'spring' in name: return 'spring'
    if 'ring_gear' in name: return 'ring'
    if 'bolt' in name: return 'bolt'
    return 'flange'

def clean_poly(path):
    rd=vtk.vtkSTLReader(); rd.SetFileName(str(path)); rd.Update(); data=rd.GetOutput()
    # large bearing/spring meshes are decimated only for rendering speed, preserving the CAD/STL deliverables.
    if path.stat().st_size>2_000_000:
        dec=vtk.vtkQuadricDecimation(); dec.SetInputData(data); dec.SetTargetReduction(0.62); dec.Update(); data=dec.GetOutput()
    n=vtk.vtkPolyDataNormals(); n.SetInputData(data); n.ComputePointNormalsOn(); n.ComputeCellNormalsOn(); n.SplittingOn(); n.SetFeatureAngle(38); n.ConsistencyOn(); n.AutoOrientNormalsOn(); n.Update()
    out=vtk.vtkPolyData(); out.DeepCopy(n.GetOutput()); return out

def actor_for(path, name, mode='assembled', explode=0):
    mapper=vtk.vtkPolyDataMapper(); mapper.SetInputData(clean_poly(path))
    actor=vtk.vtkActor(); actor.SetMapper(mapper); actor.RotateY(90)
    col,met,rough=MAT[kind(name)]; pr=actor.GetProperty(); pr.SetColor(*col); pr.SetInterpolationToPBR(); pr.SetMetallic(met); pr.SetRoughness(rough)
    if mode=='ghost' and name=='housing': pr.SetOpacity(0.20)
    if mode=='ghost' and ('flange' in name or 'hub' in name): pr.SetOpacity(0.55)
    if mode=='exploded':
        rd=vtk.vtkSTLReader(); rd.SetFileName(str(path)); rd.Update(); b=rd.GetOutput().GetBounds(); zc=(b[4]+b[5])/2
        if zc<-32: dx=-explode
        elif zc>32: dx=explode
        elif zc<-5: dx=-0.44*explode
        elif zc>5: dx=0.44*explode
        else: dx=0
        actor.AddPosition(dx,0,0)
    return actor

def setup_scene(mode='assembled', az=32, el=17, explode=0, size=(1600,900), show_floor=True):
    ren=vtk.vtkRenderer(); ren.SetBackground(0.017,0.020,0.027)
    win=vtk.vtkEGLRenderWindow(); win.SetOffScreenRendering(1); win.AddRenderer(ren); win.SetSize(*size)
    for path in sorted(STL.glob('*.stl')):
        name=path.stem
        if mode=='core' and (name=='housing' or 'flange' in name or 'hub' in name or 'bearing' in name or 'end_plate' in name or 'ring_gear' in name or 'tie_bolt' in name):
            continue
        ren.AddActor(actor_for(path,name,mode,explode))
    if show_floor:
        plane=vtk.vtkPlaneSource(); plane.SetOrigin(-280,-72,-220); plane.SetPoint1(280,-72,-220); plane.SetPoint2(-280,-72,220)
        mp=vtk.vtkPolyDataMapper(); mp.SetInputConnection(plane.GetOutputPort()); ac=vtk.vtkActor(); ac.SetMapper(mp)
        p=ac.GetProperty(); p.SetColor(0.055,0.060,0.068); p.SetInterpolationToPBR(); p.SetMetallic(0.12); p.SetRoughness(0.42); ren.AddActor(ac)
    # Four-area-like point lights. High intensities are deliberate for direct PBR under EGL.
    for pos,col,inten in [
        ((190,220,220),(1.00,0.94,0.84),5.4),
        ((-220,120,130),(0.52,0.68,1.00),3.2),
        ((30,-180,140),(1.00,0.50,0.25),2.2),
        ((0,70,-240),(0.58,0.64,0.74),1.9),
    ]:
        l=vtk.vtkLight(); l.SetPosition(*pos); l.SetFocalPoint(0,0,0); l.SetColor(*col); l.SetIntensity(inten); l.SetPositional(True); l.SetConeAngle(80); ren.AddLight(l)
    head=vtk.vtkLight(); head.SetLightTypeToHeadlight(); head.SetColor(0.88,0.92,1.0); head.SetIntensity(1.15); ren.AddLight(head)
    rad=255 if mode!='exploded' else 345
    ar=math.radians(az); er=math.radians(el)
    cam=ren.GetActiveCamera(); cam.SetPosition(rad*math.cos(er)*math.cos(ar),rad*math.sin(er),rad*math.cos(er)*math.sin(ar)); cam.SetFocalPoint(0,0,0); cam.SetViewUp(0,1,0); cam.SetViewAngle(28)
    ren.ResetCameraClippingRange()
    return win,ren

def grab(win):
    f=vtk.vtkWindowToImageFilter(); f.SetInput(win); f.SetInputBufferTypeToRGB(); f.ReadFrontBufferOff(); f.Update()
    im=f.GetOutput(); d=im.GetDimensions(); arr=numpy_support.vtk_to_numpy(im.GetPointData().GetScalars()).reshape(d[1],d[0],3); return np.flipud(arr)

def save_supersampled(out, mode, az, el, explode=0, final=(1600,900)):
    scale=2; size=(final[0]*scale,final[1]*scale)
    win,ren=setup_scene(mode,az,el,explode,size); win.Render(); arr=grab(win); win.Finalize()
    Image.fromarray(arr).resize(final,Image.Resampling.LANCZOS).save(out,quality=96)

def make_video(out,mode='assembled',explode=0,frames=72,fps=24,size=(960,540)):
    win,ren=setup_scene(mode,0,14,explode,size); cam=ren.GetActiveCamera(); rad=255 if mode!='exploded' else 345
    writer=imageio.get_writer(str(out),fps=fps,codec='libx264',quality=8,macro_block_size=None)
    for i in range(frames):
        az=360*i/frames if mode!='exploded' else 22+250*i/max(1,frames-1); el=14+2.5*math.sin(2*math.pi*i/frames)
        ar=math.radians(az); er=math.radians(el); cam.SetPosition(rad*math.cos(er)*math.cos(ar),rad*math.sin(er),rad*math.cos(er)*math.sin(ar)); cam.SetViewUp(0,1,0); ren.ResetCameraClippingRange(); win.Render(); writer.append_data(grab(win))
    writer.close(); win.Finalize()

def export_glb():
    scene=trimesh.Scene(); T=trimesh.transformations.rotation_matrix(math.radians(90),(0,1,0))
    for path in sorted(STL.glob('*.stl')):
        name=path.stem; m=trimesh.load_mesh(path,force='mesh'); m.apply_transform(T); rgb=(np.array(MAT[kind(name)][0])*255).astype(np.uint8); m.visual.face_colors=np.tile(np.r_[rgb,255],(len(m.faces),1)); scene.add_geometry(m,node_name=name,geom_name=name)
    scene.export(ROOT/'torque_biasing_differential.glb')

if __name__=='__main__':
    task=os.environ.get('TASK','hero')
    if task=='hero': save_supersampled(REND/'assembled_hero.png','assembled',58,17,final=(1600,900))
    elif task=='stills':
        save_supersampled(REND/'assembled_hero.png','assembled',58,17)
        save_supersampled(REND/'rear_hero.png','assembled',238,15)
        save_supersampled(REND/'transparent_cutaway.png','ghost',56,18)
        save_supersampled(REND/'exploded_hero.png','exploded',48,15,explode=72)
        save_supersampled(REND/'front_axial.png','assembled',1,1)
    elif task=='videos':
        make_video(VID/'turntable.mp4','assembled',frames=72)
        make_video(VID/'exploded_orbit.mp4','exploded',explode=72,frames=60)
    elif task=='glb': export_glb()
    print('complete',task)
