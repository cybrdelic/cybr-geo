"""Four-second, absolutely stationary-camera proof of independent gear motion."""
from movies import *
if __name__=='__main__':
    core=load_parts(GEOM/'kinematic_core_parts.npz')
    core += [tracker('Left_tracking_patch',-35.012,16.5,21.5,'left'),tracker('Right_tracking_patch',25.012,17.5,21.5,'right')]
    s=Studio(core,SIZE)
    for p,a in zip(s.parts,s.actors):
        if p.name=='Left_tracking_patch':a.GetProperty().SetColor(.08,.43,.72);a.GetProperty().SetMetallic(.1)
        if p.name=='Right_tracking_patch':a.GetProperty().SetColor(.80,.33,.06);a.GetProperty().SetMetallic(.1)
    s.visible(lambda p:'support_plate' not in p.name)
    s.set_camera(az=237,el=24,scale=75,target=(0,0,4))
    enc=Encoder(V/'03_fixed_camera_motion.mp4');start=time.time()
    for i in range(96):
        t=i/FPS;s.pose(c=0,d=-4*2*math.pi/60*t)
        im=hud(s.render(),'06 / FIXED-CAMERA MOTION CHECK','Camera and carrier remain stationary. Only the two output groups and their linked pinions rotate.',tag='NEW KINEMATIC CORE / NO CAMERA MOVEMENT',speeds=[0.,-4.,4.],progress=t/4)
        enc.append(im)
        if i in [0,32,64,95]:im.save(V/f'fixed_frame_{i:04d}.png')
        if i%24==0:print('fixed',i,'/ 96',round(time.time()-start,1),flush=True)
    s.close();report=enc.close();report['camera_changes']=0;report['carrier_angle_radians']=0;report['output_rpm']=[-4,4];(ROOT/'validation'/'fixed_camera_video.json').write_text(json.dumps(report,indent=2));print('FIXED CAMERA COMPLETE',flush=True)
