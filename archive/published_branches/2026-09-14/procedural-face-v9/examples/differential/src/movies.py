"""New frame-by-frame inspection and gear-constrained kinematic video recordings."""
from __future__ import annotations
from render import *
import hashlib,csv
FPS=24;SIZE=(1280,720)

def smooth(t):t=np.clip(t,0,1);return float(t*t*(3-2*t))
def lerp(a,b,t):return a+(b-a)*t


def tracker(name,x,ri,ro,motion):
    aa=np.linspace(.6,.88,15);v=[];f=[]
    for a in aa:v.extend([(x,ri*math.cos(a),ri*math.sin(a)),(x,ro*math.cos(a),ro*math.sin(a))])
    for j in range(len(aa)-1):k=j*2;f.extend([(k,k+1,k+3),(k,k+3,k+2)])
    return make_part(name,v,f,3,motion,group='motion_annotation',role='render-only rotation indicator')


def hud(im,title,sub,tag='GEOMETRY INSPECTION',speeds=None,progress=0):
    im=im.copy();d=ImageDraw.Draw(im)
    d.rectangle((0,0,1280,104),fill=(10,15,21));d.text((30,15),tag,font=font(13),fill=MUTED);d.text((28,39),title,font=font(26,True),fill=FG);d.text((30,79),sub,font=font(13),fill=MUTED)
    if speeds is None:
        d.rectangle((0,677,1280,720),fill=(10,15,21));d.text((30,687),'Original 81-mesh reconstruction / inspection offsets are not an assembly-path simulation.',font=font(13),fill=MUTED)
    else:
        d.rectangle((0,611,1280,720),fill=(10,15,21))
        for j,(label,val,color) in enumerate(zip(['CARRIER / INPUT','LEFT OUTPUT','RIGHT OUTPUT'],speeds,[FG,BLUE,GOLD])):
            x=32+j*287;d.text((x,626),label,font=font(12),fill=MUTED);d.text((x,648),f'{val:+.1f} rpm',font=font(25,True),fill=color)
        residual=speeds[1]+speeds[2]-2*speeds[0]
        d.text((924,630),'L + R = 2 x INPUT',font=font(16,True),fill=FG)
        d.text((924,658),f'residual: {residual:.1e} rpm',font=font(12),fill=MUTED)
        d.text((32,699),'Prescribed gear-constrained motion. Contact forces, torque bias and load response are not simulated.',font=font(12),fill=MUTED)
    d.rectangle((0,717,int(1280*progress),720),fill=(102,151,182))
    return im


class Encoder:
    def __init__(self,path):
        self.path=path
        self.proc=subprocess.Popen(['ffmpeg','-y','-v','error','-f','rawvideo','-pix_fmt','rgb24','-s','1280x720','-r',str(FPS),'-i','-','-an','-c:v','libx264','-preset','fast','-crf','17','-pix_fmt','yuv420p','-movflags','+faststart','-threads','2',str(path)],stdin=subprocess.PIPE)
        self.count=0;self.hashes=[]
    def append(self,im):
        a=np.asarray(im,dtype=np.uint8);self.proc.stdin.write(a.tobytes());self.hashes.append(hashlib.sha256(a.tobytes()).hexdigest());self.count+=1
    def close(self):
        self.proc.stdin.close();code=self.proc.wait()
        if code!=0:raise RuntimeError(f'ffmpeg failed: {code}')
        return dict(path=str(self.path.relative_to(ROOT)),frames=self.count,unique_frames=len(set(self.hashes)),resolution=[1280,720],fps=FPS,seconds=self.count/FPS,sha256=hashlib.sha256(self.path.read_bytes()).hexdigest())


def inspection(preview=False):
    ref=load_parts(GEOM/'reference_parts.npz');s=Studio(ref,SIZE);duration=12.;count=int(duration*FPS)
    writer=None if preview else Encoder(V/'01_exploded_inspection.mp4');start=time.time();saved=[]
    indices=[0,72,130,160,205,263] if preview else range(count)
    for i in indices:
        t=i/FPS
        if t<2.:
            e=0.;a=lerp(232,245,t/2);scale=88;target=(0,0,6);title='01 / THE EXISTING RECONSTRUCTION';sub='Short hollow spline, eight-hole flange, sculpted windowed carrier.'
        elif t<5.5:
            e=smooth((t-2)/3.5);a=lerp(245,259,e);scale=lerp(88,206,e);target=(5*e,0,6+33*e);title='02 / COMPONENT SEPARATION';sub='Every moving piece is a separately transformed mesh. Carrier lifted for visibility.'
        elif t<9.2:
            e=1.;a=lerp(259,282,(t-5.5)/3.7);scale=206;target=(5,0,39);title='03 / EXPLODED ORBIT';sub='Individual plates, spring, bearing races, balls, sleeves, flanges and housing.'
        else:
            e=1.-smooth((t-9.2)/2.8);a=lerp(282,235,1-e);scale=lerp(88,206,e);target=(5*e,0,6+33*e);title='04 / RETURN TO ASSEMBLED';sub='The original source components return to their exact un-exploded positions.'
        s.pose(explode=e);s.set_camera(az=a,el=22,scale=scale,target=target);im=hud(s.render(),title,sub,progress=t/duration)
        if i in [0,72,130,160,205,263]:
            path=V/f'inspection_frame_{i:04d}.png';im.save(path);saved.append(str(path.relative_to(ROOT)))
        if writer:writer.append(im)
        if not preview and i%48==0:print('inspection',i,'/',count,'elapsed',round(time.time()-start,1),flush=True)
    s.close()
    if writer:
        report=writer.close();report['preview_frames']=saved;report['renderer']='VTK PBR, analytic studio IBL, SSAO, Mesa EGL';(ROOT/'validation'/'inspection_video.json').write_text(json.dumps(report,indent=2));print('INSPECTION COMPLETE',report,flush=True)


def velocity(t):
    """Prescribed speeds, smoothly transitioned; all are demonstration RPM."""
    if t<3:return 6.,0.
    if t<3.8:return 6.,-3.*smooth((t-3)/.8)
    if t<7:return 6.,-3.
    if t<7.8:return 6.,-3.-3.*smooth((t-7)/.8)
    if t<11:return 6.,-6.
    if t<11.8:
        a=smooth((t-11)/.8);return 6*(1-a),-6+2*a
    return 0.,-4.


def trajectory(duration=19.):
    # Midpoint integration of these prescribed smooth speeds; exact rigid-body
    # angular constraints are applied to all components at every output frame.
    dt=1/(FPS*20);steps=int(duration/dt)+1;c=np.zeros(steps);d=np.zeros(steps)
    for i in range(1,steps):
        wc,wd=velocity((i-.5)*dt);c[i]=c[i-1]+wc*2*math.pi/60*dt;d[i]=d[i-1]+wd*2*math.pi/60*dt
    return c[::20],d[::20]


def operational(preview=False):
    work=load_parts(GEOM/'working_variant_parts.npz');marks=[tracker('Left_tracking_patch',-35.012,16.5,21.5,'left'),tracker('Right_tracking_patch',25.012,17.5,21.5,'right')];work+=marks
    s=Studio(work,SIZE)
    for p,a in zip(s.parts,s.actors):
        if p.name=='Left_tracking_patch':a.GetProperty().SetColor(.08,.43,.72);a.GetProperty().SetMetallic(.1);a.GetProperty().SetRoughness(.45)
        if p.name=='Right_tracking_patch':a.GetProperty().SetColor(.80,.33,.06);a.GetProperty().SetMetallic(.1);a.GetProperty().SetRoughness(.45)
    duration=19.;count=int(duration*FPS);cs,ds=trajectory(duration);writer=None if preview else Encoder(V/'02_differential_in_motion.mp4');start=time.time();records=[]
    indices=[0,96,156,216,276,336,420] if preview else range(count)
    for i in indices:
        t=i/FPS;c=float(cs[i]);d=float(ds[i]);wc,wd=velocity(t)
        if t<3:
            visible=lambda p: True
            title='01 / BOTH OUTPUTS AT EQUAL SPEED';sub='Original exterior around the new core. Pinions have zero spin relative to the carrier.'
            az=232+4*t;el=22;scale=89;target=(0,0,6)
        elif t<7:
            visible=lambda p:p.group in ['kinematic_core','shaft','motion_annotation','front_hub','rear_hub'] and 'support_plate' not in p.name
            title='02 / DIFFERENTIAL ACTION';sub='The paired pinions turn relative to the carrier as one output slows and the other speeds up.'
            az=242+4*(t-3);el=24;scale=79;target=(0,0,5)
        elif t<11:
            visible=lambda p:p.group in ['kinematic_core','shaft','motion_annotation','front_hub','rear_hub'] and 'support_plate' not in p.name
            title='03 / LEFT OUTPUT HELD';sub='One output reaches zero; the other reaches twice the input speed. This is a prescribed bench condition.'
            az=258-7*(t-7);el=23;scale=79;target=(0,0,5)
        elif t<15:
            visible=lambda p:p.group in ['kinematic_core','shaft','motion_annotation'] and 'support_plate' not in p.name
            title='04 / CARRIER HELD';sub='With the carrier stationary, the two outputs counter-rotate through the connecting gear train.'
            az=234+3*(t-11);el=20;scale=64;target=(-2,0,4)
        else:
            # All remaining objects are drawn at their current dynamic transforms.
            # Only one pinion pair is retained to expose the actual coupling.
            visible=lambda p: p.name.startswith('K01_') or p.name.startswith('K02_') or p.name.startswith('K_Pair1_') or p.group=='motion_annotation'
            title='05 / CENTRAL COUPLING CLOSE-UP';sub='One pinion pair isolated. Its spur stages counter-rotate while the helical stages connect both outputs.'
            # Carrier is stopped at a nonzero angle. Look along the transformed
            # outward radial direction of this pair, rather than into the back.
            ca=(155+math.degrees(c))%360
            az=(ca+7)+10*math.sin((t-15)*.4);el=27
            angle=math.radians(ca);target=(-3.,24*math.cos(angle),24*math.sin(angle));scale=43
        s.visible(visible);s.pose(c=c,d=d);s.set_camera(az=az,el=el,scale=scale,target=target)
        speeds=[wc,wc+wd,wc-wd]
        im=hud(s.render(),title,sub,tag='NEW KINEMATIC CORE / ORIGINAL EXTERIOR',speeds=speeds,progress=t/duration)
        if i in [0,96,156,216,276,336,420]:im.save(V/f'motion_frame_{i:04d}.png')
        if writer:writer.append(im)
        records.append(dict(frame=i,time=t,carrier_angle=c,differential_angle=d,left_angle=c+d,right_angle=c-d,pinionA_relative=-RATIO*d,pinionB_relative=RATIO*d,carrier_rpm=wc,left_rpm=wc+wd,right_rpm=wc-wd))
        if not preview and i%48==0:print('motion',i,'/',count,'elapsed',round(time.time()-start,1),flush=True)
    s.close()
    if writer:
        report=writer.close();report.update(renderer='VTK PBR, analytic studio IBL, SSAO, Mesa EGL',model='119-component alternative with 44 new kinematic-core meshes; original reference remains separate',simulation='Prescribed gear-constrained rigid-body motion, not contact force dynamics',max_speed_constraint_residual_rpm=max(abs(r['left_rpm']+r['right_rpm']-2*r['carrier_rpm']) for r in records));(ROOT/'validation'/'operational_video.json').write_text(json.dumps(report,indent=2))
        with open(ROOT/'validation'/'frame_kinematics.csv','w') as f:
            writer=csv.DictWriter(f,fieldnames=list(records[0]));writer.writeheader();writer.writerows(records)
        print('OPERATIONAL COMPLETE',report,flush=True)

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('task',choices=['inspection','operational','preview']);args=parser.parse_args()
    if args.task=='preview':inspection(True);operational(True)
    else:globals()[args.task]()
