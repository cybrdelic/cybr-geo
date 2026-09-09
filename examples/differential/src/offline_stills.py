"""Monte Carlo path-traced stills derived from the very same inspection meshes."""
from render import *
import finish_render as filt

def section_parts(parts):
    result=[]
    for p in parts:
        q=clip_closed(polydata(p));tri=vtk.vtkTriangleFilter();tri.SetInputData(q);tri.Update();q=tri.GetOutput()
        if q.GetNumberOfPoints()==0:continue
        v=vtk_to_numpy(q.GetPoints().GetData()).copy();fa=vtk_to_numpy(q.GetPolys().GetConnectivityArray()).reshape(-1,3).copy();n=vtk_to_numpy(q.GetPointData().GetNormals()).copy()
        result.append(Part(p.name,v,fa,n,p.material,p.group))
    return result


def run():
    ref=load_parts(GEOM/'reference_parts.npz');core=load_parts(GEOM/'kinematic_core_parts.npz')
    recipes=[
      ('pt_exploded',ref,[transform(p,explode=1.) for p in ref],2200,1400,48,258,21,365,(5,0,33),2.,'EXPLODED / PATH-TRACED','Original 81 component meshes, independently separated for inspection.','REFERENCE GEOMETRY / MONTE CARLO PATH TRACING'),
      ('pt_internal',[p for p in ref if p.group not in ['carrier','marking','front_flange','rear_flange','front_hub','rear_hub']],None,1800,1200,48,233,28,140,(0,0,4),1.,'INTERNAL ASSEMBLY / PATH-TRACED','The original visual core, with housing and interfaces removed.','REFERENCE GEOMETRY / NO CONNECTING PINION TRAIN'),
      ('pt_kinematic_core',[p for p in core if 'support_plate' not in p.name],None,1800,1200,64,232,25,135,(0,0,4),1.,'KINEMATIC CORE / PATH-TRACED','Three paired pinion trains connect the two side gears.','NEW INTERNAL ALTERNATIVE / PRESERVED EXTERIOR'),
      ('pt_section',section_parts(ref),None,1800,1200,48,263,12,165,(0,0,6),1.,'LONGITUDINAL SECTION / PATH-TRACED','Capped geometric section through the original reconstruction.','REFERENCE GEOMETRY / REAL SECTION CUT'),
    ]
    reports=[]
    for name,parts,poses,w,h,spp,az,el,scale,target,studio,title,sub,tag in recipes:
        mesh=GEOM/(name+'.meshbin');export_meshbin(parts,mesh,poses);ppm=R/(name+'.ppm');out=R/(name+'.png')
        cmd=[str(ROOT/'src'/'pathtrace'),str(mesh),str(ppm),'--w',str(w),'--h',str(h),'--spp',str(spp),'--depth','7','--threads','2','--ortho','--az',str(az),'--el',str(el),'--scale',str(scale),'--tx',str(target[0]),'--ty',str(target[1]),'--tz',str(target[2]),'--studio-scale',str(studio),'--no-floor']
        st=time.time();print('PATH TRACE',name,flush=True)
        with open(ROOT/'logs'/(name+'.log'),'w') as log:subprocess.run(cmd,stdout=log,stderr=log,check=True)
        arr=filt.read_pfm(str(ppm)+'.pfm')
        with open(str(ppm)+'.guides','rb') as f:
            fw,fh=np.fromfile(f,'<u4',2);guide=np.fromfile(f,'<f4').reshape(fh,fw,9)
        Image.fromarray(filt.tonemap(arr)).save(R/(name+'_raw.png'));var=guide[:,:,7].copy()
        for i in range(3):arr,var=filt.atrous(arr,guide,var,2**i,i)
        clean=Image.fromarray(filt.tonemap(arr));clean.save(R/(name+'_clean.png'))
        labelled(clean,title,sub,'Triangle-geometry path tracing / seven-bounce limit / classical non-neural denoising.',tag).save(out)
        reports.append(dict(file=str(out.relative_to(ROOT)),resolution=[w,h],spp=spp,max_bounces=7,seconds=time.time()-st,triangle_count=sum(len(p.faces) for p in parts),command=cmd));print('DONE',name,round(time.time()-st,1),flush=True)
        # Retain PNG raw radiance preview; large transient transport buffers are regenerable.
        for fp in [ppm,Path(str(ppm)+'.pfm'),Path(str(ppm)+'.guides'),mesh]:fp.unlink(missing_ok=True)
        (ROOT/'validation'/'pathtrace_stills.json').write_text(json.dumps(reports,indent=2))
    print('ALL PATH TRACED STILLS COMPLETE',flush=True)
if __name__=='__main__':run()
