"""A3 white-background technical sheets from geometry, not image generation.

Analytic CAD: OpenCascade hidden-line removal with visible/hidden edges sampled
at a declared chordal deflection. Outputs SVG, PDF and millimetre DXF layers.
Mesh-only assemblies: a separately labelled feature-line projection fallback,
never silently represented as analytic CAD or manufacturing certification.
"""
from __future__ import annotations
from dataclasses import dataclass
from pathlib import Path
import html,json,math
import numpy as np
import cadquery as cq
from OCP.HLRBRep import HLRBRep_Algo,HLRBRep_HLRToShape
from OCP.HLRAlgo import HLRAlgo_Projector
from OCP.gp import gp_Ax2,gp_Pnt,gp_Dir
from OCP.BRepLib import BRepLib
from .core import pose_cad

@dataclass(frozen=True)
class Projection:
    name: str
    normal: tuple
    right: tuple
    center: tuple
    label: str

PROJECTIONS=[
    Projection('top',(0,0,1),(1,0,0),(109,83),'TOP / +Z'),
    Projection('iso',(1,-1,1),(1,1,0),(310,83),'ISOMETRIC'),
    Projection('side',(0,-1,0),(1,0,0),(109,193),'SIDE / -Y'),
    Projection('end',(1,0,0),(0,1,0),(310,193),'OUTPUT FACE / +X'),
]

def basis(projection):
    n=np.array(projection.normal,float);n/=np.linalg.norm(n)
    r=np.array(projection.right,float);r-=n*np.dot(r,n);r/=np.linalg.norm(r)
    return r,np.cross(n,r),n

def projected_edges(shape,projection,deflection=.025):
    r,u,n=basis(projection)
    axes=gp_Ax2(gp_Pnt(0,0,0),gp_Dir(*n),gp_Dir(*r))
    algo=HLRBRep_Algo();algo.Add(shape.wrapped);algo.Projector(HLRAlgo_Projector(axes));algo.Update();algo.Hide()
    result=HLRBRep_HLRToShape(algo);lines=[]
    for hidden,functions in [(False,['VCompound','Rg1LineVCompound','OutLineVCompound']),
                             (True,['HCompound','OutLineHCompound'])]:
        for fn in functions:
            raw=getattr(result,fn)()
            if raw.IsNull():continue
            BRepLib.BuildCurves3d_s(raw,1e-7)
            for edge in cq.Shape.cast(raw).Edges():
                points,_=edge.sample(2 if edge.geomType()=='LINE' else float(deflection))
                arr=np.array([[p.x,p.y] for p in points])
                if len(arr)<2:continue
                if (edge.startPoint()-edge.endPoint()).Length<1e-7:arr=np.vstack([arr,arr[0]])
                lines.append((hidden,arr))
    return lines

def mesh_projected_edges(assembly,projection):
    """Feature + silhouette projections. Hidden line removal not claimed for fallback."""
    import vtk
    from vtk.util.numpy_support import vtk_to_numpy
    from .render import polydata
    r,u,n=basis(projection);lines=[]
    for p in assembly.parts:
        data=polydata(p)
        T=assembly.pose(p)
        transform=vtk.vtkTransform();matrix=vtk.vtkMatrix4x4()
        for row in range(4):
            for col in range(4):matrix.SetElement(row,col,T[row,col])
        transform.SetMatrix(matrix)
        tf=vtk.vtkTransformPolyDataFilter();tf.SetInputData(data);tf.SetTransform(transform);tf.Update()
        clean=vtk.vtkCleanPolyData();clean.SetInputData(tf.GetOutput());clean.SetTolerance(1e-7);clean.Update()
        camera=vtk.vtkCamera();camera.SetPosition(*(n*10000));camera.SetFocalPoint(0,0,0);camera.SetViewUp(*u);camera.ParallelProjectionOn()
        feature=vtk.vtkFeatureEdges();feature.SetInputConnection(clean.GetOutputPort());feature.FeatureEdgesOn();feature.BoundaryEdgesOn();feature.NonManifoldEdgesOff();feature.ManifoldEdgesOff();feature.SetFeatureAngle(35);feature.Update()
        silhouette=vtk.vtkPolyDataSilhouette();silhouette.SetInputConnection(clean.GetOutputPort());silhouette.SetCamera(camera);silhouette.SetEnableFeatureAngle(0);silhouette.Update()
        for filt in [feature,silhouette]:
            q=filt.GetOutput()
            if not q.GetNumberOfPoints():continue
            pts=vtk_to_numpy(q.GetPoints().GetData())
            arr=np.column_stack([pts@r,pts@u]);cells=q.GetLines();cells.InitTraversal();ids=vtk.vtkIdList()
            while cells.GetNextCell(ids):
                indices=[ids.GetId(i) for i in range(ids.GetNumberOfIds())]
                if len(indices)>=2:lines.append((False,arr[indices]))
    return lines

class Sheet:
    def __init__(self,width=420,height=297):
        self.width=width;self.height=height;self.items=[];self.dxf_lines=[];self.dxf_text=[]
    def line(self,points,layer='VISIBLE',width=.22,closed=False):
        a=np.asarray(points,float)
        if closed:a=np.vstack([a,a[0]])
        dash=' stroke-dasharray="1.7 1.1"' if layer=='HIDDEN' else ' stroke-dasharray="5 1 1 1"' if layer=='CENTER' else ''
        col='#8a9197' if layer=='HIDDEN' else '#252c33'
        coordinates=' '.join(f'{x:.4f},{y:.4f}' for x,y in a)
        self.items.append(f'<polyline points="{coordinates}" fill="none" stroke="{col}" stroke-width="{width}" stroke-linejoin="round"{dash}/>')
        self.dxf_lines.append((layer,a.copy()))
    def text(self,x,y,text,size=3.2,bold=False,anchor='start'):
        text=str(text);self.items.append(f'<text x="{x}" y="{y}" font-family="DejaVu Sans, sans-serif" font-size="{size}" font-weight="{700 if bold else 400}" fill="#252c33" text-anchor="{anchor}">{html.escape(text)}</text>')
        self.dxf_text.append((x,y,text,size,anchor))
    def box(self,x,y,w,h,width=.25):self.line([(x,y),(x+w,y),(x+w,y+h),(x,y+h)],'BORDER',width,True)
    def arrow(self,p,d,size=1.6):
        p=np.asarray(p,float);d=np.asarray(d,float);d/=np.linalg.norm(d);n=np.array([-d[1],d[0]])
        self.line([p+size*d+.42*n,p,p+size*d-.42*n],'DIM',.16)
    def dimension(self,p,q,offset=8.,orientation='horizontal',label=None):
        p=np.asarray(p,float);q=np.asarray(q,float)
        if orientation=='horizontal':
            a=np.array([p[0],max(p[1],q[1])+offset]);b=np.array([q[0],a[1]])
            self.line([p+[0,1.2],a+[0,2]],'DIM',.13);self.line([q+[0,1.2],b+[0,2]],'DIM',.13)
            self.line([a,b],'DIM',.15);self.arrow(a,b-a);self.arrow(b,a-b)
            self.text((a[0]+b[0])/2,a[1]-1.4,label if label is not None else f'{abs(q[0]-p[0]):.2f}',3.1,anchor='middle')
        else:
            a=np.array([min(p[0],q[0])-offset,p[1]]);b=np.array([a[0],q[1]])
            self.line([p+[-1.2,0],a+[-2,0]],'DIM',.13);self.line([q+[-1.2,0],b+[-2,0]],'DIM',.13)
            self.line([a,b],'DIM',.15);self.arrow(a,b-a);self.arrow(b,a-b)
            self.text(a[0]-2,(a[1]+b[1])/2,label if label is not None else f'{abs(q[1]-p[1]):.2f}',3.1,anchor='end')
    def leader(self,start,elbow,end,text):
        self.line([start,elbow,end],'DIM',.15);self.arrow(start,np.asarray(elbow)-start);self.text(end[0]+1,end[1]-1.0,text,2.85)
    def save(self,path):
        import cairosvg,ezdxf
        path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
        for layer,points in self.dxf_lines:
            if np.any(points[:,0]<0) or np.any(points[:,0]>self.width) or np.any(points[:,1]<0) or np.any(points[:,1]>self.height):
                raise ValueError(f'{layer} linework leaves the sheet. Check annotation origin or requested scale.')
        for x,y,text,size,anchor in self.dxf_text:
            if not (0<=x<=self.width and 0<=y<=self.height):
                raise ValueError(f'Text anchor leaves the sheet: {text}')
        svg=f'<svg xmlns="http://www.w3.org/2000/svg" width="{self.width}mm" height="{self.height}mm" viewBox="0 0 {self.width} {self.height}"><rect width="100%" height="100%" fill="white"/>'+''.join(self.items)+'</svg>'
        path.with_suffix('.svg').write_text(svg)
        cairosvg.svg2pdf(bytestring=svg.encode(),write_to=str(path.with_suffix('.pdf')))
        # PNG is only a preview; the PDF/SVG preserve vector curves/lines and text.
        cairosvg.svg2png(bytestring=svg.encode(),write_to=str(path.with_suffix('.png')),output_width=2100,output_height=1485)
        doc=ezdxf.new('R2010');doc.units=4;ms=doc.modelspace()
        for layer in ['VISIBLE','HIDDEN','CENTER','DIM','BORDER']:doc.layers.new(layer,dxfattribs={'color':8 if layer=='HIDDEN' else 7})
        for layer,a in self.dxf_lines:ms.add_lwpolyline([(float(x),float(self.height-y)) for x,y in a],dxfattribs={'layer':layer})
        from ezdxf.enums import TextEntityAlignment
        for x,y,text,size,anchor in self.dxf_text:
            obj=ms.add_text(text,dxfattribs={'height':size,'layer':'DIM'})
            obj.set_placement((x,self.height-y),align={'start':TextEntityAlignment.LEFT,'middle':TextEntityAlignment.CENTER,'end':TextEntityAlignment.RIGHT}[anchor])
        doc.saveas(path.with_suffix('.dxf'))


def whiteprint(assembly,output,part_names=None,title=None,scale=None,annotation_spec=None):
    """Generate one four-view A3 sheet; all generated dimensions are nominal."""
    key=part_names[0] if part_names and len(part_names)==1 else 'default'
    specification=annotation_spec or assembly.metadata.get('drawings',{}).get(key,{})
    if part_names is None:part_names=specification.get('parts')
    title=title or specification.get('title')
    if part_names:parts=[p for p in assembly.parts if p.name in part_names]
    else:
        groups=assembly.metadata.get('drawing_groups')
        parts=[p for p in assembly.parts if groups is None or p.group in groups]
    if not parts:raise ValueError('No drawing parts match the selection')
    analytic=all(p.cad is not None for p in parts)
    shape=cq.Compound.makeCompound([pose_cad(p.cad,assembly.pose(p)) for p in parts]) if analytic else None
    from dataclasses import replace
    subset=replace(assembly,parts=parts);raw={}
    for pr in PROJECTIONS:
        raw[pr.name]=projected_edges(shape,pr) if analytic else mesh_projected_edges(subset,pr)
    all_bounds={}
    for name,lines in raw.items():
        if not lines:raise ValueError('Projection has no edges')
        points=np.concatenate([arr for hidden,arr in lines]);all_bounds[name]=(points.min(0),points.max(0))
    if scale is None:
        maxw=max((hi-lo)[0] for lo,hi in all_bounds.values());maxh=max((hi-lo)[1] for lo,hi in all_bounds.values())
        fit=min(147/maxw,100/maxh)
        scale=next((s for s in [2.,1.,.5,.25,.1,.05,.01] if s<=fit+1e-6),.01)
    sheet=Sheet();sheet.box(8,8,404,281,.45);sheet.line([(8,27),(412,27)],'BORDER',.3)
    sheet.text(14,19,'CYBR / MECHANISM LAB',4.6,True);sheet.text(406,18,'WHITEPRINT  |  GEOMETRY-DERIVED',3.1,anchor='end')
    sheet.text(406,24,'OpenCascade hidden-line removal' if analytic else 'MESH FEATURE PROJECTION - OCCLUSION NOT SUPPRESSED',2.6,anchor='end')
    mappings={}
    for pr in PROJECTIONS:
        lo,hi=all_bounds[pr.name];center=(lo+hi)/2;paper=np.asarray(pr.center)
        def mapping(a,center=center,paper=paper):return (np.asarray(a)-center)*np.array([scale,-scale])+paper
        mappings[pr.name]=mapping
        for hidden,arr in sorted(raw[pr.name],key=lambda x:not x[0]):sheet.line(mapping(arr),'HIDDEN' if hidden else 'VISIBLE',.13 if hidden else .24)
        sheet.text(pr.center[0],137 if pr.name in ['top','iso'] else 144,pr.label,2.7,True,anchor='middle')
    if specification.get('auto_dimensions',True):
        lo,hi=all_bounds['side'];p=mappings['side']([[lo[0],lo[1]],[hi[0],lo[1]]])
        sheet.dimension(p[0],p[1],7,label=f'{hi[0]-lo[0]:.2f} REF')
        lo,hi=all_bounds['end'];p=mappings['end']([[lo[0],0],[hi[0],0]])
        sheet.dimension(p[0],p[1],(hi[1]-lo[1])*scale/2+7,label=f'{hi[0]-lo[0]:.2f} REF')
    # Annotation anchors live in the recipe, never in the shared drawing engine.
    # Coordinates are model-space 2D projection coordinates; offsets are paper mm.
    for annotation in specification.get('annotations',[]):
        kind=annotation['kind']
        view_key=annotation.get('view','end')
        mapping=mappings[view_key]
        origin=np.asarray(specification.get('anchor_origins',{}).get(view_key,[0,0]))
        fn=lambda points,mapping=mapping,origin=origin:mapping(np.asarray(points)+origin)
        if kind=='dimension':
            points=fn(annotation['points'])
            offset=annotation.get('offset_paper_mm',7)+annotation.get('offset_model_mm',0)*scale
            sheet.dimension(points[0],points[1],offset,annotation.get('orientation','horizontal'),annotation['label'])
        elif kind=='line':
            sheet.line(fn(annotation['points']),annotation.get('layer','CENTER'),.13)
        elif kind=='circle':
            angles=np.linspace(0,math.tau,192)
            points=np.column_stack([np.cos(angles),np.sin(angles)])*annotation['radius']+np.array(annotation.get('center',[0,0]))
            sheet.line(fn(points),annotation.get('layer','CENTER'),.12)
        elif kind=='leader':
            start=fn(annotation['point'])
            sheet.leader(start,start+np.array(annotation['elbow_paper_mm']),start+np.array(annotation['end_paper_mm']),annotation['text'])
        elif kind=='note':
            sheet.text(*annotation['paper_xy'],annotation['text'],annotation.get('size',2.7),annotation.get('bold',False))
        else:
            raise ValueError(f'Unknown whiteprint annotation kind: {kind}')
    # Bottom notes and title block. No invented material grades or tolerances.
    sheet.line([(8,256),(412,256)],'BORDER',.3);sheet.line([(215,256),(215,289)],'BORDER',.3)
    sheet.text(14,263,'NOTES',2.7,True)
    notes=['1  Nominal geometry only. No unspecified tolerances are assigned.',
           '2  Internal details and custom interfaces are not manufacturing-qualified.',
           '3  Do not use this study as a released fabrication or safety drawing.',
           '4  Curved edges: 0.025 mm sampling deflection before sheet scaling.' if analytic else '4  Mesh fallback: feature/silhouette lines; hidden edges not removed.']
    for i,text in enumerate(notes):sheet.text(14,269+i*4.4,text,2.65)
    sheet.text(221,264,(title or assembly.name.replace('_',' ').upper())[:49],3.6,True)
    sheet.line([(215,270),(412,270)],'BORDER',.18)
    sheet.text(221,277,'UNITS: mm',2.6);sheet.text(274,277,f'SCALE: {scale:g}:1',2.6);sheet.text(338,277,'REV: A  |  SHEET 1/1',2.6)
    sheet.text(221,285,'STATUS: CONCEPT / NOT RELEASED FOR FABRICATION',2.6,True)
    output=Path(output);sheet.save(output)
    report=dict(model=assembly.name,parts=[p.name for p in parts],paper='A3 landscape 420 x 297 mm',
                units='mm',scale=scale,method='OCP analytic hidden-line removal' if analytic else 'mesh feature-line projection without hidden removal',
                curve_deflection_mm=.025 if analytic else None,views=[pr.name for pr in PROJECTIONS],
                files=[output.with_suffix('.'+ext).name for ext in ['svg','pdf','dxf','png']],
                annotations=specification,
                note='PDF/SVG are vector; DXF uses sampled polylines. No manufacturing tolerances or approval implied.')
    output.with_suffix('.json').write_text(json.dumps(report,indent=2)+'\n');return report
