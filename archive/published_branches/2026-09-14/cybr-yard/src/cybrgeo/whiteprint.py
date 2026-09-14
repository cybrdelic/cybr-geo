"""CAD-derived black-on-white drawings, not raster edge filters.

OpenCascade performs hidden-line removal. SVG/PDF are vector documents; DXF
contains segregated visible and hidden curve polylines in millimetres.
No arbitrary precision tolerances, fits or load ratings are invented.
"""
from __future__ import annotations
from dataclasses import dataclass
from pathlib import Path
from html import escape
import json, math
import numpy as np

@dataclass
class View:
    name: str
    direction: tuple[float,float,float]
    visible: list[np.ndarray]
    hidden: list[np.ndarray]
    bounds: np.ndarray

def project(shape, direction=(1,0,0), chord=.06) -> View:
    import cadquery as cq
    from OCP.HLRBRep import HLRBRep_Algo, HLRBRep_HLRToShape
    from OCP.HLRAlgo import HLRAlgo_Projector
    from OCP.gp import gp_Ax2,gp_Dir,gp_Pnt
    from OCP.BRepLib import BRepLib
    alg=HLRBRep_Algo();alg.Add(shape.wrapped)
    axis=gp_Ax2(gp_Pnt(),gp_Dir(*direction))
    if abs(direction[0])>.999 and direction[1:]==(0,0):axis.SetXDirection(gp_Dir(0,-1 if direction[0]<0 else 1,0))
    elif direction==(0,1,0):axis.SetXDirection(gp_Dir(-1,0,0))
    elif direction==(0,0,1):axis.SetXDirection(gp_Dir(1,0,0))
    alg.Projector(HLRAlgo_Projector(axis))
    alg.Update();alg.Hide();out=HLRBRep_HLRToShape(alg)
    def curves(kinds):
        paths=[]
        for shape in kinds:
            if shape.IsNull():continue
            BRepLib.BuildCurves3d_s(shape,1e-7)
            for edge in cq.Shape.cast(shape).Edges():
                if edge.Length()<1e-8:continue
                pts,_=edge.sample(chord)
                arr=np.array([p.toTuple()[:2] for p in pts])
                if len(arr)>1:paths.append(arr)
        return paths
    visible=curves([out.VCompound(),out.Rg1LineVCompound(),out.OutLineVCompound()])
    hidden=curves([out.HCompound(),out.OutLineHCompound()])
    if not visible and not hidden:raise ValueError(f'OpenCascade returned no projected edges for direction {direction}; choose a different view basis')
    allpts=np.concatenate(visible+hidden)
    return View('',direction,visible,hidden,np.array([allpts.min(0),allpts.max(0)]))

def write_whiteprint(shape, destination, title='PART DRAWING', number='CYBR-0001',
                     revision='A', notes=None, chord=.06, scale=None):
    """Make A3 sheet in explicit millimetres, four named projections and a DXF.

    Curves are sampled from projected CAD within the stated chord tolerance.
    Sheet scale is selected from conventional ratios unless specified by caller.
    Dimensions identify actual model bounding extents, not tolerance-qualified fits.
    """
    import cairosvg,ezdxf
    dst=Path(destination);dst.parent.mkdir(parents=True,exist_ok=True)
    configs=[('FRONT / X AXIS',(-1,0,0)),('SIDE / X-Z',(0,1,0)),
             ('TOP / X-Y',(0,0,1)),('AXONOMETRIC',(1.07,-1,.82))]
    views=[]
    for name,direction in configs:
        v=project(shape,direction,chord);v.name=name;views.append(v)
    # 420 x297 A3, 2x2 of 192 x98 model spaces, footer40.
    fit=min(min(160/max(np.ptp(v.bounds,axis=0)[0],1),80/max(np.ptp(v.bounds,axis=0)[1],1)) for v in views)
    ratios=[.05,.1,.2,.25,.5,1.,2.,5.,10.]
    if scale is None:scale=max([r for r in ratios if r<=fit] or [fit])
    if scale<=0:raise ValueError('Scale must be positive')
    if scale>fit:raise ValueError(f'Requested scale does not fit A3; maximum {fit:.3g}')
    scl=f'{scale:g}:1' if scale>=1 else f'1:{1/scale:g}'
    svg=['<svg xmlns="http://www.w3.org/2000/svg" width="420mm" height="297mm" viewBox="0 0 420 297">',
         '<rect width="420" height="297" fill="white"/>',
         '<style>text{font-family:DejaVu Sans,Arial,sans-serif;fill:#151a20} .edge{fill:none;stroke:#1c232b;stroke-width:.21;stroke-linecap:round;stroke-linejoin:round}.hidden{fill:none;stroke:#939aa0;stroke-width:.12;stroke-dasharray:1.5,1}.rule{stroke:#55616b;stroke-width:.18;fill:none}.dim{stroke:#54616e;stroke-width:.13;fill:none}</style>',
         '<rect x="8" y="8" width="404" height="281" class="rule"/>',
         '<path d="M8 32H412M8 241H412M210 32V241M8 136.5H412" class="rule"/>',
         '<text x="15" y="18" font-size="3.1" letter-spacing="1.1">CYBR GEO / CAD WHITEPRINT</text>',
         f'<text x="15" y="27" font-size="6" font-weight="bold">{escape(title[:63])}</text>',
         f'<text x="404" y="18" font-size="3" text-anchor="end">{escape(number)} / REV {escape(revision)}</text>',
         f'<text x="404" y="27" font-size="3" text-anchor="end">A3 · mm · SCALE {scl}</text>']
    doc=ezdxf.new('R2018');doc.units=4
    for name,color in [('VISIBLE',7),('HIDDEN',8),('ANNOTATION',7)]:
        doc.layers.new(name,dxfattribs={'color':color})
    msp=doc.modelspace()
    for i,v in enumerate(views):
        col,row=i%2,i//2;x0=8+202*col;y0=32+104.5*row
        origin=np.array([x0+101,y0+55]);center=v.bounds.mean(0)
        def points(arr):return (arr-center)*np.array([scale,-scale])+origin
        for kind,paths in [('hidden',v.hidden),('edge',v.visible)]:
            for path in paths:
                a=points(path);d='M'+' L'.join(f'{x:.4f},{y:.4f}' for x,y in a)
                svg.append(f'<path class="{kind}" d="{d}"/>')
                # Sheet coordinates in mm; flip back Y for native DXF.
                msp.add_lwpolyline([(float(x),297-float(y)) for x,y in a],dxfattribs={'layer':'VISIBLE' if kind=='edge' else 'HIDDEN'})
        svg.append(f'<text x="{x0+7}" y="{y0+9}" font-size="3.3" font-weight="bold">{escape(v.name)}</text>')
        spans=np.ptp(v.bounds,axis=0);lo=points(v.bounds[0]);hi=points(v.bounds[1]);yy=y0+94
        if i<3:
            # Projected horizontal extents, clearly a reference envelope dimension.
            svg.extend([f'<path class="dim" d="M{lo[0]:.3f} {yy-4}V{yy+2}M{hi[0]:.3f} {yy-4}V{yy+2}M{lo[0]:.3f} {yy}H{hi[0]:.3f}"/>',
                        f'<path class="dim" d="M{lo[0]-1:.3f} {yy+1}l2 -2M{hi[0]-1:.3f} {yy+1}l2 -2"/>',
                        f'<text x="{origin[0]:.3f}" y="{yy-1.8}" text-anchor="middle" font-size="3">{spans[0]:.2f} REF</text>'])
        msp.add_text(v.name,dxfattribs={'height':3.,'layer':'ANNOTATION'}).set_placement((x0+7,297-y0-9))
    default=['Geometry-derived projections; visible and hidden lines computed by OpenCascade.',
             'REF dimensions describe model extents only. No fits, tolerances or strength are certified.',
             f'Curve sampling tolerance: {chord:g} mm in model coordinates. Print at 100% for stated scale.']
    notes=default+(notes or [])
    for j,note in enumerate(notes[:7]):
        svg.append(f'<text x="15" y="{249+j*4.5}" font-size="2.65">{escape(note[:157])}</text>')
    svg.append('<text x="405" y="285" font-size="2.8" text-anchor="end">DESIGN STUDY / NOT RELEASED FOR MANUFACTURE</text>')
    svg.append('</svg>');text='\n'.join(svg);dst.with_suffix('.svg').write_text(text)
    cairosvg.svg2pdf(bytestring=text.encode(),write_to=str(dst.with_suffix('.pdf')))
    cairosvg.svg2png(bytestring=text.encode(),write_to=str(dst.with_suffix('.png')),output_width=2100,output_height=1485)
    doc.saveas(str(dst.with_suffix('.dxf')))
    metadata={'paper':'A3','units':'mm','scale':scale,'projection':'independently labeled views',
              'source':'analytic B-rep hidden-line removal','curve_sampling_mm':chord,
              'title':title,'number':number,'revision':revision,'view_directions':[v.direction for v in views],
              'limitations':'REF envelope dimensions; no production tolerances assigned'}
    dst.with_suffix('.json').write_text(json.dumps(metadata,indent=2))
    return metadata
