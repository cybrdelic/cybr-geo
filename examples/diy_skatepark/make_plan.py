"""Generate a dimensioned nominal site diagram from the built model metadata."""
from __future__ import annotations
import argparse, csv, html, json, math
from pathlib import Path
import cairosvg


def create_plan(out: Path):
    meta=json.loads((out/'design.json').read_text())
    cfg=meta['config'];obstacles=meta['obstacles'];transitions=meta['transitions']
    S=.0485;OX=100;OY=170;L=cfg['slab_length'];W=cfg['slab_width']
    X=lambda x:OX+(x+L/2)*S
    Y=lambda y:OY+(W/2-y)*S
    e=[]
    def text(x,y,t,size=18,fill='#273733',weight='400',anchor='start',extra=''):
        e.append(f'<text x="{x:.2f}" y="{y:.2f}" font-size="{size}" fill="{fill}" font-weight="{weight}" text-anchor="{anchor}" {extra}>{html.escape(t)}</text>')
    def line(x1,y1,x2,y2,color='#6c7772',w=1,dash=''):
        e.append(f'<line x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" stroke="{color}" stroke-width="{w}"'+(f' stroke-dasharray="{dash}"' if dash else '')+'/>')
    def rect(x0,y0,x1,y1,fill,stroke='#6a624d',w=1):
        e.append(f'<rect x="{X(x0):.3f}" y="{Y(y1):.3f}" width="{(x1-x0)*S:.3f}" height="{(y1-y0)*S:.3f}" fill="{fill}" stroke="{stroke}" stroke-width="{w}"/>')
    e.append('<svg xmlns="http://www.w3.org/2000/svg" width="1600" height="1260" viewBox="0 0 1600 1260"><rect width="1600" height="1260" fill="#f5f4ee"/><g font-family="DejaVu Sans,Arial,sans-serif">')
    text(100,65,'CYBR YARD',40,weight='700')
    text(100,98,'DIY SKATEPARK / NOMINAL LAYOUT',16,fill='#617570',weight='600')
    text(1500,65,'DY–01',27,weight='700',anchor='end')
    text(1500,97,'REAL CAD + PROCEDURAL GEOMETRY',12,fill='#617570',anchor='end')
    line(100,118,1500,118,'#b8c1b9')
    # Slab and actual six-by-four pour pattern, with explicit metric dimensions.
    rect(-L/2,-W/2,L/2,W/2,'#e0e2d9','#86968f',2)
    for i in range(1,6):line(X(-L/2+i*L/6),Y(-W/2),X(-L/2+i*L/6),Y(W/2),'#c4cac1',1)
    for j in range(1,4):line(X(-L/2),Y(-W/2+j*W/4),X(L/2),Y(-W/2+j*W/4),'#c4cac1',1)
    line(X(-L/2),147,X(L/2),147,'#52746e',1.3)
    for x in (-L/2,L/2):line(X(x),140,X(x),163,'#52746e',1.3)
    text((X(-L/2)+X(L/2))/2,142,'22.00 m',16,anchor='middle',weight='700')
    line(75,Y(-W/2),75,Y(W/2),'#52746e',1.3)
    for y in (-W/2,W/2):line(68,Y(y),94,Y(y),'#52746e',1.3)
    text(60,(Y(-W/2)+Y(W/2))/2,'16.00 m',16,weight='700',anchor='middle',extra=f'transform="rotate(-90 60 {(Y(-W/2)+Y(W/2))/2})"')
    # Fence and amenities are placed using the same authored coordinates as CAD.
    line(X(-11000),Y(8155),X(11000),Y(8155),'#8b7960',6)
    line(X(11155),Y(-7200),X(11155),Y(8000),'#8b7960',6)
    rect(2050,6420,3950,6915,'#ae9776')
    for px in (6200,8600):rect(px-525,6575,px+525,7225,'#8eaa7b')
    text(X(2950),Y(7460),'BENCH + PLANTERS',12,fill='#586c61')
    rect(1500,7990,5100,8180,'#275d57','#275d57')
    # Mini ramp: decks, actual planar divisions and sampled curved-sheet joins.
    mini=obstacles[0];rect(mini['x0'],mini['y0'],mini['x1'],mini['y1'],'#d3b888','#8f7855',1.5)
    for t in transitions[:2]:
        toe,run,sign=t['toe_mm'],t['run_mm'],t['sign'];lip=toe+sign*run
        for x in (toe,lip):line(X(x),Y(mini['y0']),X(x),Y(mini['y1']),'#8f7855',1.2)
        line(X(t['coping_center_mm'][0]),Y(mini['y0']),X(t['coping_center_mm'][0]),Y(mini['y1']),'#285b58',3)
        for a in (t['theta_rad']/3,2*t['theta_rad']/3):
            x=toe+sign*t['radius_mm']*math.sin(a)
            line(X(x),Y(mini['y0']),X(x),Y(mini['y1']),'#b49b72',.8)
    for x in (-5500,-4300):line(X(x),Y(600),X(x),Y(5400),'#b49b72',.8)
    for y in (1800,3000,4200):line(X(mini['x0']),Y(y),X(mini['x1']),Y(y),'#b49b72',.8)
    east=transitions[1];cx=east['toe_mm']+east['run_mm']+500;edge=mini['y1']
    rect(cx-460,edge,cx+460,edge+1560,'#bcaa88')
    for i in range(1,6):line(X(cx-460),Y(edge+i*260),X(cx+460),Y(edge+i*260),'#8f7855',1)
    text(X(-4900),Y(3160),'01  MINI RAMP',21,weight='700',anchor='middle')
    text(X(-4900),Y(2560),'9.29 × 4.80 m overall',14,anchor='middle')
    text(X(-4900),Y(2080),'R 2.20 m  /  H 1.00 m above flat',13,anchor='middle')
    text(X(-2150),Y(7220),'STAIRS',11,fill='#617570')
    text(X(-2150),Y(6880),'1.04 m exit space to slab edge',10,fill='#617570')
    line(X(-1500),Y(6910),X(cx-480),Y(6760),'#617570',.8)
    # Street elements, all bounding envelopes derived from design metadata.
    for ob in obstacles[1:]:
        rect(ob['x0'],ob['y0'],ob['x1'],ob['y1'], '#8ca49b' if ob['kind']=='rail' else ('#c6cabb' if ob['kind']=='curb' else '#d0b385'))
    rect(950,-5425,3950,-5375,'#2f5551','#2f5551')
    for x in (1250,2450,3650):rect(x-55,-5750,x+55,-5050,'#435e59','#435e59')
    # Labels with leaders stay outside narrow elements.
    for n,name,x,y in [(2,'BANK',-7985,-4050),(3,'MANUAL PAD',-2450,-4460),(7,'QUARTER',8600,-4000)]:
        text(X(x),Y(y),f'{n:02}  {name}',14 if n!=3 else 12,weight='700',anchor='middle')
    text(X(2250),Y(-1670),'04  GRIND LEDGE',13,anchor='middle',weight='700')
    line(X(2250),Y(-1750),X(2250),Y(-1990),'#60776e',1)
    text(X(2450),Y(-6320),'05  FLAT BAR',13,anchor='middle',weight='700')
    line(X(2450),Y(-5920),X(2450),Y(-5750),'#60776e',1)
    text(X(6350),Y(4510),'06  SLAPPY CURB',13,anchor='middle',weight='700')
    line(X(6350),Y(4340),X(6350),Y(3965),'#60776e',1)
    # A diagrammatic connector, not a claimed skating simulation or certified fall zone.
    line(X(-9800),Y(-760),X(10000),Y(-760),'#7b9389',1.5,'8 8')
    text(X(400),Y(-620),'OPEN CONNECTING AISLE',13,fill='#58766a',anchor='middle',weight='600')
    text(X(400),Y(-1110),'Obstacle envelope separation only; rider trajectories and fall zones not simulated.',10,fill='#58766a',anchor='middle')
    # Right-hand schedule.
    sx=1220;text(sx,179,'ELEMENT SCHEDULE',18,weight='700')
    labels=[('01','Mini ramp','4.80 m wide / 3.60 m flat','1.00 m rise / R 2.20 m'),('02','Bank','3.00 m wide / 2.35 m slope run','0.65 m high / 0.85 m deck'),('03','Manual pad','2.40 × 1.20 m','0.20 m high'),('04','Grind ledge','3.20 × 0.62 m','0.36 m high / steel angles'),('05','Flat bar','3.00 m long / 50 mm section','0.34 m high / 3 mm wall'),('06','Slappy curb','3.20 × 0.33 m','0.12 m high'),('07','Quarter pipe','3.60 m wide / R 2.10 m','0.84 m high / 0.76 m deck')]
    for i,(n,title,l1,l2) in enumerate(labels):
        yy=222+i*87
        text(sx,yy,n,17,fill='#36786b',weight='700');text(sx+40,yy,title,17,weight='700')
        text(sx+40,yy+23,l1,12,fill='#5f6e67');text(sx+40,yy+42,l2,12,fill='#5f6e67')
    line(sx,837,1500,837,'#b8c1b9');text(sx,869,'1,404 components',18,weight='700')
    text(sx,893,'1,401 analytic CAD parts',13,fill='#5f6e67')
    text(sx,916,'157,034 rendered triangles',13,fill='#5f6e67')
    # Scale and qualification, kept visually separate from the design.
    y=990
    for j,(a,b) in enumerate(((0,1000),(1000,2000),(2000,5000))):
        e.append(f'<rect x="{100+a*S}" y="{y}" width="{(b-a)*S}" height="10" fill="'+('#325b53' if j%2==0 else '#d7ded3')+'" stroke="#325b53"/>')
    for a in (0,1000,2000,5000):text(100+a*S,y+31,f'{a/1000:g}'+(' m' if a==5000 else ''),12,anchor='middle')
    text(720,1016,'Open south edge / access from existing property',13,fill='#5f6e67')
    line(100,1055,1500,1055,'#b8c1b9')
    text(100,1090,'DESIGN STUDY — NOT CONSTRUCTION-QUALIFIED',17,weight='700')
    text(100,1120,'All sizes are nominal. Geometry checks do not establish structural capacity, foundation design, drainage, anchorage, traction, or safe fall clearances.',13,fill='#5f6e67')
    text(100,1145,'Mini-ramp height is measured above its raised flat bottom; the flat is 0.165 m above grade and the decks are 1.165 m above grade.',13,fill='#5f6e67')
    text(100,1170,'Plywood thicknesses, bending capability, support spacing and fasteners need a build-specific review before physical construction.',13,fill='#5f6e67')
    text(100,1220,'CYBR GEO  /  DY-01  /  MM MODEL → METRIC PLAN',11,fill='#617570',weight='600')
    text(1500,1220,'PROCEDURAL GEOMETRY · NO IMAGE SYNTHESIS',11,fill='#617570',anchor='end')
    e.append('</g></svg>');svg=''.join(e)
    (out/'CYBR_YARD_layout.svg').write_text(svg)
    cairosvg.svg2png(bytestring=svg.encode(),write_to=str(out/'CYBR_YARD_layout.png'),output_width=2400,output_height=1890)
    # Nominal developed skin lengths from the neutral radius, not a kerf/nesting plan.
    rows=[]
    for t in transitions:
        R=t['radius_mm'];standalone=t['prefix']=='street_quarter'
        a0=math.acos(1-45/R) if standalone else 0
        a1=math.asin((t['run_mm']-7)/R);count=2 if standalone else 3;bay=t['width_mm']/round(t['width_mm']/1200)
        for j in range(round(t['width_mm']/1200)):
            for k in range(count):
                angle=(a1-a0)/count-2*.38/R
                for layer,offset,thick in ((0,0,6),(1,6,9),(2,15,9)):
                    rows.append(dict(part=f"{t['prefix']}_curve_panel_{j:02}_{k:02}_ply{layer}",thickness_mm=thick,
                        developed_length_mm=round((R+offset+thick/2)*angle,2),width_mm=round(bay-.7,2),
                        basis='Nominal neutral-axis development; bend/kerf/tolerance qualification excluded'))
    with (out/'nominal_curved_skin_schedule.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    return len(rows)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('out',type=Path);a=p.parse_args();print('Developed curved skins:',create_plan(a.out))
