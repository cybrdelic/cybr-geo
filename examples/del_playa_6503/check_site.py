#!/usr/bin/env python3
"""Compare independent plan constraints without forcing disagreement to disappear."""
import json
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parent
def cross(a,b):return a[0]*b[1]-a[1]*b[0]
def area(p):return .5*abs(sum(cross(a,b) for a,b in zip(p,[*p[1:],p[0]])))
def clip_polygon(poly,clip):
    if sum(cross(a,b) for a,b in zip(clip,np.roll(clip,-1,axis=0)))<0:clip=clip[::-1]
    q=list(poly)
    for a,b in zip(clip,np.roll(clip,-1,axis=0)):
        if not q:return []
        out=[]
        for u,v in zip(q,[*q[1:],q[0]]):
            cu=cross(b-a,u-a);cv=cross(b-a,v-a)
            if cu>=0:out.append(u)
            if (cu>=0)!=(cv>=0):out.append(u+(v-u)*cu/(cu-cv))
        q=out
    return q

def main():
    c=json.loads((ROOT/'site_constraints.json').read_text());p=np.array(c['footprint']['local_m']);q=np.array(c['parcel']['local_m'][:-1]);overlap=clip_polygon(p,q)
    r={'footprint_area_m2':area(list(p)),'assessment_parcel_area_m2':area(list(q)),'overlap_area_m2':area(overlap),'footprint_fraction_inside_assessment_parcel':area(overlap)/area(list(p)),'decision':'Retain both independent source outlines. About 12% of the mapped envelope lies outside the assessment polygon; do not treat either as a survey or silently rescale the building.','elevation':'No vertical constraint was found that establishes building height, grade, bluff crest or tide level.'}
    (ROOT/'site_check.json').write_text(json.dumps(r,indent=2)+'\n')
    def pts(poly):return ' '.join(f'{390+x*6:.2f},{490-y*6:.2f}' for x,y in poly)
    x,y=c['address_point']['local_m'];path=np.array(c['context']['paths'][0]['local_m'])
    svg=['<svg xmlns="http://www.w3.org/2000/svg" width="900" height="780" viewBox="0 0 900 780">','<rect width="900" height="780" fill="#11191d"/>','<g font-family="sans-serif" fill="#e9e7de">','<text x="32" y="38" font-size="24">6503 DEL PLAYA · INDEPENDENT PLAN CONSTRAINTS</text>',f'<polygon points="{pts(q)}" fill="#efb36d" fill-opacity=".12" stroke="#efb36d" stroke-width="2"/>',f'<polygon points="{pts(p)}" fill="#75cbb7" fill-opacity=".25" stroke="#75cbb7" stroke-width="2"/>',f'<polyline points="{pts(path)}" fill="none" stroke="#889bad" stroke-width="3"/>',f'<circle cx="{390+x*6:.2f}" cy="{490-y*6:.2f}" r="5" fill="#f6f2e7"/>','<text x="650" y="150" font-size="15">Local +Y (inland)</text>','<line x1="680" y1="230" x2="680" y2="165" stroke="#d8d8ce" stroke-width="2"/>','<line x1="80" y1="555" x2="200" y2="555" stroke="#d8d8ce" stroke-width="3"/>','<text x="80" y="582" font-size="15">20 m · approximate tangent scale</text>','<text x="32" y="625" font-size="16" fill="#75cbb7">Green: OSM way 42753197 v7 · mapped building envelope, 438.5 m²</text>','<text x="32" y="653" font-size="16" fill="#efb36d">Amber: County assessment parcel APN 075-223-019 · not a legal survey</text>','<text x="32" y="681" font-size="16">White point: County address locator · Gray line: OSM dirt path centerline</text>',f'<text x="32" y="715" font-size="16">Overlap: {r["footprint_fraction_inside_assessment_parcel"]:.1%}. Disagreement retained; elevations remain estimated.</text>','<text x="32" y="750" font-size="13">OpenStreetMap contributors, ODbL 1.0. County public GIS. Local XY is rotated to the ocean facade.</text>','</g></svg>']
    (ROOT/'site_check.svg').write_text('\n'.join(svg)+'\n');print(json.dumps(r,indent=2))

if __name__=='__main__':main()
