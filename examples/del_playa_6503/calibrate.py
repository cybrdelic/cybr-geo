#!/usr/bin/env python3
"""Sparse manual alignment of authored model landmarks, not a photogrammetry pipeline.

No dense correspondence, feature detection, bundle adjustment, triangulation,
scan mesh, photo texture or learned image generation is performed. Landmarks
are approximate human-readable observations; the known plan is mapped OSM.
"""
from pathlib import Path
import argparse, json, math
import numpy as np
from scipy.optimize import least_squares

ROOT=Path(__file__).resolve().parent
# [name, authored point in metres, manually observed pixel, annotation sigma in px]
OBSERVATIONS={
 'ocean':{'size':[960,720],'initial':[10,-24,8.2,-.35,-.19,49,0], 'points':[
  ['front_left_roof',[-4.6,.5,5.475],[301,327],7],
  ['front_right_roof',[5.351,0,5.475],[651,315],5],
  ['front_left_floor',[-4.6,.5,2.62],[303,409],10],
  ['front_right_floor',[5.351,0,2.62],[647,419],8],
  ['front_right_grade',[5.351,0,0],[644,510],7],
  ['bay_left_grade',[-5.90,-.624,0],[221,491],8],
  ['bay_right_grade',[-3.00,-.624,0],[325,501],6],
 ]},
 'east':{'size':[576,432],'initial':[31,24,12.5,-2.84,-.32,54,0], 'points':[
  ['front_east_roof',[5.351,0,5.475],[147,140],12],
  ['rear_east_roof',[-4.18,31.9,5.475],[487,105],12],
  ['rear_west_roof',[-15.69,32.78,5.475],[456,92],14],
  ['front_west_roof',[-4.6,.5,5.475],[131,121],14],
  ['front_east_grade',[5.351,0,0],[144,183],16],
  ['rear_east_grade',[-4.18,31.9,0],[484,151],16],
 ]}
}

def frame(p):
 origin=np.array(p[:3]);yaw,pitch=float(p[3]),float(p[4]);f=np.array([math.cos(pitch)*math.cos(yaw),math.cos(pitch)*math.sin(yaw),math.sin(pitch)])
 right=np.cross(f,[0,0,1]);right/=np.linalg.norm(right);top=np.cross(right,f)
 roll=float(p[6]);r=right*math.cos(roll)+top*math.sin(roll);t=top*math.cos(roll)-right*math.sin(roll)
 return origin,f,r,t

def project(points,p,size):
 origin,f,r,t=frame(p);q=np.asarray(points)-origin;depth=q@f;focal=.5*size[1]/math.tan(math.radians(p[5])/2)
 return np.c_[size[0]/2+focal*(q@r)/depth,size[1]/2-focal*(q@t)/depth]

def camera(p):
 o,f,r,t=frame(p)
 return {'origin':o.tolist(),'target':(o+f*20).tolist(),'up':t.tolist(),'fov':float(p[5])}

def fit_all():
 result={'method':'Manual sparse pinhole camera fit to authored metric landmarks; no reconstructed photographic geometry','limitations':['No lens metadata, calibrated camera, survey, or capture dates. East sea/sky horizon at pixel (55,53), sigma 5 px, constrains orientation independently of estimated building heights.','Pixel labels have 5–16 px uncertainty; several east-view roof/grade labels are partly occluded.','Vertical heights and upper facade corner (-4.6, 0.5 m) are manual reference estimates. A weak ocean camera-height prior (7 m, sigma 2 m) preserves visible roof foreshortening. Fit does not establish dimensional accuracy.','Both images can be accessed from one listing. These are two views, not two independent surveys.','Ocean fit reaches its 6 m camera-height bound; east fit reaches the 20-degree field-of-view bound; focal length, distance and pose are ambiguous. Camera is a virtual alignment solution, not recovered survey metadata.'],'views':{}}
 for name,spec in OBSERVATIONS.items():
  xyz=np.array([v[1] for v in spec['points']]);px=np.array([v[2] for v in spec['points']]);sigma=np.array([v[3] for v in spec['points']])
  initial=np.array(spec['initial'],float)
  # Start by looking towards the centroid, irrespective of initial angle annotations.
  d=xyz.mean(axis=0)-initial[:3];initial[3]=math.atan2(d[1],d[0]);initial[4]=math.atan2(d[2],math.hypot(d[0],d[1]))
  def objective(p):
   residual=(project(xyz,p,spec['size'])-px)/sigma[:,None]
   # Weak lens/roll priors address camera ambiguity without disguising residuals.
   
   if name=="east":
    _,forward,right,top=frame(p);focal=.5*spec["size"][1]/math.tan(math.radians(p[5])/2)
    horizon=spec["size"][1]/2+(forward[2]*focal+(55-spec["size"][0]/2)*right[2])/top[2]
    residual=np.r_[residual.ravel(),(horizon-53)/5]
   return np.r_[residual.ravel(),(p[5]-initial[5])/80,p[6]/.05, *(([(p[2]-7)/2]) if name=="ocean" else [])]
  bounds=([-80,-100,6 if name=='ocean' else 8,-2*math.pi,-1.2,20,-.15],[120,110,50,2*math.pi,.2,100,.15])
  fit=least_squares(objective,initial,bounds=bounds,loss='soft_l1',max_nfev=2500)
  predicted=project(xyz,fit.x,spec['size']);errors=np.linalg.norm(predicted-px,axis=1)
  result['views'][name]={'reference_id':name,'image_size':spec['size'],'camera':camera(fit.x),'solver_success':bool(fit.success),'horizon_constraint':({'observed_px':[55,53],'sigma_px':5,'meaning':'Visible sea/sky horizon in left of east reference; orientation constraint only.'} if name=='east' else None),'active_parameter_bounds':[n for n,a in zip(['x','y','height','yaw','pitch','vertical_fov','roll'],fit.active_mask) if a],'reprojection_rmse_px':float(np.sqrt(np.mean(errors**2))),'median_error_px':float(np.median(errors)),'max_error_px':float(errors.max()),'landmarks':[{'name':v[0],'model_m':v[1],'observed_px':v[2],'sigma_px':v[3],'projected_px':q.tolist(),'error_px':float(e)} for v,q,e in zip(spec['points'],predicted,errors)]}
 return result

def svg(result,path):
 elements=['<svg xmlns="http://www.w3.org/2000/svg" width="1120" height="800" viewBox="0 0 1120 800">','<rect width="1120" height="800" fill="#11191d"/>','<text x="34" y="38" fill="#f4f0df" font-size="23" font-family="sans-serif">6503 DEL PLAYA · MANUAL REFERENCE ALIGNMENT</text>']
 for col,(name,spec) in enumerate(result['views'].items()):
  ox=34+col*550;oy=105;scale=510/spec['image_size'][0]
  elements.append(f'<text x="{ox}" y="78" fill="#e7dfcc" font-size="17" font-family="sans-serif">{name.upper()} · RMSE {spec["reprojection_rmse_px"]:.1f} px</text>')
  for i,p in enumerate(spec['landmarks']):
   x,y=np.array(p['observed_px'])*scale+[ox,oy];a,b=np.array(p['projected_px'])*scale+[ox,oy]
   elements.extend([f'<line x1="{x}" y1="{y}" x2="{a}" y2="{b}" stroke="#e6b26e"/>',f'<circle cx="{x}" cy="{y}" r="4" fill="#72d1c3"/>',f'<circle cx="{a}" cy="{b}" r="6" fill="none" stroke="#e6b26e"/>',f'<text x="{x+8}" y="{y-5}" fill="#eceddf" font-size="12" font-family="sans-serif">{i+1}</text>'])
  for i,p in enumerate(spec['landmarks']):elements.append(f'<text x="{ox}" y="{540+i*22}" fill="#a9b7ba" font-size="13" font-family="sans-serif">{i+1}. {p["name"]}: {p["error_px"]:.1f} px (sigma {p["sigma_px"]})</text>')
 elements.extend(['<text x="34" y="750" fill="#72d1c3" font-size="14" font-family="sans-serif">Filled dots: manual image labels · Rings: pinhole projection · Links: residuals</text>','<text x="34" y="776" fill="#b9b3a5" font-size="13" font-family="sans-serif">Mapped plan + estimated elevations. Alignment is not a survey or proof of hidden geometry. Source photographs are not redistributed.</text>','</svg>']);path.write_text('\n'.join(elements))

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--out',type=Path,default=ROOT/'alignment.json');args=p.parse_args();r=fit_all();args.out.write_text(json.dumps(r,indent=2)+'\n');svg(r,args.out.with_suffix('.svg'))
 for k,v in r['views'].items():print(k,'RMSE',round(v['reprojection_rmse_px'],2),'camera',v['camera'])
