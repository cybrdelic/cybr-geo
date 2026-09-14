"""Engineering gate for native CYBR GEO REACH-2 v6.

Loads are derived from the analytic BReps stored on cybrgeo.Assembly.cad.
Rendering is not used as evidence of strength. The external support pair is
sized from the bearing constants declared by the native CYBR GEO recipe.
"""
from __future__ import annotations
import argparse, importlib.util, json, math
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
RECIPE=ROOT/'examples'/'reach2_cybrgeo_v6.py'
G=9.80665
PAYLOAD_KG=2.0
TOOL_EXTENSION_M=.050
ACCEL=4.0
MAX_SPEED_DEG_S=60.0
SHOCK_G=3.0
EFF=.75


def load_recipe():
    spec=importlib.util.spec_from_file_location('reach2_native_v6',RECIPE);m=importlib.util.module_from_spec(spec);assert spec.loader;spec.loader.exec_module(m);return m

def density(name):
    s=name.lower()
    if 'steel' in s:return 7.85e-6
    if 'bronze' in s:return 8.8e-6
    if 'copper' in s:return 8.96e-6
    if 'elastomer' in s or 'polymer' in s:return 1.2e-6
    return 2.70e-6

def mass(a,p,r):
    if p.name=='R6_10_CSG20_160_LW_envelope':return r.GEAR_MASS_KG
    if p.name=='R6_21_ECMA100W_motor_envelope':return r.MOTOR_MASS_KG
    return float(a.cad[p.name].Volume())*density(a.materials[p.material].name)

def radius_center(shape,pivot):
    c=np.asarray(shape.Center().toTuple(),float);return math.hypot(c[0]-pivot[0],c[2]-pivot[2])/1000

def radius_bounds(shape,pivot):
    b=shape.BoundingBox();return max(math.hypot(x-pivot[0],z-pivot[2]) for x in (b.xmin,b.xmax) for z in (b.zmin,b.zmax))/1000

def check(name,value,limit,relation='<=',units=''):
    ok=value<=limit if relation=='<=' else value>=limit
    margin=(limit/value if relation=='<=' and value>0 else value/limit if relation=='>=' and limit>0 else float('inf'))
    return dict(name=name,passed=bool(ok),value=float(value),limit=float(limit),relation=relation,units=units,factor_margin=float(margin))

def bolt_group_increment(moment_nm,coords,axis):
    vals=np.array([p[1] if axis=='x' else p[0] for p in coords],float);vals-=vals.mean();den=np.sum(vals**2)
    return abs(moment_nm*1000)*np.max(np.abs(vals))/den

def qualify():
    r=load_recipe();a=r.build();pivot=np.asarray(r.PIVOT,float)
    bad=[n for n,s in a.cad.items() if not s.isValid() or len(s.Faces())==0]
    moving=[p for p in a.parts if p.motion in ('orbit','elbow')]
    mm=sum(mass(a,p,r) for p in moving)
    grav=sum(mass(a,p,r)*G*radius_center(a.cad[p.name],pivot) for p in moving)
    inertia=sum(mass(a,p,r)*radius_center(a.cad[p.name],pivot)**2 for p in moving)
    rmax=max(radius_bounds(a.cad[p.name],pivot) for p in moving)
    rp=rmax+TOOL_EXTENSION_M
    grav_p=PAYLOAD_KG*G*rp;inertia_p=PAYLOAD_KG*rp**2
    static=grav+grav_p;It=inertia+inertia_p;accel=It*ACCEL
    continuous=static*1.5;repeated=(static+accel)*1.25;momentary=static*SHOCK_G*1.25
    radial=(mm+PAYLOAD_KG)*G*SHOCK_G*1.5
    bearing_moment=(sum(mass(a,p,r)*G*SHOCK_G*radius_center(a.cad[p.name],pivot) for p in moving)+PAYLOAD_KG*G*SHOCK_G*rp)*1.5

    out_rpm=MAX_SPEED_DEG_S/360*60;in_rpm=out_rpm*r.GEAR_RATIO
    mot_cont=continuous/(r.GEAR_RATIO*EFF);mot_rep=repeated/(r.GEAR_RATIO*EFF);mot_shock=momentary/(r.GEAR_RATIO*EFF)

    spacing=r.SUPPORT_BEARING_SPACING_MM/1000
    bearing_each=bearing_moment/spacing+radial/2
    bearing_dynamic=(bearing_moment/1.5)/spacing+(radial/1.5)/2
    l10=(r.SUPPORT_BEARING_DYNAMIC_N/max(bearing_dynamic,1e-9))**3*1e6

    Z=2*(12.0*24.0**2/6.0);pedestal_stress=bearing_moment*1000/Z
    M6_AREA=20.1;M8_AREA=36.6;PROOF10_9=830.0
    orbit_coords=list(r.ORBIT_PATTERN)
    orbit_inc=math.hypot(bolt_group_increment(momentary,orbit_coords,'x'),bolt_group_increment(momentary,orbit_coords,'y'))/M6_AREA
    lower_center=np.array([-22.,9.,-151.]);total_mass=sum(mass(a,p,r) for p in a.parts)
    lower_moment=sum(mass(a,p,r)*G*SHOCK_G*radius_center(a.cad[p.name],lower_center) for p in a.parts)*1.5
    lower_moment+=PAYLOAD_KG*G*SHOCK_G*(rp+abs(pivot[2]-lower_center[2])/1000)*1.5
    lower_inc=math.hypot(bolt_group_increment(lower_moment,list(r.LOWER_PATTERN),'x'),bolt_group_increment(lower_moment,list(r.LOWER_PATTERN),'y'))/M8_AREA
    orbit_clear=(6.8-6.03)/2-math.sqrt(2)*(.05+.05);lower_clear=(9.0-8.03)/2-math.sqrt(2)*(.05+.05)

    rb=[a.cad[p.name].BoundingBox() for p in a.parts if p.name.startswith('R6_')]
    dims=dict(x=max(b.xmax for b in rb)-min(b.xmin for b in rb),y=max(b.ymax for b in rb)-min(b.ymin for b in rb),z=max(b.zmax for b in rb)-min(b.zmin for b in rb))

    bname=r.SUPPORT_BEARING_MODEL
    checks=[
      check('valid analytic CAD parts',len(bad),0,'<=','parts'),
      check('gear continuous torque',continuous,r.GEAR_RATED_TORQUE_NM,'<=','Nm'),
      check('gear repeated torque',repeated,r.GEAR_REPEATED_PEAK_NM,'<=','Nm'),
      check('gear 3g momentary torque',momentary,r.GEAR_MOMENTARY_PEAK_NM,'<=','Nm'),
      check('gear average input speed',in_rpm,r.GEAR_MAX_AVG_INPUT_RPM,'<=','rpm'),
      check('motor continuous torque',mot_cont,r.MOTOR_RATED_TORQUE_NM,'<=','Nm'),
      check('motor repeated torque',mot_rep,r.MOTOR_MAX_TORQUE_NM,'<=','Nm'),
      check('motor 3g shock equivalent torque',mot_shock,r.MOTOR_MAX_TORQUE_NM,'<=','Nm'),
      check('motor rated-speed requirement',in_rpm,r.MOTOR_RATED_RPM,'<=','rpm'),
      check(f'{bname} pair static force per bearing',bearing_each,r.SUPPORT_BEARING_STATIC_N,'<=','N'),
      check(f'{bname} pair dynamic force per bearing',bearing_dynamic,r.SUPPORT_BEARING_DYNAMIC_N,'<=','N'),
      check(f'{bname} pair L10 life',l10,100_000_000,'>=','rev'),
      check('pedestal bending stress',pedestal_stress,276/2,'<=','MPa'),
      check('ORBIT M6 incremental tension stress',orbit_inc,PROOF10_9*.30,'<=','MPa'),
      check('lower M8 incremental tension stress',lower_inc,PROOF10_9*.30,'<=','MPa'),
      check('ORBIT pattern worst-case clearance',orbit_clear,.05,'>=','mm'),
      check('lower pattern worst-case clearance',lower_clear,.15,'>=','mm'),
      check('compact joint x silhouette',dims['x'],110,'<=','mm'),
    ]
    return dict(model=a.name,qualified=all(c['passed'] for c in checks),manufacturing_release=False,
      load_case=dict(payload_kg=PAYLOAD_KG,tool_extension_m=TOOL_EXTENSION_M,accel_rad_s2=ACCEL,max_speed_deg_s=MAX_SPEED_DEG_S,shock_g=SHOCK_G),
      cad_derived=dict(moving_mass_kg=mm,total_mass_kg=total_mass,continuous_design_torque_nm=continuous,repeated_design_torque_nm=repeated,momentary_design_torque_nm=momentary,bearing_design_moment_nm=bearing_moment,bearing_design_radial_n=radial,package_mm=dims),
      drivetrain=dict(input_rpm=in_rpm,motor_continuous_nm=mot_cont,motor_repeated_nm=mot_rep,motor_shock_nm=mot_shock,support_bearing=bname,bearing_pair_force_n=bearing_each,bearing_l10_rev=l10),checks=checks,
      release_blockers=['Overlay purchased CSG-20-160-2UH-LW and ECMA-C10401 STEP/drawings.','Finalize NSK 6905 fits/preload and tolerance stack for the purchased clearance/tolerance class.','Prototype proof-load, thermal, cable-flex and fatigue/cycle testing.'])

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path);ap.add_argument('--require-pass',action='store_true');args=ap.parse_args();r=qualify();text=json.dumps(r,indent=2);print(text)
    if args.out:args.out.parent.mkdir(parents=True,exist_ok=True);args.out.write_text(text+'\n')
    if args.require_pass and not r['qualified']:raise SystemExit('failed: '+', '.join(c['name'] for c in r['checks'] if not c['passed']))
if __name__=='__main__':main()
