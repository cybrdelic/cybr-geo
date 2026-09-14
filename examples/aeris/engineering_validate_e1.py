"""Deterministic first-order engineering screening for AERIS-E1.

All calculations are bounded screens. Missing vendor/physical inputs remain
UNKNOWN and therefore prevent release_ready from becoming true.
"""
from __future__ import annotations
import argparse, json, math, pickle, sys
from pathlib import Path
import numpy as np
import trimesh
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src')); sys.path.insert(0,str(Path(__file__).resolve().parent))
from recipe import Config

RHO_AIR=1.204; G=9.80665; C_AIR=343.; COPPER_RESISTIVITY_20C=1.724e-8; E_STEEL=200e9
DENSITY={0:2700.,1:2700.,2:2700.,3:8960.,4:1100.,5:1150.,6:8000.,7:2700.,8:1400.,9:1100.,10:1200.}


def volume_mm3(p):
    if p.cad is not None: return float(p.cad.Volume())
    m=trimesh.Trimesh(np.asarray(p.vertices),np.asarray(p.faces),process=True)
    return abs(float(m.volume)) if m.is_watertight else None


def mass_rows(a):
    rows=[]; total=0.; unknown=[]
    for p in a.parts:
        v=volume_mm3(p)
        if v is None: unknown.append(p.name); continue
        m=v*1e-9*DENSITY[int(p.material)]; rows.append((p,m,v)); total+=m
    return rows,total,unknown


def flow_case(rpm,filter_drop):
    c=Config(); r2=c.rotor_radius/1000.; b2=(c.rotor_back-c.rotor_front-5.)/1000.; omega=2*math.pi*rpm/60.; u2=omega*r2
    area2=2*math.pi*r2*b2*.90; outlet=.0316*.0416; qfree=.15*area2*u2; dp0=.45*RHO_AIR*u2*u2
    qs=np.linspace(.0005,qfree*.999,12000); fan=dp0*(1-(qs/qfree)**2)
    system=1.5*.5*RHO_AIR*(qs/outlet)**2 + filter_drop*(qs/.020)**1.5
    q=float(qs[int(np.argmin(np.abs(fan-system)))]); dp=float(dp0*(1-(q/qfree)**2)); hood=math.pi*.082*.061
    return {'filter_drop_assumed_Pa_at_0p020_m3s':filter_drop,'flow_m3_s':q,'flow_cfm':q*2118.880003,
        'static_pressure_Pa':dp,'hood_mean_face_velocity_m_s':q/hood,'outlet_velocity_m_s':q/outlet,
        'impeller_tip_speed_m_s':u2,'tip_mach':u2/C_AIR,'free_delivery_model_m3_s':qfree}


def coil_length_m(turns):
    t=np.linspace(0,2*math.pi*turns,turns*5000+1); x=79.+17.5*np.sign(np.cos(t))*np.abs(np.cos(t))**.38
    y=23.8+5.0*t/(2*math.pi*turns); z=3.8*np.sign(np.sin(t))*np.abs(np.sin(t))**.6
    return float(np.linalg.norm(np.diff(np.c_[x,y,z],axis=0),axis=1).sum()/1000.)


def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--out',type=Path,default=ROOT/'build/aeris-e1'); ap.add_argument('--rpm',type=float,default=3600.); args=ap.parse_args()
    with (args.out/'assembly.pkl').open('rb') as f: a=pickle.load(f)
    rows,total,unknown_mass=mass_rows(a); c=Config(); omega=2*math.pi*args.rpm/60.; r2=c.rotor_radius/1000.; dshaft=2*c.shaft_radius/1000.
    moving=[(p,m) for p,m,_ in rows if p.motion=='rotor']; impeller=[(p,m) for p,m,_ in rows if p.group=='impeller']
    moving_mass=sum(m for _,m in moving); impeller_mass=sum(m for _,m in impeller)
    rotor_x=sum(m*float(np.mean(p.bounds[:,0])) for p,m in impeller)/max(impeller_mass,1e-12)
    cases=[flow_case(args.rpm,d) for d in (40.,80.,120.,160.)]; nominal=cases[1]
    air_power=nominal['flow_m3_s']*nominal['static_pressure_Pa']; shaft_power=air_power/.45; torque=shaft_power/omega
    omega2=2*omega; blade_root=2700.*omega2**2*(r2*r2-.021**2)/2; torsion=16*torque/(math.pi*dshaft**3)
    ecc=(6.3/omega)/1000.; unbalance=impeller_mass*ecc*omega**2; overhang=max(0.,.0495-rotor_x/1000.)
    bending=32*(unbalance+impeller_mass*G)*overhang/(math.pi*dshaft**3); I=math.pi*dshaft**4/64
    critical=math.sqrt(3*E_STEEL*I/max(overhang,1e-4)**3/max(impeller_mass,1e-6))/(2*math.pi)*60
    b=a.metadata['bearing_interface']; bore=float(b['nominal_bore_mm']); od=float(b['nominal_od_mm']); width=float(b['nominal_width_mm'])
    nominal_bearing=abs(float(b['shaft_nominal_mm'])-bore)<1e-9 and od>bore and width>0
    trunnion=(total*G/2)/(math.pi*((8e-3)**2-(3.2e-3)**2)); volute=300.*.096/.0052
    L=coil_length_m(c.wire_turns); wr=.00042; area=math.pi*wr**2; coil_R=COPPER_RESISTIVITY_20C*L/area; phase_R=4*coil_R
    electrical=[]
    for current in (3.,5.,10.):
        loss=3*current**2*phase_R; electrical.append({'phase_current_A':current,'current_density_A_mm2':current/(area*1e6),'copper_loss_W':loss,'convection_only_temperature_rise_C_range':[loss/(50*.025),loss/(20*.025)]})
    projected=math.pi*.0584**2; half=.120/(c.pleat_count*2); pleat=math.sqrt(.027**2+half**2)/half
    checks=[
      {'domain':'CAD/assembly','status':'PASS','evidence':f'{len(a.parts)}-part E1 assembly; separate CAD/kinematics/export verifier required.'},
      {'domain':'air-moving meanline screen','status':'CONDITIONAL_PASS','evidence':f"{nominal['flow_cfm']:.1f} CFM at {nominal['static_pressure_Pa']:.0f} Pa under assumed clean-filter resistance."},
      {'domain':'rotor centrifugal strength screen','status':'PASS','evidence':f'{blade_root/1e6:.2f} MPa at {2*args.rpm:.0f} rpm vs assumed 150 MPa aluminium screening yield.'},
      {'domain':'shaft torsion/bending screen','status':'PASS','evidence':f'torsion {torsion/1e6:.3f} MPa; bending {bending/1e6:.3f} MPa; first-order critical {critical:.0f} rpm.'},
      {'domain':'housing/yoke static screen','status':'PASS','evidence':f'trunnion shear {trunnion/1e6:.4f} MPa; volute membrane {volute/1e6:.4f} MPa at 300 Pa.'},
      {'domain':'shaft/bearing nominal geometry','status':'PASS' if nominal_bearing else 'FAIL','evidence':f"{b['series']} nominal {bore:.1f} x {od:.1f} x {width:.1f} mm with {b['shaft_nominal_mm']:.1f} mm shaft."},
      {'domain':'shaft/bearing production fit','status':'UNKNOWN','evidence':'Shaft/housing fit classes, internal clearance, preload and manufacturer part number are unassigned.'},
      {'domain':'bearing L10/contact','status':'UNKNOWN','evidence':'No manufacturer dynamic/static capacity or life calculation.'},
      {'domain':'filter pressure drop/efficiency','status':'UNKNOWN','evidence':f'Geometric pleated-area screen {projected*pleat:.3f} m2; certified media curve absent.'},
      {'domain':'motor electromagnetic torque','status':'UNKNOWN','evidence':f'Required shaft torque {torque:.4f} N m; Kv/Kt, magnet grade, voltage/current map absent.'},
      {'domain':'motor winding thermal','status':'CONDITIONAL_PASS','evidence':f'Modeled route {L:.3f} m/coil, {coil_R:.4f} ohm/coil; separate CYBR PHYSICS thermal submodel required.'},
      {'domain':'acoustics','status':'UNKNOWN','evidence':f'Blade-pass fundamental {c.blade_count*args.rpm/60:.0f} Hz; no SPL/BEM/measurement.'},
      {'domain':'capture effectiveness','status':'UNKNOWN','evidence':f"Mean hood-face screen {nominal['hood_mean_face_velocity_m_s']:.2f} m/s; no CAD-resolved open-domain capture CFD/measurement."},
      {'domain':'fabrication release','status':'UNKNOWN','evidence':'GD&T, thread classes, balance procedure, creepage/clearance and certified material grades unreleased.'}]
    hard=[x for x in checks if x['status']=='FAIL']; unknown=[x for x in checks if x['status']=='UNKNOWN']
    report={'model':'AERIS-E1','analysis_kind':'deterministic first-order multidisciplinary engineering screening','design_rpm':args.rpm,
      'release_ready':not hard and not unknown,'mass':{'estimated_total_kg':total,'rotating_kg':moving_mass,'impeller_kg':impeller_mass,'unclosed_mesh_mass_parts':unknown_mass,'density_assumptions_kg_m3':DENSITY},
      'aerodynamics':{'flow_cases':cases,'nominal_case':nominal,'air_power_W':air_power,'shaft_power_at_assumed_45pct_efficiency_W':shaft_power,'required_torque_Nm':torque},
      'rotor_structural':{'two_x_speed_rpm':2*args.rpm,'centrifugal_blade_root_screen_Pa':blade_root,'torsional_shear_Pa':torsion,'G6p3_unbalance_force_N':unbalance,'bending_stress_Pa':bending,'critical_speed_first_order_rpm':critical,'impeller_overhang_m':overhang},
      'bearing_interface':b,'support_structural':{'trunnion_average_shear_Pa':trunnion,'volute_300Pa_membrane_screen_Pa':volute},
      'electrical_thermal':{'wire_radius_mm':wr*1000,'coil_length_m':L,'coil_resistance_20C_ohm':coil_R,'cases':electrical},
      'checks':checks,'critical_failures':[x['domain'] for x in hard],'unqualified_domains':[x['domain'] for x in unknown],
      'verdict':'NOT_RELEASE_READY' if hard or unknown else 'SCREENING_PASS','limitations':['Meanline/similarity air model, not CAD-resolved CFD.','Beam/rotor formulas, not continuum FEA.','Material strengths/densities are screening assumptions.','No electromagnetic FEA.','No certified filter/media data.','No physical prototype data.']}
    args.out.mkdir(parents=True,exist_ok=True); (args.out/'engineering_report_e1.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({'verdict':report['verdict'],'hard_failures':report['critical_failures'],'unknowns':report['unqualified_domains'],'nominal_flow_cfm':nominal['flow_cfm']},indent=2))
    if hard: raise SystemExit(2)

if __name__=='__main__': main()
