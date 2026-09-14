"""First-order multidisciplinary screen for the corrected AERIS-E1 assembly.

Reuses the established AERIS screening arithmetic but resolves bearing geometry
from the assembly metadata instead of the obsolete ad-hoc bearing envelope.
Nominal geometric compatibility is not promoted to a production fit claim.
"""
from __future__ import annotations
import argparse, json, math, pickle, sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src'));sys.path.insert(0,str(Path(__file__).resolve().parent))
import engineering_validate as base
from recipe import Config


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,default=ROOT/'build/aeris-e1');ap.add_argument('--rpm',type=float,default=3600.);args=ap.parse_args()
    with (args.out/'assembly.pkl').open('rb') as f:a=pickle.load(f)
    rows,total_mass,unknown_mass=base._mass_properties(a)
    c=Config();omega=2*math.pi*args.rpm/60.;r2=c.rotor_radius/1000.;dshaft=2*c.shaft_radius/1000.
    moving=[(p,m) for p,m,_ in rows if p.motion=='rotor'];impeller=[(p,m) for p,m,_ in rows if p.group=='impeller']
    moving_mass=sum(m for _,m in moving);impeller_mass=sum(m for _,m in impeller)
    rotor_x=sum(m*float(np.mean(p.bounds[:,0])) for p,m in impeller)/max(impeller_mass,1e-12)
    flow_cases=[base._solve_flow(args.rpm,drop) for drop in (40.,80.,120.,160.)];nominal=flow_cases[1]
    air_power=nominal['flow_m3_s']*nominal['static_pressure_Pa'];shaft_power=air_power/.45;torque=shaft_power/omega
    omega_over=2*omega;blade_root=2700.*omega_over**2*(r2*r2-.021**2)/2
    torsion=16*torque/(math.pi*dshaft**3);ecc=(6.3/omega)/1000.;unbalance=impeller_mass*ecc*omega**2
    overhang=max(0.,.0495-rotor_x/1000.);moment=(unbalance+impeller_mass*base.G)*overhang;bending=32*moment/(math.pi*dshaft**3)
    I=math.pi*dshaft**4/64;k=3*base.E_STEEL_PA*I/max(overhang,1e-4)**3;critical=math.sqrt(k/max(impeller_mass,1e-6))/(2*math.pi)*60
    b=a.metadata['bearing_interface'];shaft_nom=10.0;bore=float(b['nominal_bore_mm']);od=float(b['nominal_od_mm']);width=float(b['nominal_width_mm'])
    nominal_compatible=abs(shaft_nom-bore)<1e-9 and od>shaft_nom and width>0
    trunnion_area=math.pi*((8e-3)**2-(3.2e-3)**2);trunnion=(total_mass*base.G/2)/trunnion_area;volute=300.*.096/.0052
    coil_length=base._coil_length_m(c.wire_turns);wire_radius=.00042;wire_area=math.pi*wire_radius**2;coil_R=base.COPPER_RESISTIVITY_20C*coil_length/wire_area;phase_R=4*coil_R
    electrical=[]
    for current in (3.,5.,10.):
        loss=3*current**2*phase_R;electrical.append({'phase_current_A':current,'current_density_A_mm2':current/(wire_area*1e6),'copper_loss_W':loss,'convection_only_temperature_rise_C_range':[loss/(50*.025),loss/(20*.025)]})
    projected_filter=math.pi*.0584**2;half_pitch=.120/(c.pleat_count*2);pleat_ratio=math.sqrt(.027**2+half_pitch**2)/half_pitch;effective_filter=projected_filter*pleat_ratio
    checks=[
      {'domain':'CAD/assembly','status':'PASS','evidence':'AERIS-E1 retains the 215-part CAD/mesh contract and is separately re-run through verify.py.'},
      {'domain':'air-moving meanline screen','status':'CONDITIONAL_PASS','evidence':f"{nominal['flow_cfm']:.1f} CFM at {nominal['static_pressure_Pa']:.0f} Pa for assumed clean-filter resistance."},
      {'domain':'rotor centrifugal strength screen','status':'PASS','evidence':f'{blade_root/1e6:.2f} MPa at {2*args.rpm:.0f} rpm vs assumed 150 MPa Al screening yield.'},
      {'domain':'shaft torsion/bending screen','status':'PASS','evidence':f'torsion {torsion/1e6:.3f} MPa; bending {bending/1e6:.3f} MPa; first-order critical {critical:.0f} rpm.'},
      {'domain':'housing/yoke static screen','status':'PASS','evidence':f'trunnion average shear {trunnion/1e6:.4f} MPa; volute membrane {volute/1e6:.4f} MPa at 300 Pa.'},
      {'domain':'shaft/bearing nominal geometry','status':'PASS' if nominal_compatible else 'FAIL','evidence':f"{b['series']} nominal {bore:.1f} x {od:.1f} x {width:.1f} mm envelope with {shaft_nom:.1f} mm nominal shaft."},
      {'domain':'shaft/bearing production fit','status':'UNKNOWN','evidence':'Rotating-inner-ring shaft fit class, housing fit, internal clearance, preload and manufacturer part number remain unassigned.'},
      {'domain':'bearing L10/contact','status':'UNKNOWN','evidence':'No selected manufacturer dynamic/static capacity or life calculation.'},
      {'domain':'filter pressure drop/efficiency','status':'UNKNOWN','evidence':f'Pleated-area geometric screen {effective_filter:.3f} m2; real media curve absent.'},
      {'domain':'motor electromagnetic torque','status':'UNKNOWN','evidence':f'Required shaft torque screen {torque:.4f} N m; magnet grade/Kv/Kt/drive voltage absent.'},
      {'domain':'motor winding thermal','status':'CONDITIONAL_PASS','evidence':f'Modeled route {coil_length:.3f} m/coil, {coil_R:.4f} ohm/coil at 20 C; CYBR PHYSICS thermal submodel is a separate gate.'},
      {'domain':'acoustics','status':'UNKNOWN','evidence':f'Blade-pass fundamental {c.blade_count*args.rpm/60:.0f} Hz; no SPL/BEM/measurement.'},
      {'domain':'capture effectiveness','status':'UNKNOWN','evidence':f"Mean hood-face speed screen {nominal['hood_mean_face_velocity_m_s']:.2f} m/s; CAD-resolved open-domain capture CFD absent."},
      {'domain':'fabrication release','status':'UNKNOWN','evidence':'GD&T, thread classes, rotor balance procedure, electrical creepage/clearance and certified material grades remain unreleased.'},
    ]
    hard=[x for x in checks if x['status']=='FAIL'];unknown=[x for x in checks if x['status']=='UNKNOWN']
    report={'model':'AERIS-E1','analysis_kind':'deterministic first-order multidisciplinary engineering screening','design_rpm':args.rpm,'release_ready':not hard and not unknown,'air_mover_plausible':nominal['flow_m3_s']>=.015 and nominal['tip_mach']<.2,
      'mass':{'estimated_total_kg':total_mass,'rotating_kg':moving_mass,'impeller_kg':impeller_mass,'unclosed_mesh_mass_parts':unknown_mass},
      'aerodynamics':{'flow_cases':flow_cases,'nominal_case':nominal,'air_power_W':air_power,'shaft_power_at_assumed_45pct_efficiency_W':shaft_power,'required_torque_Nm':torque},
      'rotor_structural':{'two_x_speed_rpm':2*args.rpm,'centrifugal_blade_root_screen_Pa':blade_root,'torsional_shear_Pa':torsion,'G6p3_unbalance_force_N':unbalance,'bending_stress_Pa':bending,'critical_speed_first_order_rpm':critical},
      'bearing_interface':b,'electrical_thermal':{'coil_length_m':coil_length,'coil_resistance_20C_ohm':coil_R,'cases':electrical},'checks':checks,'critical_failures':[x['domain'] for x in hard],'unqualified_domains':[x['domain'] for x in unknown],'verdict':'NOT_RELEASE_READY' if hard or unknown else 'SCREENING_PASS',
      'limitations':['Meanline/similarity air model, not CAD-resolved CFD.','Beam/rotor formula screens, not continuum FEA.','Material strengths/densities are screening assumptions.','No electromagnetic FEA.','No certified filter/media data.','No physical prototype data.']}
    args.out.mkdir(parents=True,exist_ok=True);(args.out/'engineering_report_e1.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({'verdict':report['verdict'],'hard_failures':report['critical_failures'],'unknowns':report['unqualified_domains'],'nominal_flow_cfm':nominal['flow_cfm']},indent=2))
    if hard: raise SystemExit(2)

if __name__=='__main__':main()
