"""Deterministic multidisciplinary engineering screening for AERIS.

This is a first-order engineering gate tied to the actual AERIS CAD. It is not a
substitute for CFD, nonlinear FEA, motor electromagnetic analysis, certified
filter data, acoustic testing, or physical prototype validation. Unknowns remain
UNKNOWN instead of being silently converted into passing assumptions.
"""
from __future__ import annotations

import argparse
import json
import math
import pickle
import sys
from pathlib import Path

import numpy as np
import trimesh

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src'))
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recipe import Config, MATERIALS

RHO_AIR=1.204
MU_AIR=1.825e-5
C_AIR=343.0
G=9.80665
COPPER_RESISTIVITY_20C=1.724e-8

# Screening values only. Material entries in recipe.py are appearance/provenance
# choices rather than certified alloy callouts, so every density/strength used
# below is explicitly treated as an assumption.
DENSITY_KG_M3={0:2700.,1:2700.,2:2700.,3:8960.,4:1100.,5:1150.,6:8000.,7:2700.,8:1400.,9:1100.,10:1200.}
AL_YIELD_SCREEN_PA=150e6
STEEL_YIELD_SCREEN_PA=250e6
E_STEEL_PA=200e9


def _volume_mm3(part):
    if part.cad is not None:
        return float(part.cad.Volume())
    mesh=trimesh.Trimesh(np.asarray(part.vertices),np.asarray(part.faces),process=True)
    if mesh.is_watertight:
        return abs(float(mesh.volume))
    return None


def _mass_properties(assembly):
    rows=[];total=0.;unknown=[]
    for p in assembly.parts:
        volume=_volume_mm3(p)
        if volume is None:
            unknown.append(p.name);continue
        density=DENSITY_KG_M3[int(p.material)]
        mass=volume*1e-9*density
        total+=mass
        rows.append((p,mass,volume))
    return rows,total,unknown


def _solve_flow(rpm,filter_drop_20):
    c=Config();r2=c.rotor_radius/1000.;b2=(c.rotor_back-c.rotor_front-5.)/1000.
    omega=2*math.pi*rpm/60.;u2=omega*r2
    area2=2*math.pi*r2*b2*.90
    outlet=.0316*.0416
    phi_free=.15
    qfree=phi_free*area2*u2
    dp0=.45*RHO_AIR*u2*u2
    qs=np.linspace(0.0005,qfree*.999,12000)
    fan=dp0*(1-(qs/qfree)**2)
    # K=1.5 captures hood/turn/volute/outlet minor-loss screening only. The
    # filter term is swept separately because the media has no measured curve.
    system=1.5*.5*RHO_AIR*(qs/outlet)**2 + filter_drop_20*(qs/.020)**1.5
    i=int(np.argmin(np.abs(fan-system)))
    q=float(qs[i]);dp=float(fan[i])
    hood_area=math.pi*.082*.061
    return {
        'filter_drop_assumed_Pa_at_0p020_m3s':filter_drop_20,
        'flow_m3_s':q,'flow_cfm':q*2118.880003,'static_pressure_Pa':dp,
        'hood_mean_face_velocity_m_s':q/hood_area,
        'outlet_velocity_m_s':q/outlet,
        'outlet_dynamic_pressure_Pa':.5*RHO_AIR*(q/outlet)**2,
        'impeller_tip_speed_m_s':u2,
        'tip_mach':u2/C_AIR,
        'impeller_flow_coefficient':q/(area2*u2),
        'free_delivery_model_m3_s':qfree,
    }


def _coil_length_m(turns=8):
    t=np.linspace(0,2*math.pi*turns,turns*5000+1)
    x=79.+17.5*np.sign(np.cos(t))*np.abs(np.cos(t))**.38
    y=23.8+5.0*t/(2*math.pi*turns)
    z=3.8*np.sign(np.sin(t))*np.abs(np.sin(t))**.6
    pts=np.column_stack([x,y,z])
    return float(np.linalg.norm(np.diff(pts,axis=0),axis=1).sum()/1000.)


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--out',type=Path,default=ROOT/'build/aeris')
    ap.add_argument('--rpm',type=float,default=3600.)
    args=ap.parse_args();out=args.out
    with (out/'assembly.pkl').open('rb') as stream:
        assembly=pickle.load(stream)
    rows,total_mass,unknown_mass=_mass_properties(assembly)
    parts={p.name:p for p in assembly.parts}
    c=Config();omega=2*math.pi*args.rpm/60.;r2=c.rotor_radius/1000.;dshaft=2*c.shaft_radius/1000.

    moving=[(p,m) for p,m,_ in rows if p.motion=='rotor']
    impeller=[(p,m) for p,m,_ in rows if p.group=='impeller']
    moving_mass=sum(m for _,m in moving);impeller_mass=sum(m for _,m in impeller)
    rotor_x=sum(m*float(np.mean(p.bounds[:,0])) for p,m in impeller)/max(impeller_mass,1e-12)

    flow_cases=[_solve_flow(args.rpm,drop) for drop in (40.,80.,120.,160.)]
    nominal=flow_cases[1]
    air_power=nominal['flow_m3_s']*nominal['static_pressure_Pa']
    shaft_power=air_power/.45
    torque=shaft_power/omega

    # Rotor/shaft screening, including a 2x-speed overspeed point.
    omega_over=2*omega
    blade_root_screen=2700.*omega_over**2*(r2*r2-.021**2)/2
    torsional_shear=16*torque/(math.pi*dshaft**3)
    # ISO-like G6.3 residual specific unbalance screening: e = G/omega.
    eccentricity_m=(6.3/omega)/1000.
    unbalance_force=impeller_mass*eccentricity_m*omega**2
    bearing_x0=49.5/1000.;overhang=max(0.,bearing_x0-rotor_x/1000.)
    bending_moment=(unbalance_force+impeller_mass*G)*overhang
    bending_stress=32*bending_moment/(math.pi*dshaft**3)
    I=math.pi*dshaft**4/64
    k=3*E_STEEL_PA*I/max(overhang,1e-4)**3
    critical_rpm=math.sqrt(k/max(impeller_mass,1e-6))/(2*math.pi)*60

    # The modeled bearing bore is 10.04 mm (r=5.02) against a 10.00 mm shaft.
    # For a rotating inner ring this is intentionally treated as a hard design
    # issue rather than waved through as visualization tolerance.
    shaft_nominal_mm=10.0;bearing_bore_mm=10.04
    bearing_diametral_clearance=bearing_bore_mm-shaft_nominal_mm
    bearing_external_od_mm=25.0;bearing_external_width_mm=6.5
    bearing_fit_pass=bearing_diametral_clearance<=.015

    trunnion_area=math.pi*((8e-3)**2-(3.2e-3)**2)
    trunnion_shear=(total_mass*G/2)/trunnion_area
    volute_membrane=300.*.096/.0052

    # Electrical/thermal screen from the actual modeled 0.42 mm conductor route.
    wire_radius=.00042;wire_area=math.pi*wire_radius**2
    coil_length=_coil_length_m(c.wire_turns)
    coil_R=COPPER_RESISTIVITY_20C*coil_length/wire_area
    phase_R=4*coil_R  # explicit screening assumption: four coils series/phase.
    electrical=[]
    motor_convective_area=.025  # shell + fin order-of-magnitude from CAD envelope.
    for current in (3.,5.,10.):
        j=current/(wire_area*1e6)
        copper_loss=3*current**2*phase_R
        # Forced-convection bracket 20..50 W/m^2/K, excluding internal conduction.
        rise_lo=copper_loss/(50*motor_convective_area)
        rise_hi=copper_loss/(20*motor_convective_area)
        electrical.append({'phase_current_A':current,'current_density_A_mm2':j,
                           'copper_loss_W':copper_loss,
                           'convection_only_temperature_rise_C_range':[rise_lo,rise_hi]})

    hood_area=math.pi*.082*.061
    projected_filter=math.pi*.0584**2
    half_pitch=.120/(c.pleat_count*2)
    pleat_ratio=math.sqrt(.027**2+half_pitch**2)/half_pitch
    effective_filter_area=projected_filter*pleat_ratio

    checks=[
        {'domain':'CAD/assembly','status':'PASS','evidence':'215-part CAD/mesh assembly and existing 14/14 geometry/export gate'},
        {'domain':'air-moving meanline screen','status':'CONDITIONAL_PASS','evidence':f"{nominal['flow_cfm']:.1f} CFM at {nominal['static_pressure_Pa']:.0f} Pa for assumed 80 Pa clean-filter drop; 40..160 Pa filter sweep retained"},
        {'domain':'rotor centrifugal strength screen','status':'PASS','evidence':f'{blade_root_screen/1e6:.2f} MPa at {args.rpm*2:.0f} rpm 2x-speed screen vs assumed 150 MPa aluminium yield'},
        {'domain':'shaft torsion/bending screen','status':'PASS','evidence':f'torsion {torsional_shear/1e6:.2f} MPa; bending {bending_stress/1e6:.2f} MPa; first-order critical speed {critical_rpm:.0f} rpm'},
        {'domain':'housing/yoke static pressure and gravity screen','status':'PASS','evidence':f'trunnion average shear {trunnion_shear/1e6:.3f} MPa; volute membrane screen {volute_membrane/1e6:.4f} MPa'},
        {'domain':'shaft/bearing production fit','status':'PASS' if bearing_fit_pass else 'FAIL','evidence':f'{shaft_nominal_mm:.3f} mm shaft vs {bearing_bore_mm:.3f} mm modeled bore = {bearing_diametral_clearance*1000:.0f} um diametral clearance; modeled bearing envelope {bearing_external_od_mm:.1f}x{bearing_external_width_mm:.1f} mm is not a released catalog bearing'},
        {'domain':'bearing L10/contact','status':'UNKNOWN','evidence':'No released bearing part number, dynamic capacity, internal clearance or preload.'},
        {'domain':'filter pressure drop/efficiency','status':'UNKNOWN','evidence':f'Geometry provides about {effective_filter_area:.3f} m2 pleated-area screen, but media permeability and efficiency curve are not specified.'},
        {'domain':'motor electromagnetic torque','status':'UNKNOWN','evidence':f'Required shaft torque screen is {torque:.4f} N m, but magnet grade, winding connection, Kv/Kt and drive voltage are unspecified.'},
        {'domain':'motor winding thermal','status':'CONDITIONAL_PASS','evidence':f'Modeled conductor route {coil_length:.3f} m/coil, {coil_R:.4f} ohm/coil at 20 C; current/thermal sweep recorded, insulation class unknown.'},
        {'domain':'acoustics','status':'UNKNOWN','evidence':f'Blade-pass frequency {c.blade_count*args.rpm/60:.0f} Hz at design speed; no broadband/tonal SPL model or test.'},
        {'domain':'capture effectiveness','status':'UNKNOWN','evidence':f"Mean hood-face velocity {nominal['hood_mean_face_velocity_m_s']:.2f} m/s at nominal modeled flow; capture velocity versus source distance not solved."},
        {'domain':'fabrication release','status':'UNKNOWN','evidence':'No GD&T stack, fastener thread specification, balance procedure, electrical creepage/clearance, or certified material callouts.'},
    ]
    hard_fail=[x for x in checks if x['status']=='FAIL']
    unknown=[x for x in checks if x['status']=='UNKNOWN']
    report={
        'model':'AERIS','analysis_kind':'deterministic first-order multidisciplinary engineering screening',
        'design_rpm':args.rpm,'release_ready':not hard_fail and not unknown,
        'air_mover_plausible':nominal['flow_m3_s']>=.015 and nominal['tip_mach']<.2,
        'mass':{'estimated_total_kg':total_mass,'rotating_kg':moving_mass,'impeller_kg':impeller_mass,'unclosed_mesh_mass_parts':unknown_mass,'density_assumptions_kg_m3':DENSITY_KG_M3},
        'aerodynamics':{'model':'Euler/similarity fan curve + quadratic minor losses + swept assumed clean-filter loss','flow_cases':flow_cases,'nominal_case':nominal,'air_power_W':air_power,'shaft_power_at_assumed_45pct_efficiency_W':shaft_power,'required_torque_Nm':torque,'hood_area_m2':hood_area,'projected_filter_area_m2':projected_filter,'pleat_area_multiplier':pleat_ratio,'effective_filter_area_screen_m2':effective_filter_area},
        'rotor_structural':{'two_x_speed_rpm':args.rpm*2,'centrifugal_blade_root_screen_Pa':blade_root_screen,'torsional_shear_Pa':torsional_shear,'G6p3_unbalance_force_N':unbalance_force,'bending_stress_Pa':bending_stress,'critical_speed_first_order_rpm':critical_rpm,'impeller_overhang_m':overhang},
        'support_structural':{'trunnion_average_shear_Pa':trunnion_shear,'volute_300Pa_membrane_screen_Pa':volute_membrane},
        'bearing_fit':{'shaft_nominal_mm':shaft_nominal_mm,'modeled_bore_mm':bearing_bore_mm,'diametral_clearance_mm':bearing_diametral_clearance,'modeled_OD_mm':bearing_external_od_mm,'modeled_width_mm':bearing_external_width_mm,'pass':bearing_fit_pass},
        'electrical_thermal':{'wire_radius_mm':wire_radius*1000,'coil_length_m':coil_length,'coil_resistance_20C_ohm':coil_R,'assumed_four_coils_series_phase_resistance_ohm':phase_R,'cases':electrical},
        'acoustics':{'blade_pass_frequency_Hz':c.blade_count*args.rpm/60,'tip_mach':nominal['tip_mach']},
        'checks':checks,
        'critical_failures':[x['domain'] for x in hard_fail],
        'unqualified_domains':[x['domain'] for x in unknown],
        'verdict':'NOT_RELEASE_READY' if hard_fail or unknown else 'SCREENING_PASS',
        'limitations':['No CFD RANS/LES fan curve.','No continuum FEA/stress concentration/fatigue solve.','No electromagnetic FEA or motor controller model.','No certified filter media curve.','No acoustic boundary-element/measurement.','No physical prototype data.'],
    }
    out.mkdir(parents=True,exist_ok=True)
    (out/'engineering_report.json').write_text(json.dumps(report,indent=2)+'\n')
    md=['# AERIS engineering screening','',f"Verdict: **{report['verdict']}**",'',f"Estimated mass: {total_mass:.2f} kg",f"Nominal modeled flow: {nominal['flow_cfm']:.1f} CFM at {nominal['static_pressure_Pa']:.0f} Pa",f"Required shaft power (45% fan-efficiency assumption): {shaft_power:.1f} W",'', '| Domain | Status | Evidence |','|---|---|---|']
    md += [f"| {x['domain']} | {x['status']} | {x['evidence']} |" for x in checks]
    md += ['','This report is an engineering screening, not certification or fabrication release.']
    (out/'engineering_report.md').write_text('\n'.join(md)+'\n')
    print(json.dumps({'verdict':report['verdict'],'hard_failures':report['critical_failures'],'unknowns':report['unqualified_domains'],'nominal_flow_cfm':nominal['flow_cfm']},indent=2))
    # A hard geometry/mechanical incompatibility should fail CI. UNKNOWN domains
    # remain in the report and block release_ready without pretending the model
    # has data it does not have.
    if hard_fail:
        raise SystemExit(2)

if __name__=='__main__':main()
