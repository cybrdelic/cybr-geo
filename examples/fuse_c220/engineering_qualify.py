"""Deterministic first-order engineering qualification for FUSE C220.

The objective is to turn the printer's nominal CAD/kinematic proof into an
engineering screen without converting missing supplier or physical-test data into
fake passes.  All dimensions come from the active printer recipe; density,
material strength and drivetrain-efficiency values below are explicitly screening
assumptions until bound to purchased parts and measured specimens.
"""
from __future__ import annotations
import argparse, json, math, sys
from pathlib import Path
import numpy as np
import trimesh

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
sys.path.insert(0,str(ROOT/'src'));sys.path.insert(0,str(HERE))
from printer import build, RP, LEAD, VOLUME
from toolpath import generate, verify as verify_toolpath, BEAD_AREA

G=9.80665
E_AL=69e9
E_STEEL=200e9
# Material-index density assumptions matching the visual material ledger in printer.py.
DENSITY_KG_M3={0:2700.,1:2700.,2:7850.,3:7850.,4:2700.,5:8500.,6:1200.,7:7800.,8:1240.,9:8960.,10:2500.,11:1200.,12:1500.,13:1200.,14:0.,15:3800.}


def volume_mm3(p):
    if p.cad is not None:return float(p.cad.Volume())
    m=trimesh.Trimesh(np.asarray(p.vertices),np.asarray(p.faces),process=True)
    if m.is_watertight:return abs(float(m.volume))
    return None


def mass_properties(a):
    rows=[];unknown=[]
    for p in a.parts:
        v=volume_mm3(p)
        if v is None:unknown.append(p.name);continue
        rho=DENSITY_KG_M3.get(int(p.material))
        if rho is None:unknown.append(p.name);continue
        rows.append((p,v*1e-9*rho,v))
    return rows,unknown


def moving_mass(rows, motions):
    return sum(m for p,m,_ in rows if p.motion in motions)


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,default=ROOT/'build/fuse-c220-engineering');args=ap.parse_args()
    out=args.out;out.mkdir(parents=True,exist_ok=True)
    a=build();rows,unknown_mass=mass_properties(a)
    total=sum(m for _,m,_ in rows)
    # Motions that translate with the physical assemblies. Rotary subparts are
    # included with their parent carriage/gantry for conservative inertial mass.
    head_m=moving_mass(rows,{'head','e_spin','e_idler_spin'})
    bed_m=moving_mass(rows,{'bed'})
    gantry_m=moving_mass(rows,{'gantry','x_spin','x_idler_spin'})+head_m

    accel=600./1000. # recipe timing acceleration, m/s^2
    rp=RP/1000.;lead=LEAD/1000.
    x_force=head_m*accel; y_force=bed_m*accel
    x_torque=x_force*rp; y_torque=y_force*rp
    z_eff=.35 # deliberately conservative first-order screw efficiency assumption
    z_load=gantry_m*G
    z_torque_each=(z_load/2)*lead/(2*math.pi*z_eff)

    # X gantry screen: 20x20 mm custom slotted profile. The 0.55 effective-I
    # factor is an explicit surrogate for the unmodeled exact vendor extrusion.
    span=.350; b=.020; I_solid=b**4/12.; I_eff=.55*I_solid
    worst_head_load=head_m*math.hypot(G,accel)
    x_deflection=worst_head_load*span**3/(48*E_AL*I_eff)
    x_bending=worst_head_load*span/4.; x_stress=x_bending*(b/2)/I_eff

    # Z screw buckling screen. 7.3 mm root and 0.38 m unsupported length are
    # screening assumptions, not a vendor T8x8 root-diameter certificate.
    screw_root=.0073; screw_I=math.pi*screw_root**4/64.; screw_L=.380
    pcr=math.pi**2*E_STEEL*screw_I/screw_L**2; z_sf=pcr/max(z_load/2,1e-12)

    # Bed plate gravity screen as a conservative simply-supported strip across
    # the 235 mm span, using the actual 4 mm aluminium heater plate thickness.
    plate_span=.235; plate_w=.235; plate_t=.004; plate_I=plate_w*plate_t**3/12.
    bed_deflection=5*(bed_m*G)*plate_span**3/(384*E_AL*plate_I)

    # Re-run the real supplied calibration G-code and use its positive-E moves.
    gcode=out/'FUSE_C220_engineering_calibration.gcode';tp=generate(gcode);tp_report=verify_toolpath(tp)
    max_flow=max(m.feed*BEAD_AREA for m in tp.deposits)

    # Supplier-independent geometry/physics screens can PASS. Anything depending
    # on catalog ratings, controller wiring or measured thermomechanics stays UNKNOWN.
    checks=[
      {'domain':'nominal CAD/kinematics','status':'PASS','evidence':'Existing independent mechanism/hotend/G-code workflows are required alongside this report.'},
      {'domain':'X gantry static/inertial screen','status':'PASS' if x_deflection<.00010 and x_stress<40e6 else 'FAIL','evidence':f'equivalent 20 mm slotted-beam screen: {x_deflection*1e6:.1f} um deflection, {x_stress/1e6:.2f} MPa at 600 mm/s^2.'},
      {'domain':'Z screw Euler buckling screen','status':'PASS' if z_sf>4 else 'FAIL','evidence':f'assumed 7.3 mm root / 380 mm unsupported screw gives first-order buckling SF {z_sf:.1f}.'},
      {'domain':'bed plate gravity screen','status':'PASS' if bed_deflection<.00015 else 'FAIL','evidence':f'235x235x4 mm aluminium strip surrogate gives {bed_deflection*1e6:.1f} um gravity deflection.'},
      {'domain':'toolpath volumetric flow','status':'PASS' if tp_report['all_passed'] and max_flow<5. else 'FAIL','evidence':f'generated calibration path max imposed bead flow {max_flow:.3f} mm3/s; toolpath verifier all_passed={tp_report["all_passed"]}.'},
      {'domain':'X/Y motor torque margin','status':'UNKNOWN','evidence':f'ideal inertial torque demand X={x_torque:.4f} N m, Y={y_torque:.4f} N m; selected motor torque-speed curves, belt preload/friction and driver current absent.'},
      {'domain':'Z motor/lead-screw torque margin','status':'UNKNOWN','evidence':f'first-order lift demand {z_torque_each:.4f} N m per screw using assumed 35% screw efficiency; selected motor/nut friction data absent.'},
      {'domain':'belt strength/tension/life','status':'UNKNOWN','evidence':'6 mm 2 mm-pitch closed paths are modeled, but belt construction, rated working tension, preload, tooth shear and fatigue curve are not supplier-bound.'},
      {'domain':'linear-guide preload/life','status':'UNKNOWN','evidence':'Guide geometry/travel is verified, but rail/block manufacturer preload, C/C0 ratings and lubrication are unassigned.'},
      {'domain':'hotend thermal performance','status':'UNKNOWN','evidence':'Continuous feed/melt passage is verified and G-code targets 210 C, but heater power, thermistor transfer function, melt pressure and heatbreak thermal calibration are unmeasured.'},
      {'domain':'bed thermal performance','status':'UNKNOWN','evidence':'G-code target is 60 C; heater wattage, temperature uniformity, insulation and thermal runaway behavior are not physically validated.'},
      {'domain':'electrical/control safety','status':'UNKNOWN','evidence':'24 V packaging envelope exists; PSU/controller part numbers, fusing, wire gauges, grounding, creepage, connector ratings, firmware pins and protection behavior are unreleased.'},
      {'domain':'tolerance/backlash stack','status':'UNKNOWN','evidence':'Nominal CAD interfaces pass, but purchased-part tolerances, printed-part shrinkage, belt stretch, screw backlash, rail play and assembly datum stack are not measured.'},
      {'domain':'vibration/modal/input shaping','status':'UNKNOWN','evidence':'No CAD-resolved modal solve or accelerometer measurement; firmware input-shaping parameters cannot be qualified digitally.'},
      {'domain':'molten polymer/adhesion/print quality','status':'UNKNOWN','evidence':'Deposited beads are imposed geometry; no melt CFD, pressure-advance model, layer adhesion, dimensional coupon or surface-quality measurement.'},
      {'domain':'physical build qualification','status':'UNKNOWN','evidence':'No physical FUSE C220 has been assembled, instrumented or printed on.'}]
    hard=[x for x in checks if x['status']=='FAIL'];unknown=[x for x in checks if x['status']=='UNKNOWN']
    report={'model':'CYBR FUSE C220','analysis_kind':'deterministic first-order multidisciplinary engineering screening',
      'release_ready':not hard and not unknown,'verdict':'NOT_RELEASE_READY' if hard or unknown else 'SCREENING_PASS',
      'mass_screen':{'estimated_total_kg':total,'head_kg':head_m,'bed_kg':bed_m,'gantry_plus_head_kg':gantry_m,'unknown_mass_parts':unknown_mass,'density_assumptions_kg_m3':DENSITY_KG_M3},
      'drive_screen':{'acceleration_m_s2':accel,'pulley_pitch_radius_m':rp,'x_ideal_inertial_force_N':x_force,'y_ideal_inertial_force_N':y_force,'x_ideal_inertial_torque_Nm':x_torque,'y_ideal_inertial_torque_Nm':y_torque,'z_assumed_efficiency':z_eff,'z_lift_torque_each_Nm':z_torque_each},
      'structure_screen':{'x_gantry_span_m':span,'x_effective_I_m4':I_eff,'x_deflection_m':x_deflection,'x_bending_stress_Pa':x_stress,'z_screw_assumed_root_m':screw_root,'z_screw_assumed_unsupported_m':screw_L,'z_euler_critical_load_each_N':pcr,'z_buckling_safety_factor':z_sf,'bed_plate_deflection_m':bed_deflection},
      'toolpath_screen':{'max_imposed_volumetric_flow_mm3_s':max_flow,'verification':tp_report},
      'build_volume_mm':list(VOLUME),'checks':checks,'critical_failures':[x['domain'] for x in hard],'unqualified_domains':[x['domain'] for x in unknown],
      'limitations':['First-order beam/plate/buckling formulas, not continuum/modal FEA.','Density and effective-section values are screening assumptions.','No motor electromagnetic or driver model.','No thermal calibration or molten-polymer CFD.','No supplier tolerance/life data.','No physical printer data.']}
    (out/'FUSE_C220_engineering_report.json').write_text(json.dumps(report,indent=2)+'\n')
    md=['# FUSE C220 engineering screening','',f"Verdict: **{report['verdict']}**",'',f'Estimated screened mass: {total:.2f} kg',f'Head mass: {head_m:.3f} kg; bed mass: {bed_m:.3f} kg','', '| Domain | Status | Evidence |','|---|---|---|']
    md += [f"| {c['domain']} | {c['status']} | {c['evidence']} |" for c in checks]
    md += ['','This is a computational screening package, not a manufacturing or safety release.']
    (out/'FUSE_C220_engineering_report.md').write_text('\n'.join(md)+'\n')
    print(json.dumps({'verdict':report['verdict'],'hard_failures':report['critical_failures'],'unknowns':report['unqualified_domains'],'head_kg':head_m,'bed_kg':bed_m},indent=2))
    if hard:raise SystemExit(2)

if __name__=='__main__':main()
