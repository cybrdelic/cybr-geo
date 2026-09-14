"""Export CAD-derived AERIS-E1 parameters for CYBR PHYSICS submodels."""
from __future__ import annotations
from pathlib import Path
import argparse, math, pickle
import numpy as np
import trimesh
DENSITY={0:2700.,1:2700.,2:2700.,3:8960.,4:1100.,5:1150.,6:8000.,7:2700.,8:1400.,9:1100.,10:1200.}
G=9.80665

def volume_mm3(p):
    if p.cad is not None:return float(p.cad.Volume())
    m=trimesh.Trimesh(np.asarray(p.vertices),np.asarray(p.faces),process=True)
    return abs(float(m.volume)) if m.is_watertight else 0.0

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,default=Path('build/aeris-e1'));args=ap.parse_args();root=args.out
    with (root/'assembly.pkl').open('rb') as f:a=pickle.load(f)
    masses={p.name:volume_mm3(p)*1e-9*DENSITY[int(p.material)] for p in a.parts}
    impeller=[p for p in a.parts if p.group=='impeller'];moving=[p for p in a.parts if p.motion=='rotor']
    impeller_mass=sum(masses[p.name] for p in impeller);rotating_mass=sum(masses[p.name] for p in moving);total_mass=sum(masses.values())
    rotor_cg_x=sum(masses[p.name]*float(np.mean(p.bounds[:,0])) for p in impeller)/impeller_mass
    c=a.metadata['functional_assembly'];b=a.metadata['bearing_interface']
    kv={'schema':'aeris-cybr-physics-1','revision':a.metadata.get('engineering_revision','UNKNOWN'),'parts':len(a.parts),
        'total_mass_kg':total_mass,'rotating_mass_kg':rotating_mass,'impeller_mass_kg':impeller_mass,'impeller_cg_x_m':rotor_cg_x/1000.,
        'rotor_radius_m':.064,'rotor_axial_width_m':.030,'shaft_diameter_m':.010,'shaft_length_m':.110,
        'bearing_x0_m':float(c['bearings_x_mm'][0])/1000.,'bearing_x1_m':float(c['bearings_x_mm'][1])/1000.,
        'bearing_nominal_bore_m':float(b['nominal_bore_mm'])/1000.,'bearing_nominal_od_m':float(b['nominal_od_mm'])/1000.,'bearing_width_m':float(b['nominal_width_mm'])/1000.,
        'hood_area_m2':math.pi*.082*.061,'outlet_area_m2':.0316*.0416,'filter_projected_area_m2':math.pi*.0584**2,
        'blade_count':11,'design_rpm':3600.,'overspeed_rpm':7200.,'gravity_N':total_mass*G,
        'screen_pressure_Pa':247.,'screen_shaft_torque_Nm':.0267,'air_density_kg_m3':1.204,'air_kinematic_viscosity_m2_s':1.516e-5,
        'stator_conductor_radius_m':.00042,'screen_phase_current_A':5.0}
    target=root/'aeris_cybr_physics.manifest'
    with target.open('w') as f:
        for k,v in kv.items():f.write(f'{k}={v}\n')
    print(target)
    for k in sorted(kv):print(k,kv[k])
if __name__=='__main__':main()
