"""Export deterministic FUSE C220 engineering inputs to CYBR PHYSICS."""
from pathlib import Path
import argparse, json, math
from printer import RP, LEAD, VOLUME


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,default=Path('build/fuse-c220-engineering'));args=ap.parse_args();out=args.out
    report=json.loads((out/'FUSE_C220_engineering_report.json').read_text())
    m=report['mass_screen'];d=report['drive_screen'];s=report['structure_screen']
    kv={
      'schema':'fuse-c220-cybr-physics-1','revision':'FUSE-C220-E1','total_mass_kg':m['estimated_total_kg'],
      'head_mass_kg':m['head_kg'],'bed_mass_kg':m['bed_kg'],'gantry_head_mass_kg':m['gantry_plus_head_kg'],
      'axis_acceleration_m_s2':d['acceleration_m_s2'],'x_gantry_span_m':s['x_gantry_span_m'],
      'x_extrusion_nominal_width_m':.020,'x_extrusion_effective_I_m4':s['x_effective_I_m4'],
      'pulley_pitch_radius_m':RP/1000.,'belt_pitch_m':.002,'belt_width_m':.006,
      'z_lead_m_per_rev':LEAD/1000.,'z_screw_count':2,'z_screw_root_screen_m':s['z_screw_assumed_root_m'],
      'z_screw_unsupported_screen_m':s['z_screw_assumed_unsupported_m'],
      'bed_plate_x_m':.235,'bed_plate_y_m':.235,'bed_plate_thickness_m':.004,
      'hotend_block_x_m':.018,'hotend_block_y_m':.014,'hotend_block_z_m':.009,
      'nozzle_orifice_m':.00040,'filament_diameter_m':.00175,
      'hotend_target_K':210.+273.15,'bed_target_K':60.+273.15,
      # Deliberate sensitivity powers: these are NOT selected heater ratings.
      'hotend_screen_power_W':40.,'bed_screen_power_W':200.,
      'aluminum_density_kg_m3':2700.,'aluminum_young_modulus_Pa':69e9,
      'build_x_m':VOLUME[0]/1000.,'build_y_m':VOLUME[1]/1000.,'build_z_m':VOLUME[2]/1000.,
    }
    path=out/'FUSE_C220_cybr_physics.manifest'
    with path.open('w') as f:
        for k,v in kv.items():f.write(f'{k}={v}\n')
    print(path)
    for k in sorted(kv):print(k,kv[k])

if __name__=='__main__':main()
