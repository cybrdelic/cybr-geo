# Custom motor-to-carrier drive study

This is a **prototype interface concept, not a bolt-on compatibility determination**. The differential was reconstructed from an inconsistent generated concept image and has no validated torque rating. The illustrated 260 Nm value is not a manufacturer's rating and is not used to size this interface.

The model combines the preserved v3 alternative differential assembly (119 meshes), 70 motor bodies with long leads hidden, and 11 new interface pieces, for **200 named meshes**. The original reconstruction and original alternative core are not overwritten.

The motor's 20-tooth spur pinion drives an 80-tooth gear attached to the carrier's rear flange, not to an output shaft. Module is 1.75 mm, pressure angle is 20 degrees, nominal centre distance is 87.5 mm, and the input pinion face is 10 mm wide with 10 mm of modeled shaft engagement. The carrier gear is 15 mm wide, with eight 9.8 mm flange holes at radius49.8 mm and a76 mm centre clearance.

The motor's mounting plate has six 5.2 mm clearance holes on a50.8 mm pitch circle at0,45,135,180,225,315 degrees. These clear the nominal4.826 mm major diameter of a#10 screw. Thread engagement must still respect REV's6.3 mm maximum mounting depth. The L bracket is one analytic unioned CAD solid.

`drive_motion.mp4` has192 newly rendered1280x720 frames at24 fps. `drive_cutaway.mp4` has144 fixed-camera geometry frames. Motion is prescribed: motor angle=-4*carrier angle; left+right=2*carrier. 601 sampled algebraic checks are recorded in`validation.json`. 241 sampled 2D gear-profile checks recorded zero intersection area and a minimum modeled gap of0.093995 mm. Those checks are not a proof of all-phase 3D clearance or loaded tooth contact.

Not qualified: shaft-key orientation and retention, bearing support and reaction loads, complete base mounting, fastener preload, tooth strength, backlash under load, speed rating, enclosure/guards, lubrication, thermal behaviour and continuous motor loading. The tooth-root transitions remain approximate. Do not energize this exposed, unqualified gearing as though it were finished hardware.

The mixed assembly's GLB contains all200 meshes. Its STEP contains only the11 newly authored analytic interface bodies; it is not falsely presented as a complete CAD differential. The full motor STEP is supplied separately in the motor study.
