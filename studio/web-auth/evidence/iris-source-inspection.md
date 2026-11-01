# IRIS-09 source inspection

Read-only benchmark inspection of [cybrdelic/cybr-iris09 at commit 6aac71594c211cc5934674b2113d9db8c5ec17c6](https://github.com/cybrdelic/cybr-iris09/tree/6aac71594c211cc5934674b2113d9db8c5ec17c6). This is a source analysis, not a rendered demonstration or validation of the mechanism. No source, shader, image, film or geometry from that repository was copied into CYBR GEO.

The repository README describes an animated visual study and excludes manufacturing/mechanical-feasibility claims. Inspection found no supplied project license granting reuse. Public availability alone does not establish those rights.

The readable shader constructs an annular body with retaining details, three mounting ears, screw-head details and a moving thumb handle. Its body is one analytic distance field with material IDs, rather than a body/interface/load graph. Nine equally spaced leaves use analytic intersections of circular profile constraints and thin canted slabs. Optical rendering and mechanical representation are separate concerns.

Handle and leaf angle both follow prescribed time functions. Their algebraic relation is not a solved pin-slot linkage, transmission closure or torque path. The viewer runs one full-screen WebGL2 shader draw with time/camera uniforms; it does not hide a mesh hierarchy or a rigid-body dynamics engine behind the shader.

An especially relevant distinction is the stationary world-space clipping cylinder used inside each leaf intersection. As the local leaf profile moves, its displayed boundary can be clipped differently. That visual construction is not sufficient evidence of an invariant rigid leaf solid. A CAD mechanism must preserve each body's local geometry and model housing occlusion separately.

The source comments include a sampled clearance assertion in scene units. This checkout supplies neither its verifier nor a physical unit/load mapping. That assertion was not transferred into CYBR GEO evidence. No claim from this design applies to either ARX source.

Useful requirements for the shared architecture are reusable component descriptors; local frames and immutable component geometry; repeated component arrays; explicit retained/bushed interfaces and load paths; drive causation distinct from prescribed animation; nonlinear closure or pin-slot constraints when required; real mass properties; and validation bound to both parameter domains and source identity. Current CYBR examples exercise the shared rotary-cartridge/gripper catalog. They do not reproduce the nine-leaf iris or establish that the current catalog can synthesize it.

The repository viewer embeds the same readable shader as the standalone file after newline normalization (43,865 characters; SHA-256 `455e1ba9b0740a6ab925f9c022eea5f217fd3bbfc08ffec3177fb3b1d925843b`). Its compact variant also matches its standalone source (22,533 characters; SHA-256 `65d9cc416dd9fc460cc7753110d6878c5d5966f624df98489dc71f269a08a0ca`). These are source-text consistency checks, not simulation evidence.
