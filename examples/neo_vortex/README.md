# REV NEO Vortex / SPARK Flex mechanical study

This study imports and inspects the manufacturer's complete published mechanical CAD. It preserves **75 solids plus one surface body**, with 76 separate image/model exports. Raw import and styled study remain separate. The controller, connectors, cables, shaft, shell, bearing geometry and fasteners are included. The long cable bodies are retained in the complete model but hidden in body-focused views.

`renders/hero_pathtraced.png` and `exploded_pathtraced.png` are new Monte Carlo path-traced stills; raw and filtered versions are both present. `videos/motor_inspection.mp4` is 240 actual geometry-rendered frames at 1280x720/24 fps. The outer rotor can and shaft move together; the stationary motor frame and controller do not rotate. The explosion is an inspection layout, not a manufacturer's service procedure.

The official product specifies a 12 V motor, 8 mm keyed-shaft option, 6784 rpm free speed, 3.6 Nm stall torque, 211 A stall current and 640 W peak power. These are manufacturer quantities, **not a safe continuous operating point**. The drawing specifies six #10-32 mounting threads on a 50.8 mm circle and a maximum mounting depth of 6.3 mm. The retrieved mounting layout uses 0,45,135,180,225,315-degree locations rather than a regular 60-degree pattern.

A source-and-mechanical-CAD model is not a complete electromagnetic or electronics reverse engineering: the supplied model does not expose detailed copper winding paths, magnet segmentation or PCB circuitry. Those omissions are explicitly retained rather than invented. No thermal, torque or load simulation of the motor was performed.

Sources and original rights are recorded in [PROVENANCE](../../docs/PROVENANCE.md). The custom motor-to-carrier interface is documented separately in [motor_drive](../motor_drive/README.md).
