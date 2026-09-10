"""Parametric straight-wire Nitinol fiber bundle actuator.

The default design is an engineering concept anchored to Dynalloy's published
0.008 in / 0.20 mm, 70 C Flexinol actuator-wire guide values.  The geometry is
not vendor CAD and the calculations are not a thermal, fatigue, contact, or
structural qualification.

Coordinates follow Mechanism Lab convention: millimetres, X-axis actuation,
Z-up.  The moving carriage translates toward -X as the wire bundle heats.
"""
from __future__ import annotations
from dataclasses import dataclass, asdict
import math
import numpy as np

from ..core import Assembly, Material, View, Part, cad_part, mesh_part
from ..geometry import ring, drill, bolt_circle, tube_mesh

G0 = 9.80665
SOURCE_WIRE_DATA = "https://dynalloy.com/technical-data-wires/"
SOURCE_TECH_SHEET = "https://dynalloy.com/wp-content/uploads/2025/03/TCF1140.pdf"
SOURCE_CRIMPS = "https://dynalloy.com/crimp-styles/"


@dataclass(frozen=True)
class ActuatorConfig:
    """Nominal geometry plus conservative source-guided design assumptions."""

    fiber_count: int = 12
    fiber_diameter_mm: float = 0.20
    active_length_mm: float = 140.0
    design_strain: float = 0.035
    series_per_string: int = 4
    wire_resistance_ohm_per_m: float = 29.0
    heating_pull_grams_per_fiber: float = 570.0
    cooling_deformation_grams_per_fiber: float = 228.0
    current_for_1s_contraction_A_per_fiber: float = 0.660
    passive_cooling_time_s: float = 3.2
    nominal_activation_temperature_C: float = 70.0
    spring_preload_N: float = 24.0
    spring_rate_N_per_mm: float = 0.55
    fiber_segments: int = 12
    fiber_radius_mm: float = 10.5
    guide_radius_mm: float = 18.0
    guide_rod_radius_mm: float = 1.5
    frame_radius_mm: float = 24.0


DEFAULT_CONFIG = ActuatorConfig()

MATERIALS = [
    Material("Black anodized frame", (0.035, 0.040, 0.050), 0.78, 0.31),
    Material("Machined aluminium", (0.56, 0.59, 0.63), 0.94, 0.23),
    Material("Stainless guide rod", (0.56, 0.59, 0.61), 0.96, 0.20),
    Material("Nitinol actuator wire", (0.47, 0.50, 0.53), 0.96, 0.21),
    Material("Copper electrical jumper", (0.72, 0.31, 0.10), 0.93, 0.24),
    Material("Spring steel", (0.34, 0.36, 0.39), 0.96, 0.27),
    Material("PEEK / ceramic insulator", (0.74, 0.69, 0.51), 0.03, 0.52),
    Material("Nickel-plated crimp", (0.62, 0.65, 0.68), 0.94, 0.18),
    Material("Polymer linear bushing", (0.07, 0.075, 0.085), 0.02, 0.48),
]


def _validate_config(c: ActuatorConfig) -> None:
    if c.fiber_count <= 0 or c.series_per_string <= 0:
        raise ValueError("fiber_count and series_per_string must be positive")
    if c.fiber_count % c.series_per_string:
        raise ValueError("fiber_count must be divisible by series_per_string")
    if c.series_per_string % 2:
        raise ValueError("series_per_string must be even so both supply terminals can remain fixed")
    if not 0.0 < c.design_strain <= 0.06:
        raise ValueError("design_strain must be in (0, 0.06]")
    if c.active_length_mm <= 0 or c.fiber_diameter_mm <= 0:
        raise ValueError("active_length_mm and fiber_diameter_mm must be positive")
    if c.fiber_segments < 4:
        raise ValueError("fiber_segments must be at least 4 for a useful deformation proxy")
    m = design_metrics(c)
    if m["spring_force_hot_end_N"] > m["bundle_cooling_deformation_force_N"]:
        raise ValueError(
            "return spring exceeds the source-guided cooling deformation force at full stroke"
        )


def design_metrics(c: ActuatorConfig = DEFAULT_CONFIG) -> dict[str, float | int | str]:
    strings = c.fiber_count // c.series_per_string
    wire_R = c.wire_resistance_ohm_per_m * c.active_length_mm / 1000.0
    string_R = wire_R * c.series_per_string
    string_V = string_R * c.current_for_1s_contraction_A_per_fiber
    total_I = c.current_for_1s_contraction_A_per_fiber * strings
    power = string_V * total_I
    stroke = c.active_length_mm * c.design_strain
    heat_force = c.heating_pull_grams_per_fiber / 1000.0 * G0 * c.fiber_count
    cool_force = c.cooling_deformation_grams_per_fiber / 1000.0 * G0 * c.fiber_count
    spring_hot = c.spring_preload_N + c.spring_rate_N_per_mm * stroke
    return {
        "stroke_mm": stroke,
        "wire_resistance_ohm": wire_R,
        "series_string_count": strings,
        "series_per_string": c.series_per_string,
        "string_resistance_ohm": string_R,
        "nominal_string_voltage_V": string_V,
        "nominal_total_current_A": total_I,
        "nominal_electrical_power_W": power,
        "bundle_heating_pull_force_N": heat_force,
        "bundle_cooling_deformation_force_N": cool_force,
        "spring_force_cold_N": c.spring_preload_N,
        "spring_force_hot_end_N": spring_hot,
        "estimated_net_hot_pull_at_full_stroke_N": heat_force - spring_hot,
        "passive_cooling_time_s": c.passive_cooling_time_s,
        "activation_reference": f"{c.nominal_activation_temperature_C:.0f} C LT wire guide",
    }


def activation_fraction(t: float, c: ActuatorConfig = DEFAULT_CONFIG) -> float:
    """A deterministic demonstration cycle, not a coupled thermal solution.

    One second source-guided heating ramp -> short hold -> source-guided passive
    cooling interval -> cold dwell. Smoothstep avoids discontinuous velocity in
    generated demonstration videos.
    """
    heat_s = 1.0
    hold_s = 0.45
    cool_s = c.passive_cooling_time_s
    dwell_s = 0.85
    period = heat_s + hold_s + cool_s + dwell_s
    q = float(t) % period

    def smooth01(x: float) -> float:
        x = min(1.0, max(0.0, x))
        return x * x * (3.0 - 2.0 * x)

    if q < heat_s:
        return smooth01(q / heat_s)
    if q < heat_s + hold_s:
        return 1.0
    q -= heat_s + hold_s
    if q < cool_s:
        return 1.0 - smooth01(q / cool_s)
    return 0.0


def pose(part, t: float, explode: float, c: ActuatorConfig = DEFAULT_CONFIG) -> np.ndarray:
    """Rigid-part proxy for wire/spring deformation and carriage translation."""
    a = activation_fraction(t, c)
    stroke = c.active_length_mm * c.design_strain
    dx = 0.0
    if part.motion == "carriage":
        dx = -stroke * a
    elif part.motion == "fiber":
        # Every short wire element remains rigid, but its center follows the
        # homogeneous axial contraction map. Adjacent elements overlap by a
        # fraction of a segment when hot, visually approximating a shorter wire
        # without violating Mechanism Lab's rigid-transform contract.
        x0 = 12.0
        u = min(1.0, max(0.0, (float(part.center[0]) - x0) / c.active_length_mm))
        dx = -stroke * a * u
    elif part.motion == "spring":
        spring_x0, spring_x1 = 24.0, 148.0
        u = min(1.0, max(0.0, (float(part.center[0]) - spring_x0) / (spring_x1 - spring_x0)))
        dx = -stroke * a * u

    T = np.eye(4)
    T[:3, 3] = np.asarray(part.explode, float) * float(explode)
    T[0, 3] += dx
    return T


def _straight_cylinder_part(
    name: str,
    xa: float,
    xb: float,
    y: float,
    z: float,
    radius: float,
    material: int,
    *,
    group: str,
    motion: str = "fixed",
    center=None,
    explode=(0.0, 0.0, 0.0),
    role: str = "",
    provenance: str = "designed-concept",
    sides: int = 10,
):
    """Fast open-ended cylinder along X; ends are intentionally buried in hardware."""
    ang = np.arange(sides, dtype=float) * (2.0 * math.pi / sides)
    yz = np.column_stack([np.cos(ang), np.sin(ang)]) * radius
    v0 = np.column_stack([np.full(sides, xa), y + yz[:, 0], z + yz[:, 1]])
    v1 = np.column_stack([np.full(sides, xb), y + yz[:, 0], z + yz[:, 1]])
    vertices = np.vstack([v0, v1])
    normals = np.vstack(
        [np.column_stack([np.zeros(sides), yz[:, 0] / radius, yz[:, 1] / radius])] * 2
    )
    faces = []
    for j in range(sides):
        k = (j + 1) % sides
        faces.extend([[j, k, sides + k], [j, sides + k, sides + j]])
    if center is None:
        center = ((xa + xb) * 0.5, y, z)
    return Part(
        name,
        vertices,
        np.asarray(faces, dtype=np.int64),
        normals,
        material,
        group=group,
        motion=motion,
        center=np.asarray(center, float),
        explode=np.asarray(explode, float),
        role=role,
        provenance=provenance,
    )


def _torus_part(
    name: str,
    x: float,
    major: float,
    minor: float,
    material: int,
    *,
    group: str,
    motion: str,
    center,
    explode=(0.0, 0.0, 0.0),
    role: str = "",
    major_segments: int = 28,
    minor_segments: int = 7,
):
    """Direct torus mesh around the X axis; avoids CAD overhead for spring visualization."""
    verts = []
    norms = []
    faces = []
    for i in range(major_segments):
        a = 2 * math.pi * i / major_segments
        ca, sa = math.cos(a), math.sin(a)
        for j in range(minor_segments):
            b = 2 * math.pi * j / minor_segments
            cb, sb = math.cos(b), math.sin(b)
            rr = major + minor * cb
            verts.append((x + minor * sb, rr * ca, rr * sa))
            norms.append((sb, cb * ca, cb * sa))
    for i in range(major_segments):
        ni = (i + 1) % major_segments
        for j in range(minor_segments):
            nj = (j + 1) % minor_segments
            a = i * minor_segments + j
            b = ni * minor_segments + j
            cc = ni * minor_segments + nj
            d = i * minor_segments + nj
            faces.extend([[a, b, cc], [a, cc, d]])
    return Part(
        name,
        np.asarray(verts, float),
        np.asarray(faces, dtype=np.int64),
        np.asarray(norms, float),
        material,
        group=group,
        motion=motion,
        center=np.asarray(center, float),
        explode=np.asarray(explode, float),
        role=role,
    )


def _mesh_tube_part(
    name: str,
    points,
    radius: float,
    material: int,
    *,
    group: str,
    motion: str = "fixed",
    center=None,
    explode=(0.0, 0.0, 0.0),
    role: str = "",
    provenance: str = "designed-concept",
    sides: int = 10,
):
    v, f = tube_mesh(points, radius, sides)
    if center is None:
        center = np.mean(np.asarray(points, float), axis=0)
    return mesh_part(
        name,
        v,
        f,
        material,
        group=group,
        motion=motion,
        center=np.asarray(center, float),
        explode=np.asarray(explode, float),
        role=role,
        provenance=provenance,
    )


def _fiber_points(c: ActuatorConfig) -> list[tuple[float, float]]:
    return bolt_circle(c.fiber_radius_mm, c.fiber_count, phase=math.pi / c.fiber_count)


def _guide_points(c: ActuatorConfig) -> list[tuple[float, float]]:
    return bolt_circle(c.guide_radius_mm, 4, phase=math.pi / 4)


def build(c: ActuatorConfig = DEFAULT_CONFIG) -> Assembly:
    _validate_config(c)
    metrics = design_metrics(c)
    parts = []

    def add(
        name,
        shape,
        material,
        group,
        *,
        motion="fixed",
        center=None,
        explode=(0, 0, 0),
        role="",
        provenance="designed-concept",
    ):
        if center is None:
            center = np.zeros(3)
        p = cad_part(
            name,
            shape,
            material,
            group=group,
            motion=motion,
            center=np.asarray(center, float),
            explode=np.asarray(explode, float),
            role=role,
            provenance=provenance,
        )
        parts.append(p)
        return p

    fibers = _fiber_points(c)
    guides = _guide_points(c)

    rear = ring(c.frame_radius_mm, 4.3, 0, 6)
    rear = drill(rear, guides, c.guide_rod_radius_mm + 0.18, -0.5, 6.5)
    add(
        "SMA01_Rear_frame_plate",
        rear,
        0,
        "structure",
        explode=(-36, 0, 0),
        role="Fixed rear reaction plate",
    )

    front = ring(c.frame_radius_mm, 3.25, 182, 188)
    front = drill(front, guides, c.guide_rod_radius_mm + 0.18, 181.5, 188.5)
    add(
        "SMA02_Front_frame_plate",
        front,
        0,
        "structure",
        explode=(38, 0, 0),
        role="Front guide/output bearing plate",
    )

    for i, (y, z) in enumerate(guides):
        parts.append(
            _straight_cylinder_part(
                f"SMA03_Guide_rod_{i+1:02}",
                6,
                182,
                y,
                z,
                c.guide_rod_radius_mm,
                2,
                group="guides",
                explode=(0, 11 * math.cos(i * math.pi / 2), 11 * math.sin(i * math.pi / 2)),
                role="Fixed precision guide rod; bearing fit not tolerance-qualified",
                sides=16,
            )
        )

    fixed_anchor = ring(14.2, 4.2, 8, 12)
    fixed_anchor = drill(fixed_anchor, fibers, 0.34, 7.5, 12.5)
    add(
        "SMA04_Fixed_insulating_anchor",
        fixed_anchor,
        6,
        "anchors",
        explode=(-22, 0, 0),
        role="Electrically isolated rear fiber/crimp carrier",
    )

    moving_anchor = ring(14.2, 4.2, 152, 156)
    moving_anchor = drill(moving_anchor, fibers, 0.34, 151.5, 156.5)
    add(
        "SMA05_Moving_insulating_anchor",
        moving_anchor,
        6,
        "carriage",
        motion="carriage",
        center=(154, 0, 0),
        explode=(20, 0, 0),
        role="Moving fiber/crimp carrier bonded to the output carriage",
    )

    carriage = ring(21.2, 4.1, 156, 162)
    carriage = drill(carriage, guides, c.guide_rod_radius_mm + 0.32, 155.5, 162.5)
    add(
        "SMA06_Output_carriage",
        carriage,
        1,
        "carriage",
        motion="carriage",
        center=(159, 0, 0),
        explode=(26, 0, 0),
        role="Rigid translating output carriage",
    )

    for i, (y, z) in enumerate(guides):
        add(
            f"SMA07_Carriage_bushing_{i+1:02}",
            ring(2.8, c.guide_rod_radius_mm + 0.08, 155.5, 162.5).translate((0, y, z)),
            8,
            "carriage",
            motion="carriage",
            center=(159, y, z),
            explode=(26, 5 * math.cos(i * math.pi / 2), 5 * math.sin(i * math.pi / 2)),
            role="Representative low-friction guide bushing; no tolerance/tribology claim",
        )

    add(
        "SMA08_Output_rod",
        ring(3.0, 0, 162, 202),
        2,
        "carriage",
        motion="carriage",
        center=(182, 0, 0),
        explode=(36, 0, 0),
        role="Linear output rod",
    )
    add(
        "SMA09_Output_eye",
        ring(5.3, 2.3, 202, 208),
        1,
        "carriage",
        motion="carriage",
        center=(205, 0, 0),
        explode=(40, 0, 0),
        role="Generic output eye; user interface dimensions remain conceptual",
    )

    add(
        "SMA10_Spring_fixed_seat",
        ring(7.5, 2.2, 20, 23),
        1,
        "spring",
        role="Return-spring fixed seat",
    )
    add(
        "SMA11_Spring_moving_seat",
        ring(7.5, 2.2, 148, 152),
        1,
        "carriage",
        motion="carriage",
        center=(150, 0, 0),
        role="Return-spring moving seat",
    )
    add(
        "SMA12_Spring_pilot",
        ring(1.25, 0, 23, 149),
        2,
        "spring",
        role="Return-spring pilot rod",
    )

    spring_turns = 16
    for i in range(spring_turns):
        u = i / (spring_turns - 1)
        x = 24.0 + u * (148.0 - 24.0)
        parts.append(
            _torus_part(
                f"SMA13_Return_spring_turn_{i+1:02}",
                x,
                6.0,
                0.47,
                5,
                group="spring",
                motion="spring",
                center=(x, 0, 0),
                explode=(0, 0, -8),
                role=(
                    "Visual discretization of the return spring; preload/rate come from metadata, "
                    "not beam/contact simulation"
                ),
            )
        )

    # The active wire is divided into rigid sub-elements so the generic renderer
    # can show distributed contraction while preserving rigid transform validity.
    x0 = 12.0
    seg_len = c.active_length_mm / c.fiber_segments
    wire_r = c.fiber_diameter_mm / 2.0
    for fi, (y, z) in enumerate(fibers):
        for si in range(c.fiber_segments):
            xa = x0 + si * seg_len
            xb = x0 + (si + 1) * seg_len
            parts.append(
                _straight_cylinder_part(
                    f"SMA14_Fiber_{fi+1:02}_segment_{si+1:02}",
                    xa,
                    xb,
                    y,
                    z,
                    wire_r,
                    3,
                    group="fibers",
                    motion="fiber",
                    center=((xa + xb) / 2.0, y, z),
                    explode=(
                        0,
                        5 * math.cos(2 * math.pi * fi / c.fiber_count),
                        5 * math.sin(2 * math.pi * fi / c.fiber_count),
                    ),
                    role=(
                        f"Active Nitinol fiber {fi+1}; {c.fiber_diameter_mm:.2f} mm nominal wire, "
                        f"{c.design_strain*100:.1f}% modeled contraction"
                    ),
                    provenance="source-guided-concept",
                    sides=8,
                )
            )

        parts.append(
            _straight_cylinder_part(
                f"SMA15_Fixed_crimp_{fi+1:02}",
                9.5,
                12.2,
                y,
                z,
                0.62,
                7,
                group="electrical",
                explode=(-13, 0, 0),
                role=(
                    "Representative mechanical/electrical crimp barrel; exact commercial crimp "
                    "not reproduced"
                ),
                provenance="source-guided-concept",
                sides=12,
            )
        )
        parts.append(
            _straight_cylinder_part(
                f"SMA16_Moving_crimp_{fi+1:02}",
                151.8,
                154.5,
                y,
                z,
                0.62,
                7,
                group="carriage",
                motion="carriage",
                center=(153.15, y, z),
                explode=(14, 0, 0),
                role="Representative moving crimp barrel",
                provenance="source-guided-concept",
                sides=12,
            )
        )

    # Three electrically parallel strings, four fibers in series per string.
    # Because the series count is even, both main supply terminals remain on the
    # fixed end: F1 -> moving jumper -> F2 -> fixed jumper -> F3 -> moving jumper
    # -> F4. This avoids a high-current flexible lead on the translating carriage.
    strings = c.fiber_count // c.series_per_string
    for s in range(strings):
        idx = [s * c.series_per_string + j for j in range(c.series_per_string)]
        for j in range(c.series_per_string - 1):
            ia, ib = idx[j], idx[j + 1]
            ya, za = fibers[ia]
            yb, zb = fibers[ib]
            if j % 2 == 0:
                pts = [
                    (153.6, ya, za),
                    (156.0, (ya + yb) / 2, (za + zb) / 2),
                    (153.6, yb, zb),
                ]
                motion = "carriage"
                center = (155.0, (ya + yb) / 2, (za + zb) / 2)
                ex = (14, 0, 0)
            else:
                pts = [
                    (10.0, ya, za),
                    (7.2, (ya + yb) / 2, (za + zb) / 2),
                    (10.0, yb, zb),
                ]
                motion = "fixed"
                center = (8.0, (ya + yb) / 2, (za + zb) / 2)
                ex = (-14, 0, 0)
            parts.append(
                _mesh_tube_part(
                    f"SMA17_String_{s+1}_jumper_{j+1}",
                    pts,
                    0.42,
                    4,
                    group="electrical" if motion == "fixed" else "carriage",
                    motion=motion,
                    center=center,
                    explode=ex,
                    role="Copper series jumper for the four-fiber electrical string",
                )
            )

    # Fixed rear positive/negative bus leads. Each feeds the first/last fiber of
    # one four-fiber string. Current limiting remains an external driver duty.
    pos_terminal = (-4.0, 8.0, 0.0)
    neg_terminal = (-4.0, -8.0, 0.0)
    add(
        "SMA18_Positive_terminal",
        ring(1.6, 0.7, -6, 1).translate((0, 8, 0)),
        4,
        "electrical",
        explode=(-20, 0, 0),
        role="Fixed current-driver positive terminal",
    )
    add(
        "SMA19_Negative_terminal",
        ring(1.6, 0.7, -6, 1).translate((0, -8, 0)),
        4,
        "electrical",
        explode=(-20, 0, 0),
        role="Fixed current-driver negative terminal",
    )

    for s in range(strings):
        first = s * c.series_per_string
        last = first + c.series_per_string - 1
        y0, z0 = fibers[first]
        y1, z1 = fibers[last]
        parts.append(
            _mesh_tube_part(
                f"SMA20_Positive_bus_branch_{s+1}",
                [pos_terminal, (3, 8, 0), (7, y0, z0), (10, y0, z0)],
                0.48,
                4,
                group="electrical",
                explode=(-12, 0, 0),
                role="Positive bus branch feeding a four-fiber series string",
            )
        )
        parts.append(
            _mesh_tube_part(
                f"SMA21_Negative_bus_branch_{s+1}",
                [neg_terminal, (3, -8, 0), (7, y1, z1), (10, y1, z1)],
                0.48,
                4,
                group="electrical",
                explode=(-12, 0, 0),
                role="Negative bus branch returning a four-fiber series string",
            )
        )

    # Cold and hot hard stops provide a real geometric stroke limit independent
    # of controller assumptions. The hot stop sits one modeled stroke leftward.
    stroke = float(metrics["stroke_mm"])
    for i, (y, z) in enumerate(guides):
        add(
            f"SMA22_Cold_stop_{i+1:02}",
            ring(2.9, c.guide_rod_radius_mm + 0.08, 163.0, 164.6).translate((0, y, z)),
            1,
            "stops",
            role="Cold-position travel stop / service spacer",
        )
        add(
            f"SMA23_Hot_stop_{i+1:02}",
            ring(2.9, c.guide_rod_radius_mm + 0.08, 154.0 - stroke, 155.4 - stroke).translate((0, y, z)),
            1,
            "stops",
            role="Independent geometric stop near modeled full-contraction position",
        )

    views = {
        "hero": View(
            az=40,
            el=24,
            scale=118,
            target=(102, 0, 0),
            title="NITINOL FIBER ACTUATOR / 12 x 0.20 mm",
            note="140 mm active bundle / 3.5% modeled strain / return-spring carriage",
        ),
        "fiber_bundle": View(
            az=45,
            el=18,
            scale=86,
            target=(82, 0, 0),
            hide=("structure", "guides"),
            title="ACTIVE FIBER BUNDLE",
            note="12 mechanically parallel wires; 3 electrical strings x 4 wires in series",
        ),
        "electrical": View(
            az=150,
            el=18,
            scale=62,
            target=(18, 0, 0),
            title="FIXED-END POWER ROUTING",
            note="Both high-current terminals stay fixed; moving-end jumpers ride with the carriage",
        ),
        "section": View(
            az=42,
            el=12,
            scale=105,
            target=(102, -4, 0),
            section=(0, 1, 0),
            title="LONGITUDINAL SECTION",
            note="Central return spring resets the bundle during cooling",
        ),
        "exploded": View(
            az=58,
            el=22,
            scale=145,
            target=(100, 0, 0),
            explode=1,
            title="ACTUATOR / EXPLODED INSPECTION",
            note="Explode offsets are inspection-only and not an assembly procedure",
        ),
    }

    metadata = {
        "design": asdict(c),
        "metrics": metrics,
        "sources": [SOURCE_WIRE_DATA, SOURCE_TECH_SHEET, SOURCE_CRIMPS],
        "fidelity": (
            "Parametric actuator concept using published 0.20 mm SMA wire guide values; "
            "frame, spring, crimps, bushings and electrical routing are original conceptual geometry"
        ),
        "electrical_topology": {
            "mechanical_parallel_fibers": c.fiber_count,
            "electrical_parallel_strings": c.fiber_count // c.series_per_string,
            "series_fibers_per_string": c.series_per_string,
            "terminals": "both fixed on rear plate",
            "driver": "external current-limited low-voltage PWM/current controller required",
        },
        "motion_model": (
            "Prescribed thermally inspired cycle. Carriage motion is rigid; fibers and spring are segmented "
            "rigid geometry following an axial contraction map. No coupled thermal/phase/FEA solution."
        ),
        "assumptions": [
            "70 C class actuator wire reference values from Dynalloy 0.008 in / 0.20 mm table",
            f"{c.design_strain*100:.1f}% working strain selected within the vendor's broader 2-6% motion range",
            "uniform current sharing within the three nominally identical series strings",
            "return spring preload/rate are design targets, not derived from a modeled commercial spring",
            "crimp barrels and bus routing are representative, not vendor CAD",
            "heat sinking, ambient airflow, contact resistance, wire temperature, phase hysteresis and fatigue are not solved",
        ],
        "safety": [
            "Do not drive raw wire from an uncontrolled voltage source; use current limiting and over-temperature protection.",
            "Hot SMA and electrical joints can burn skin and ignite nearby heat-sensitive material.",
            "Guard the moving carriage and preloaded spring before physical testing.",
            "Re-characterize force, stroke, current, temperature and cycle life for the exact wire lot and crimp process before load-bearing use.",
        ],
        "drawing_groups": ["structure", "anchors", "carriage", "guides", "stops"],
        "drawings": {
            "default": {
                "parts": [
                    "SMA01_Rear_frame_plate",
                    "SMA02_Front_frame_plate",
                    "SMA04_Fixed_insulating_anchor",
                    "SMA05_Moving_insulating_anchor",
                    "SMA06_Output_carriage",
                    "SMA08_Output_rod",
                ],
                "title": "NITINOL FIBER ACTUATOR / NOMINAL INTERFACE STUDY",
                "auto_dimensions": True,
                "annotations": [
                    {
                        "kind": "note",
                        "paper_xy": [18, 28],
                        "text": "UNITS: mm / CONCEPT GEOMETRY",
                        "size": 2.6,
                    },
                    {
                        "kind": "note",
                        "paper_xy": [18, 34],
                        "text": f"ACTIVE WIRE: {c.active_length_mm:.0f} / STROKE: {metrics['stroke_mm']:.2f}",
                        "size": 2.5,
                    },
                    {
                        "kind": "note",
                        "paper_xy": [18, 40],
                        "text": f"BUNDLE: {c.fiber_count} x DIA {c.fiber_diameter_mm:.2f} NiTi",
                        "size": 2.5,
                    },
                ],
            }
        },
    }

    return Assembly(
        "nitinol_fiber_actuator",
        parts,
        MATERIALS,
        views,
        metadata=metadata,
        motion_function=lambda p, t, e: pose(p, t, e, c),
    )
