"""CYBR VISOR M1 design inputs, millimetres unless specified.

Phone body dimensions are Samsung's published nominal envelope. Camera
clearance, cushion fit, print tolerances and the lens prescription are design
inputs, not measurements of the user's handset, face, printer or purchased lens.
"""
from dataclasses import asdict, dataclass
import math


@dataclass(frozen=True)
class Spec:
    phone_width: float = 162.8       # landscape body width
    phone_height: float = 77.6
    phone_thickness: float = 8.2
    phone_mass_g: float = 218.0
    camera_allowance: float = 4.0   # conservative design allowance; measure
    screen_width: float = 159.1     # derived rectangular display envelope
    screen_height: float = 73.4
    screen_y: float = 43.5
    center_z: float = 65.0
    ipd: float = 64.0
    ipd_min: float = 58.0
    ipd_max: float = 72.0
    focus_offset: float = 0.0       # discrete plate spacers, positive toward screen
    lens_diameter: float = 34.0
    lens_thickness: float = 8.5
    lens_ior: float = 1.49
    lens_efl: float = 45.0
    eye_relief: float = 15.0        # pupil to eye-side vertex, design target
    radial_clearance: float = 0.30
    print_error: float = 0.15       # signed bound per surface, unmeasured

    def __post_init__(self):
        values = asdict(self)
        if not all(math.isfinite(v) for v in values.values()):
            raise ValueError("All design inputs must be finite")
        if not self.ipd_min <= self.ipd <= self.ipd_max:
            raise ValueError("IPD must be in the mechanical 58–72 mm travel")
        if not -1.0 <= self.focus_offset <= 1.0:
            raise ValueError("Focus uses -1/0/+1 mm plate-spacer positions")
        if min(self.phone_width, self.phone_height, self.phone_thickness,
               self.lens_diameter, self.lens_thickness, self.lens_efl) <= 0:
            raise ValueError("Dimensions must be positive")
        if self.lens_ior <= 1 or self.lens_thickness >= self.lens_ior*self.lens_efl:
            raise ValueError("Invalid biconvex prescription")
        if self.lens_diameter/2 >= self.lens_radius:
            raise ValueError("Lens aperture exceeds sphere")
        if self.lens_edge_thickness <= 0:
            raise ValueError("Lens surfaces intersect inside the aperture")

    @property
    def lens_radius(self):
        # Symmetric thick lensmaker equation in air, solved for R.
        a = self.lens_ior-1
        return a*self.lens_efl*(1+math.sqrt(
            1-self.lens_thickness/(self.lens_ior*self.lens_efl)))

    @property
    def lens_edge_thickness(self):
        r = self.lens_diameter/2
        return self.lens_thickness-2*(self.lens_radius-math.sqrt(
            self.lens_radius**2-r*r))

    @property
    def principal_offset(self):
        # Distance inward from either vertex to its principal plane.
        return self.lens_efl*(self.lens_ior-1)*self.lens_thickness/(
            self.lens_ior*self.lens_radius)

    @property
    def object_distance(self):
        screen_vertex = self.focus_offset+self.lens_thickness/2
        return self.screen_y-screen_vertex+self.principal_offset

    def report(self):
        s = self.object_distance
        virtual = self.lens_efl*s/(self.lens_efl-s)
        return {
            "inputs": asdict(self),
            "lens_radius_mm": self.lens_radius,
            "lens_edge_thickness_mm": self.lens_edge_thickness,
            "principal_plane_from_vertex_mm": self.principal_offset,
            "object_distance_from_principal_plane_mm": s,
            "paraxial_virtual_image_distance_mm": virtual,
            "tracking": "phone orientation / 3DoF; no positional tracking",
            "optical_status": "specified symmetric PMMA lens; not vendor-qualified",
            "physical_status": "unbuilt; print, lens and on-device calibration required",
        }
