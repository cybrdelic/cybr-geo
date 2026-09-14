"""Exact simple-planetary geometry and rigid-body kinematics.

Generated external profiles use tooth centers at their authored phase. Generated
internal-ring profiles use *space* centers at their authored phase. The phase
constraints below are derived against those actual conventions so a zero residual
means complementary tooth/space geometry, not merely a self-consistent ratio.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import numpy as np

TAU=2.0*math.pi

def _wrap_pi(value:float)->float:return (value+math.pi)%TAU-math.pi

@dataclass(frozen=True)
class PlanetarySpec:
    sun_teeth:int=18
    planet_teeth:int=27
    ring_teeth:int=72
    planets:int=3
    module:float=.8
    pressure_angle_deg:float=20.
    backlash_mm:float=.10
    face_width_mm:float=10.
    input_speed_rps:float=.45
    def validate(self)->None:
        if min(self.sun_teeth,self.planet_teeth,self.ring_teeth)<8:raise ValueError('all gear tooth counts must be >= 8')
        if self.ring_teeth!=self.sun_teeth+2*self.planet_teeth:raise ValueError('simple planetary requires ring_teeth = sun_teeth + 2*planet_teeth')
        if self.planets<2:raise ValueError('at least two planets required')
        if (self.sun_teeth+self.ring_teeth)%self.planets:raise ValueError('(sun_teeth + ring_teeth) must be divisible by planet count for equal spacing')
        if self.module<=0 or self.face_width_mm<=0:raise ValueError('module and face width must be positive')
        if not 10<=self.pressure_angle_deg<=35:raise ValueError('pressure angle out of supported range')
        if self.backlash_mm<0 or self.backlash_mm>=math.pi*self.module/2:raise ValueError('invalid circumferential backlash')
        if self.input_speed_rps<=0:raise ValueError('input speed must be positive')
        if self.pressure_angle_deg<=20.5 and self.sun_teeth<17:raise ValueError('sun tooth count enters standard unshifted undercut region')
    @property
    def sun_pitch_radius(self):return self.sun_teeth*self.module/2
    @property
    def planet_pitch_radius(self):return self.planet_teeth*self.module/2
    @property
    def ring_pitch_radius(self):return self.ring_teeth*self.module/2
    @property
    def planet_center_radius(self):return self.sun_pitch_radius+self.planet_pitch_radius
    @property
    def reduction(self):return 1+self.ring_teeth/self.sun_teeth
    @property
    def carrier_speed_rps(self):return self.input_speed_rps/self.reduction
    @property
    def planet_absolute_speed_rps(self):return self.carrier_speed_rps-(self.sun_teeth/self.planet_teeth)*(self.input_speed_rps-self.carrier_speed_rps)

def external_mesh_constant(spec:PlanetarySpec)->float:
    """Phase constant imposed by the opposite-side contact on an external planet.

    An external profile's tooth center is phase coordinate 0 and its space center
    is pi. At the sun/planet contact the planet is observed on its opposite side,
    adding Np*pi. The complementary condition therefore leaves (Np-1)*pi.
    """
    return ((spec.planet_teeth-1)*math.pi)%TAU

def ring_initial_phase(spec:PlanetarySpec)->float:
    """Internal ring space-center phase required by the chosen sun phase.

    Odd-tooth planets need zero ring offset. Even-tooth planets put the opposite
    planet side in the same tooth/space parity, so the ring is shifted half a ring
    pitch; this keeps the package general rather than accidentally relying on 27T.
    """
    return external_mesh_constant(spec)/spec.ring_teeth

def planet_base_angles(spec:PlanetarySpec)->tuple[float,...]:
    spec.validate();return tuple(TAU*i/spec.planets for i in range(spec.planets))

def planet_centers(spec:PlanetarySpec,carrier_angle:float=0)->tuple[tuple[float,float],...]:
    r=spec.planet_center_radius
    return tuple((r*math.cos(a+carrier_angle),r*math.sin(a+carrier_angle)) for a in planet_base_angles(spec))

def planet_initial_phases(spec:PlanetarySpec)->tuple[float,...]:
    """Total external-planet tooth phases satisfying the authored profile geometry."""
    spec.validate();c=external_mesh_constant(spec)
    return tuple(((spec.sun_teeth+spec.planet_teeth)*a+c)/spec.planet_teeth for a in planet_base_angles(spec))

def planetary_angles(spec:PlanetarySpec,time_seconds:float)->dict:
    """Exact total tooth/space phases for sun, fixed ring, carrier and planets."""
    spec.validate();theta_s=TAU*spec.input_speed_rps*float(time_seconds);theta_c=theta_s/spec.reduction
    phases=planet_initial_phases(spec);base=planet_base_angles(spec);planets=[];centers=[]
    for a0,p0 in zip(base,phases):
        alpha=a0+theta_c
        theta_p=p0+theta_c-(spec.sun_teeth/spec.planet_teeth)*(theta_s-theta_c)
        centers.append(alpha);planets.append(theta_p)
    return {'sun':theta_s,'carrier':theta_c,'ring':ring_initial_phase(spec),'planet_centers':tuple(centers),'planets':tuple(planets)}

def mesh_residuals(spec:PlanetarySpec,time_seconds:float)->dict:
    """Wrapped residuals against actual generated tooth/space phase conventions."""
    q=planetary_angles(spec,time_seconds);c=external_mesh_constant(spec);ext=[];internal=[]
    for alpha,theta_p in zip(q['planet_centers'],q['planets']):
        # External/external mesh: tooth-center coordinate + opposite planet-side
        # coordinate must be complementary by one half-pitch.
        e=spec.sun_teeth*q['sun']+spec.planet_teeth*theta_p-(spec.sun_teeth+spec.planet_teeth)*alpha-c
        # The generated internal ring profile is referenced by SPACE centers, so
        # equal normalized phase means external planet tooth sits in internal space
        # (or external space receives internal tooth) at the contact line.
        r=spec.ring_teeth*q['ring']-spec.planet_teeth*theta_p-(spec.ring_teeth-spec.planet_teeth)*alpha
        ext.append(_wrap_pi(e));internal.append(_wrap_pi(r))
    return {'sun_planet':tuple(ext),'ring_planet':tuple(internal)}

def rotation_x4(angle:float)->np.ndarray:
    c,s=math.cos(angle),math.sin(angle);T=np.eye(4);T[:3,:3]=np.array([[1,0,0],[0,c,-s],[0,s,c]],float);return T

def orbit_and_spin(center_yz:tuple[float,float],carrier_angle:float,relative_spin:float)->np.ndarray:
    y,z=center_yz;to=np.eye(4);to[:3,3]=[0,y,z];fr=np.eye(4);fr[:3,3]=[0,-y,-z]
    return rotation_x4(carrier_angle)@to@rotation_x4(relative_spin)@fr
