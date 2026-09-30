"""SI terrain state. Grain quantities are solid-equivalent depth in metres.

Layer thickness is bulk depth. Explicit porosity converts it to solid volume;
this avoids pretending sand and a porous deposited bed have equal bulk volume.
Initial geology is an authored depositional/uplift history, subsequently eroded.
"""
from __future__ import annotations

from dataclasses import dataclass, asdict
from pathlib import Path
import json
import tempfile
import zipfile
import numpy as np
from scipy.ndimage import gaussian_filter, zoom

GRAINS = ("gravel", "sand", "fines")
LOOSE_POROSITY = 0.42


@dataclass(frozen=True)
class Layer:
    name: str
    color: tuple[float, float, float]
    fractions: tuple[float, float, float]
    porosity: float
    erodibility: float  # m / (Pa s); illustrative, uncalibrated
    critical_shear: float  # Pa
    conductivity: float  # m / s
    repose_degrees: float
    roughness: float
    grain_scale: float  # m, procedural surface texture scale

    def __post_init__(self):
        values = (*self.color, *self.fractions, self.porosity, self.erodibility,
                  self.critical_shear, self.conductivity, self.repose_degrees,
                  self.roughness, self.grain_scale)
        if not np.isfinite(values).all():
            raise ValueError("Non-finite layer property")
        if abs(sum(self.fractions)-1)>1e-10 or min(self.fractions)<0:
            raise ValueError("Grain fractions must be nonnegative and sum to one")
        if not 0<=self.porosity<0.8 or not 0<self.roughness<=1:
            raise ValueError("Invalid porosity or roughness")
        if min(self.erodibility,self.critical_shear,self.conductivity)<0 or self.grain_scale<=0:
            raise ValueError("Invalid layer rate or grain scale")
        if not 0<self.repose_degrees<90 or min(self.color)<0 or max(self.color)>1:
            raise ValueError("Invalid layer angle or linear albedo")


LAYERS = {
    "basalt": Layer("Basalt basement",(.085,.092,.10),(.78,.18,.04),.12,1e-8,32,2e-7,76,.88,.09),
    "shale": Layer("Fine laminated shale",(.19,.145,.105),(.03,.15,.82),.22,3e-6,6,3e-7,66,.87,.025),
    "ochre": Layer("Iron-rich sandstone",(.44,.20,.072),(.06,.82,.12),.28,1.3e-5,2.8,4e-6,64,.80,.035),
    "chalk": Layer("Pale carbonate silt",(.59,.48,.30),(.05,.36,.59),.32,2.3e-5,1.3,1.5e-6,58,.90,.018),
    "clay": Layer("Clay-rich B horizon",(.28,.115,.046),(.02,.22,.76),.40,1.1e-5,3.8,5e-7,48,.85,.016),
    "sand": Layer("Sandy parent material",(.49,.34,.17),(.07,.85,.08),.39,4e-5,.8,5e-5,34,.84,.014),
    "gravel": Layer("Gravel alluvium",(.23,.21,.17),(.72,.23,.05),.35,2.4e-5,9,8e-5,39,.76,.13),
    "soil": Layer("Mineral topsoil A horizon",(.15,.078,.030),(.10,.48,.42),.46,4.5e-5,1.5,1.8e-5,38,.96,.012),
    "humus": Layer("Organic surface horizon",(.058,.031,.014),(.03,.32,.65),.60,5e-5,.7,3e-5,36,.98,.009),
}


@dataclass(frozen=True)
class Config:
    preset: str = "badlands"
    grid: int = 129
    extent: float = 44.0
    seed: int = 20260930
    duration: float = 180.0
    rain_mm_hour: float = 100.0
    inlet_m3_second: float = 0.25
    max_dt: float = 0.35
    cfl: float = 0.35
    manning: float = 0.048
    morphological_factor: float = 40.0
    thermal_rate: float = 0.08
    evaporation_mm_hour: float = 0.4
    boundary: str = "open"
    max_steps: int = 200000

    def __post_init__(self):
        if self.preset not in PRESETS:raise ValueError("Unknown terrain preset")
        if isinstance(self.grid,bool) or not isinstance(self.grid,int) or not 17<=self.grid<=513:
            raise ValueError("Grid must be an integer from 17 to 513")
        if isinstance(self.seed,bool) or not isinstance(self.seed,int) or self.seed<0:
            raise ValueError("Seed must be a nonnegative integer")
        if isinstance(self.max_steps,bool) or not isinstance(self.max_steps,int) or self.max_steps<1:
            raise ValueError("Step budget must be a positive integer")
        rates=(self.extent,self.duration,self.rain_mm_hour,self.inlet_m3_second,
               self.max_dt,self.cfl,self.manning,self.morphological_factor,
               self.thermal_rate,self.evaporation_mm_hour)
        if not np.isfinite(rates).all() or min(rates)<0:
            raise ValueError("Terrain settings must be finite and nonnegative")
        if min(self.extent,self.max_dt,self.manning,self.morphological_factor)<=0:
            raise ValueError("Extent, timestep, roughness and morphological factor must be positive")
        if not 0<self.cfl<=0.45 or self.boundary not in ("open","closed"):
            raise ValueError("Invalid stability or boundary setting")

    @property
    def dx(self):return self.extent/(self.grid-1)


PRESETS = {
    "badlands": {"extent":44.0,"duration":220.0,"inlet_m3_second":.32,"rain_mm_hour":110,
                 "morphological_factor":55,"description":"Tilted sandstone, shale and carbonate beds; runoff-carved gullies and sorted sediment."},
    "watershed": {"extent":36.0,"duration":180.0,"inlet_m3_second":.22,"rain_mm_hour":95,
                  "morphological_factor":35,"description":"Soil-covered uplands, a gravel channel and wet alluvial deposition."},
    "soil-profile": {"extent":10.0,"duration":90.0,"inlet_m3_second":.007,"rain_mm_hour":75,
                     "morphological_factor":18,"description":"An organic surface, mineral A horizon, clay B horizon and sandy parent material."},
}


def field_noise(shape, rng, octaves=5):
    """Seeded smooth geological variation; no external heightmaps or scans."""
    out=np.zeros(shape)
    for octave in range(octaves):
        cells=3*2**octave
        coarse=rng.normal(size=(cells,cells))
        f=zoom(coarse,(shape[0]/cells,shape[1]/cells),order=3)[:shape[0],:shape[1]]
        f-=f.mean();f/=max(f.std(),1e-10)
        out+=f*.5**octave
    return out/max(out.std(),1e-10)


def _polyline_distance(x, y, points):
    """Distance to a continuous channel, independent of the simulation grid."""
    distance=np.full(x.shape,np.inf)
    for a,b in zip(points[:-1],points[1:]):
        delta=b-a
        t=np.clip(((x-a[0])*delta[0]+(y-a[1])*delta[1])/max(float(delta@delta),1e-12),0,1)
        distance=np.minimum(distance,np.hypot(x-a[0]-t*delta[0],y-a[1]-t*delta[1]))
    return distance


def _drainage(x, y, rng):
    """Authored branching drainage and ridge relief, followed by actual runoff.

    Channels are continuous polylines rather than a quantized pixel mask. Their
    incision is an initial landscape history; it is not claimed as a result of
    the short storm simulated below. Normalized coordinates span a square site.
    """
    phase=rng.uniform(-.5,.5)
    def center(t):return .11*np.sin(3.4*t+phase)+.035*np.sin(8.7*t+1.2+phase)
    main=np.abs(x-center(y))
    tributary=np.zeros_like(x)
    rills=np.zeros_like(x)
    for side in (-1,1):
        for junction in (-.72,-.22,.29,.72):
            junction+=rng.uniform(-.06,.06)
            reach=rng.uniform(.69,.96)
            climb=rng.uniform(.28,.51)
            u=np.linspace(0,1,42)
            points=np.column_stack((center(junction)+side*reach*u,
                                    junction+climb*u+.027*np.sin(8*u)*u))
            distance=_polyline_distance(x,y,points)
            width=rng.uniform(.022,.034)
            depth=rng.uniform(.72,1.02)
            # A tributary fades gradually uphill instead of ending in a crater.
            fade=np.clip(1.1-np.abs(x-center(junction))/reach,.05,1)
            tributary=np.maximum(tributary,depth*fade*np.exp(-(distance/width)**1.30))
            for fraction in (.43,.74):
                join=points[int(fraction*(len(points)-1))]
                v=np.linspace(0,1,24)
                child=np.column_stack((join[0]+side*.21*v,
                                       join[1]+.22*v+.012*np.sin(9*v)))
                child_distance=_polyline_distance(x,y,child)
                rills=np.maximum(rills,.23*np.exp(-(child_distance/.012)**1.25))
    return center(y),main,tributary,rills


def _clipped_stack(foundation, nominal, cap):
    """Clip each depositional interface to an erosional surface, bottom up."""
    base=foundation.copy();interface=foundation.copy();thickness=[]
    for bed in nominal:
        interface+=bed
        top=np.maximum(base,np.minimum(interface,cap))
        thickness.append(top-base);base=top
    return np.asarray(thickness)


@dataclass
class State:
    config: Config
    layers: tuple[Layer,...]
    foundation: np.ndarray
    thickness: np.ndarray
    loose: np.ndarray
    sediment: np.ndarray
    water: np.ndarray
    soil_water: np.ndarray
    qx: np.ndarray
    qy: np.ndarray
    rain_pattern: np.ndarray
    inlet_pattern: np.ndarray
    initial_height: np.ndarray
    time: float = 0
    steps: int = 0

    @property
    def height(self):
        return self.foundation+self.thickness.sum(axis=0)+self.loose.sum(axis=0)/(1-LOOSE_POROSITY)

    @property
    def exposed(self):
        active=self.thickness>1e-9
        return np.max(np.where(active,np.arange(len(self.layers))[:,None,None],-1),axis=0)

    @property
    def pore_capacity(self):
        p=np.array([l.porosity for l in self.layers])[:,None,None]
        # A single shallow soil bucket, rather than an invented groundwater PDE.
        return np.minimum(.7,(self.thickness*p).sum(axis=0)+self.loose.sum(axis=0)*LOOSE_POROSITY/(1-LOOSE_POROSITY))

    @property
    def saturation(self):return np.clip(self.soil_water/np.maximum(self.pore_capacity,1e-12),0,1)

    def solid_volumes(self):
        total=(self.loose+self.sediment).sum(axis=(1,2))
        for k,layer in enumerate(self.layers):
            total+=np.asarray(layer.fractions)*(1-layer.porosity)*self.thickness[k].sum()
        return total*self.config.dx**2

    def water_volume(self):return float((self.water+self.soil_water).sum()*self.config.dx**2)

    def validate(self):
        n=self.config.grid
        expected={"foundation":(n,n),"thickness":(len(self.layers),n,n),"loose":(3,n,n),
                  "sediment":(3,n,n),"water":(n,n),"soil_water":(n,n),"qx":(n,n-1),
                  "qy":(n-1,n),"rain_pattern":(n,n),"inlet_pattern":(n,n),"initial_height":(n,n)}
        for key,shape in expected.items():
            a=getattr(self,key)
            if a.shape!=shape or not np.isfinite(a).all():raise ValueError("Invalid field: "+key)
            if key in ("thickness","loose","sediment","water","soil_water","rain_pattern","inlet_pattern") and a.min()<-1e-11:
                raise ValueError("Negative conserved field: "+key)
        if np.max(self.soil_water-self.pore_capacity)>1e-8:raise ValueError("Pore bucket exceeded capacity")
        return True

    def save(self,path):
        self.validate();path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
        keys=("foundation","thickness","loose","sediment","water","soil_water","qx","qy",
              "rain_pattern","inlet_pattern","initial_height")
        meta={"schema":1,"units":"SI metres/seconds; grains are solid-equivalent depth",
              "config":asdict(self.config),"layers":[asdict(l) for l in self.layers],"time":self.time,"steps":self.steps}
        # A streaming ZIP on a synchronized output mount can expose an archive
        # before its central directory is written. Close and CRC-check it in a
        # private temporary directory, then publish a single complete file.
        from cybr_light.runtime import publish
        with tempfile.TemporaryDirectory(prefix='cybr-terrain-state-') as temporary:
            staged=Path(temporary)/'state.npz'
            np.savez_compressed(staged,metadata=json.dumps(meta),**{k:getattr(self,k) for k in keys})
            with zipfile.ZipFile(staged) as archive:
                broken=archive.testzip()
                if broken:raise ValueError('Terrain archive CRC failed: '+broken)
            publish(staged,path)
        return path

    @classmethod
    def load(cls,path):
        try:
            with np.load(path,allow_pickle=False) as data:
                meta=json.loads(str(data["metadata"]))
                if meta.get("schema")!=1:raise ValueError("Unsupported terrain state schema")
                state=cls(Config(**meta["config"]),tuple(Layer(**l) for l in meta["layers"]),
                          **{key:np.array(data[key],dtype=float) for key in data.files if key!="metadata"},
                          time=meta["time"],steps=meta["steps"])
            state.validate();return state
        except (OSError,ValueError,KeyError,TypeError,zipfile.BadZipFile) as exc:
            raise ValueError(f'Cannot load terrain state {path}: {exc}') from exc


def make_terrain(config: Config):
    n=config.grid;rng=np.random.default_rng(config.seed)
    x,y=np.meshgrid(np.linspace(-1,1,n),np.linspace(-1,1,n))
    noise=field_noise((n,n),rng)
    channel,distance,tributary,rills=_drainage(x,y,rng)
    # Relief has both broad hills and short ridges; its amplitude remains SI.
    ridge_noise=field_noise((n,n),rng,octaves=6)
    ridges=1.0-np.abs(ridge_noise)/max(float(np.percentile(np.abs(ridge_noise),95)),1e-9)
    ridges=np.clip(ridges,-.4,1)
    if config.preset=="badlands":
        # Gently tilted beds are cut by an authored pre-existing drainage tree.
        # All rendered layer exposures subsequently come from this same stack.
        foundation=.8+2.2*(y+1)+.18*x+.10*gaussian_filter(noise,max(n/24,1))
        keys=("basalt","shale","ochre","chalk","shale","ochre","chalk","soil")
        depths=(.45,.50,1.05,.42,.34,1.15,.36,.08)
        nominal=[]
        for k,depth in enumerate(depths):
            nominal.append(depth*(1+.055*np.sin(2.1*x+1.8*y+k*.8)))
        width=.039+.022*(1-y)/2
        incision=3.70*np.exp(-(distance/width)**1.25)
        incision=np.maximum(incision,2.95*tributary)+.55*rills
        cap=foundation+3.91+.25*ridges+.13*noise-incision
        cap=np.maximum(foundation+.28,cap)
        thickness=_clipped_stack(foundation,nominal,cap)
    elif config.preset=="watershed":
        foundation=.7+1.75*(y+1)+1.35*(x-channel)**2+.20*noise
        keys=("basalt","sand","gravel","clay","soil","humus")
        depths=(.25,.65,.25,.30,.30,.045)
        nominal=[depth*(1+.10*np.sin(2.6*x+1.6*y+k)) for k,depth in enumerate(depths)]
        incision=.85*np.exp(-(distance/.058)**1.4)
        incision=np.maximum(incision,.45*tributary)+.11*rills
        cap=foundation+np.asarray(nominal).sum(axis=0)-incision+.05*ridges
        thickness=_clipped_stack(foundation,nominal,cap)
    else:
        foundation=.25+.30*(y+1)+.07*noise
        keys=("basalt","sand","clay","soil","humus")
        depths=(.15,.65,.42,.32,.060)
        nominal=[depth*(1+.08*np.sin(2*x+3*y+k)+.025*noise) for k,depth in enumerate(depths)]
        # An interior erosional bank exposes the same soil horizons as the
        # saved stack. It is part of the initial landscape, not a display wall.
        bank_center=.045+.065*np.sin(2.7*x)+.016*np.sin(8.1*x)
        bank_distance=np.abs(y-bank_center)
        bank_width=.078+.009*np.sin(5*x)
        incision=1.18*np.exp(-(bank_distance/bank_width)**2.6)
        incision+=.08*np.maximum(ridge_noise,0)*np.exp(-(bank_distance/.18)**2)
        cap=foundation+np.asarray(nominal).sum(axis=0)-incision
        cap=np.maximum(foundation+.22,cap)
        thickness=_clipped_stack(foundation,nominal,cap)
    layers=tuple(LAYERS[k] for k in keys);thickness=np.asarray(thickness)
    rain=np.clip(1+.30*gaussian_filter(noise,n/12),.25,2);rain/=rain.mean()
    inlet=np.exp(-((x-channel)/.09)**2)*(y>.94);inlet/=inlet.sum()
    z=np.zeros((n,n));height=foundation+thickness.sum(axis=0)
    state=State(config,layers,foundation,thickness,np.zeros((3,n,n)),np.zeros((3,n,n)),z.copy(),z.copy(),
                np.zeros((n,n-1)),np.zeros((n-1,n)),rain,inlet,height.copy())
    state.validate();return state
