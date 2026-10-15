"""Scientific state plots; these are diagnostic charts, never terrain renders."""
from pathlib import Path
import numpy as np


def plot_state(state,path):
    import os,tempfile
    os.environ.setdefault('MPLCONFIGDIR',str(Path(tempfile.gettempdir())/'cybr-terrain-matplotlib'))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    s=state;e=s.config.extent/2
    extent=(-e,e,-e,e)
    figure,axes=plt.subplots(2,3,figsize=(13,8),constrained_layout=True)
    fields=[(s.height,'Surviving ground elevation [m]','terrain',None,None),
            (s.height-s.initial_height,'Bed change [m]','RdBu_r',None,None),
            (s.water,'Surface water depth [m]','Blues',0,None),
            (s.saturation,'Shallow soil-bucket saturation','YlGnBu',0,1),
            (s.loose.sum(axis=0)/(1-.42),'Deposited loose layer [m]','magma',0,None),
            (s.sediment[2]/np.maximum(s.water,1e-8),'Suspended fines volume fraction','copper',0,.05)]
    for ax,(field,title,cmap,lo,hi) in zip(axes.ravel(),fields):
        if cmap=='RdBu_r':hi=max(float(np.max(np.abs(field))),.001);lo=-hi
        im=ax.imshow(field,origin='lower',extent=extent,cmap=cmap,vmin=lo,vmax=hi)
        ax.set_title(title,fontsize=10);ax.set_xlabel('x [m]');ax.set_ylabel('y [m]');figure.colorbar(im,ax=ax,shrink=.72)
    figure.suptitle(f'CYBR TERRAIN / {s.config.preset} / {s.time:.1f} s / morph. factor {s.config.morphological_factor:g}',fontsize=14)
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True);figure.savefig(path,dpi=150);plt.close(figure)
    return path
