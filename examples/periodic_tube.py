"""One continuous nonplanar closed tube; not disconnected cylinder segments."""
import numpy as np
from cybrgeo import sweep


def build():
    t = np.linspace(0, 2*np.pi, 96, endpoint=False)
    p = np.linspace(0, 2*np.pi, 16, endpoint=False)
    path = np.c_[3*np.cos(t), 3*np.sin(t), .4*np.sin(3*t)]
    section = .16*np.c_[np.cos(p), np.sin(p)]
    mesh = sweep(section, path, closed=True)
    assert mesh.closed
    return mesh

if __name__ == '__main__':
    print(build().report())
