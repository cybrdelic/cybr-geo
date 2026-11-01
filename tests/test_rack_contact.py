import numpy as np
import pytest
from mechanism_lab.rack_contact import convex_profile_parts,rectilinear_solid_boxes

def test_notch_is_not_filled_by_convex_contact():
 pieces,error=convex_profile_parts([(0,0),(3,0),(3,1),(1,1),(1,3),(0,3)])
 assert error<1e-10 and len(pieces)>1
 assert all(abs(p.convex_hull.area-p.area)<1e-10 for p in pieces)

def test_native_pad_cutout_preserved():
 cq=pytest.importorskip('cadquery')
 shape=cq.Solid.makeBox(3,3,2).cut(cq.Solid.makeBox(2,2,2,cq.Vector(1,1,0)))
 boxes,error=rectilinear_solid_boxes(shape)
 assert error<1e-10 and sum(np.prod(hi-lo) for lo,hi in boxes)==10
 with pytest.raises(ValueError):rectilinear_solid_boxes(cq.Solid.makeCylinder(2,3))
