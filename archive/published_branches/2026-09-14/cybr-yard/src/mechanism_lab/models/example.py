"""A second independent recipe exercises the generic pipeline without motor code."""
import numpy as np
from ..core import Assembly,View,Material,cad_part
from ..geometry import ring,bolt_circle,drill

def build():
    body=drill(ring(35,12,0,8),bolt_circle(27,6),3.3,-1,9)
    hub=ring(19,12,8,20)
    parts=[cad_part('F01_Six_hole_flange',body,0,role='User-designed flange'),
           cad_part('F02_Stepped_hub',hub,0,explode=np.array([25.,0,0]),role='Separate locating hub')]
    return Assembly('example_flange',parts,[Material('Aluminium',(.55,.57,.59),.95,.27)],
        {'hero':View(45,25,48,(8,0,0),title='NEW RECIPE / SIX-HOLE FLANGE'),
         'exploded':View(65,25,55,(15,0,0),explode=1,title='SIX-HOLE FLANGE / EXPLODED')},
        metadata={'drawing_groups':['structure'],'dimensions_nominal':True})
