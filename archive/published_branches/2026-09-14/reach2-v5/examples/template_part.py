"""A self-contained new-model plugin. No renderer/animation rewrite required."""
import cadquery as cq
from cybrgeo import Assembly,Material,from_shape

def build() -> Assembly:
    # Millimetres. Any CadQuery geometry can replace this one function.
    plate=(cq.Workplane('XY').box(96,64,8).edges('|Z').fillet(5)
           .faces('>Z').workplane().pushPoints([(-36,-20),(-36,20),(36,-20),(36,20)]).hole(6.6)
           .faces('>Z').workplane().circle(16).cutThruAll()).val()
    part=from_shape('mount_plate',plate,role='Parametric four-hole mounting plate / design study')
    return Assembly('TEMPLATE MOUNT',[part],[Material()],cad={part.name:plate})
