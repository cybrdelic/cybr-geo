"""Trusted local recipe example. Only geometry and annotations are model-specific.

Run: lab build examples/custom_flange.py --step
     lab render examples/custom_flange.py
     lab blueprint examples/custom_flange.py
"""
import numpy as np
from mechanism_lab import Assembly, Material, View
from mechanism_lab.core import cad_part
from mechanism_lab.geometry import ring, drill, bolt_circle


def build():
    body = drill(ring(42, 14, 0, 10), bolt_circle(33, 6), 3.3, -1, 11)
    hub = ring(23, 14, 10, 27)
    parts = [cad_part('Six_hole_plate', body, role='Custom nominal design'),
             cad_part('Locating_hub', hub, explode=np.array([35.,0,0]))]
    return Assembly('custom_flange', parts,
                    [Material('Brushed aluminium', (.5,.53,.57), .95, .28)],
                    {'hero': View(45,25,60,(12,0,0),title='CUSTOM RECIPE / FLANGE'),
                     'exploded': View(65,25,70,(27,0,0),explode=1,title='CUSTOM FLANGE / EXPLODED')},
                    metadata={'drawings': {'default': {'annotations': [
                        {'kind':'circle','radius':33},
                        {'kind':'leader','point':[33,0],'elbow_paper_mm':[12,-13],
                         'end_paper_mm':[22,-13],'text':'6 x DIA 6.6 / BCD 66'}
                    ]}}})
