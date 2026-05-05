import os

from parapy.core import Input,Attribute,Part, child, action
from parapy.geom import GeomBase



class Jet_engine(GeomBase):
    # inputs
    required_power: float = Input()
    required_TO_thrust: float = Input()
    engine_type: str = Input()