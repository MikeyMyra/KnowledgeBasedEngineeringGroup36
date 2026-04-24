import os

from parapy.core import Input,Attribute,Part, child, action
from parapy.geom import GeomBase


class Engine(GeomBase):
    # Inputs
    required_power: float = Input()
    required_TO_thrust: float = Input()
    engine_type: str = Input()

    @Attribute
    def compute_N_engines(self) -> float:
        temp = 1
        return temp

