import os

from parapy.core import Input,Attribute,Part, child, action
from parapy.geom import GeomBase



class Turboshaft(GeomBase):
    # Inputs
    required_power: float = Input()
    N_engines: float = Input()

    @Attribute
    def estimate_diameter(self) -> float:
        temp = 1
        return temp

    @Attribute
    def estimate_length(self) -> float:
        temp = 1
        return temp

    @Attribute
    def estimate_clearance_height(self) -> float:
        temp = 1
        return temp

    @Attribute
    def estimate_clearance_width(self) -> float:
        temp = 1
        return temp

    @Attribute
    def estimate_clearance_length(self) -> float:
        temp = 1
        return temp

