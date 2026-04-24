import os

from parapy.core import Input,Attribute,Part, child, action
from parapy.geom import GeomBase

from Propeller_components.Piston import Piston
from Propeller_components.Turboshaft import Turboshaft



class Propeller(GeomBase):
    #inputs
    required_power: float = Input()
    required_TO_thrust: float = Input()
    engine_type: str = Input()

    @Attribute
    def estimate_disk_diameter(self) -> float:
        temp = 1
        return temp