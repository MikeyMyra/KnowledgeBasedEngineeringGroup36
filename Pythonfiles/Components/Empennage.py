import os

from parapy.core import Input,Attribute,Part, child, action
from parapy.geom import GeomBase




class Empennage(GeomBase):
    # Inputs
    material: str = Input()

    @Attribute
    def estimate_horizontal_tail_volume(self) -> float:
        temp = 1
        return temp

    @Attribute
    def estimate_vertical_tail_volume(self) -> float:
        temp = 1
        return temp

    @Attribute
    def estimate_mass(self) -> float:
        temp = 1
        return temp
