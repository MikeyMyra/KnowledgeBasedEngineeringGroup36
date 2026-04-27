import os

from parapy.core import Input,Attribute,Part, child, action
from parapy.geom import GeomBase



class Wingbox(GeomBase):
    # Inputs
    front_spar_position: float = Input()
    rear_spar_position: float = Input()
    material: float = Input()

    @Part
    def geometry(self):
        temp = 1
        return temp

