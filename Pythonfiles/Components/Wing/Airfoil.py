import os

from parapy.core import Input,Attribute,Part, child, action, Base
from parapy.geom import GeomBase



class Airfoil(Base):
    # Inputs
    thickness_to_chord: float = Input()
    maximum_chamber: float = Input()
    maximum_chamber_position: float = Input()

    @Attribute
    def calculate_CST_coefficients(self):
        temp = 1
        return temp