import os

from parapy.core import Input,Attribute,Part, child, action
from parapy.geom import GeomBase



class Undercarriage(GeomBase):
    #Inputs
    retractible: bool = Input()
    aircraft_mass: float = Input()

    def calculate_wheel_size(self):
        temp = 1
        return temp

    def calculate_amount_of_struts(self):
        temp = 1
        return temp

    def calculate_mass(self):
        temp = 1
        return temp

    def calculate_volume(self):
        temp = 1
        return temp