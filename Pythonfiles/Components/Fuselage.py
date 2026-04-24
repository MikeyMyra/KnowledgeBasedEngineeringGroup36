import os

from parapy.core import Input,Attribute,Part, child, action
from parapy.geom import GeomBase, Cylinder

'''Wheels kunnen wel met basic sizing denk ik ipv inputs, ik bedoel ik denk dat niemand boeit hoe je wielen eruit zien'''

class Fuselage(GeomBase):
    #Inputs fuselage
    radius: float = Input()
    length: float = Input()
    material: str = Input()
    #Inputs Undercarriage
    retractible: bool = Input()

    @Part
    def geometry(self):
        return Cylinder(self.radius, self.length)

    @Attribute
    def calculate_mass(self):
        temp = 1
        return temp

    @Attribute
    def calculate_volume(self):
        temp = 1
        return temp

    @Attribute
    def calculate_skin_friction(self):
        temp = 1
        return temp