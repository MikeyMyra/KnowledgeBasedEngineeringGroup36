import os

from parapy.core import Input,Attribute,Part, child, action
from parapy.geom import GeomBase


class Wing(GeomBase):
    # Inputs
    span: float = Input()
    wing_area: float = Input()
    taper_ratio: float = Input()
    leading_edge_position: float = Input()
    material_wing: str = Input()
    material_wingbox: str = Input()
    thickness_to_chord: float = Input()
    maximum_chamber: float = Input()
    maximum_chamber_position: float = Input()
    front_spar_position: float = Input()
    rear_spar_position: float = Input()

    @Attribute
    def calculate_root_chord(self) -> float:
        temp = 1
        return temp

    @Attribute
    def calculate_tip_chord(self) -> float:
        temp = 1
        return temp

    @Attribute
    def calculate_sweep_angle(self) -> float:
        temp = 1
        return temp

    @Attribute
    def calculate_MAC(self) -> float:
        temp = 1
        return temp

    @Attribute
    def calculate_MAC_position(self) -> float:
        temp = 1
        return temp

    @Attribute
    def calculate_aspect_ratio(self) -> float:
        return self.wing_area / self.span**2

    @Attribute
    def calculate_sectional_properties(self) -> float:
        temp = 1
        return temp

    @Attribute
    def calculate_mass(self) -> float:
        temp = 1
        return temp

    @Part
    def geometry(self):
        temp = 1
        return temp
