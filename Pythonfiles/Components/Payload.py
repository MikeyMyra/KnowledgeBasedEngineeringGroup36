import os

from parapy.core import Input,Attribute,Part, child, action, validate
from parapy.geom import GeomBase

'''voor nu is het niet mogelijk om inputs the hiden, dus als we voor elke de mogelijkheid willen geven welke shape het
is, moeten we alle inputs wel geven'''


class Payload(GeomBase):
    # Inputs Comms
    geometry_type_comms = Input("box", validator=validate.OneOf(['box', 'cylinder']))
    length_comms: float = Input()
    width_comms: float = Input()
    height_box_comms: float = Input()
    diameter_comms: float = Input()
    height_cyl_comms: float = Input()
    # Inputs Custom payload
    geometry_type_cust = Input("box", validator=validate.OneOf(['box', 'cylinder']))
    length_cust: float = Input()
    width_cust: float = Input()
    height_box_cust: float = Input()
    diameter_cust: float = Input()
    height_cyl_cust: float = Input()
    # Inputs Flight Computer
    geometry_type_flcom = Input("box", validator=validate.OneOf(['box', 'cylinder']))
    length_flcom: float = Input()
    width_flcom: float = Input()
    height_box_flcom: float = Input()
    diameter_flcom: float = Input()
    height_cyl_flcom: float = Input()
    # Inputs Radar
    geometry_type_radar = Input("cylinder", validator=validate.OneOf(['box', 'cylinder']))
    length_radar: float = Input()
    width_radar: float = Input()
    height_box_radar: float = Input()
    diameter_radar: float = Input()
    height_cyl_radar: float = Input()
    # Inputs Weaponry
    geometry_type_weapon= Input("box", validator=validate.OneOf(['box', 'cylinder']))
    #position? gun in wing or in somewhere else for example
    length_weapon: float = Input()
    width_weapon: float = Input()
    height_box_weapon: float = Input()
    diameter_weapon: float = Input()
    height_cyl_weapon: float = Input()

    #calculate volume & mass
    @Attribute
    def compute_volume(self) -> float:
        temp = 1
        return temp

    @Attribute
    def compute_mass(self) -> float:
        temp = 1
        return temp


