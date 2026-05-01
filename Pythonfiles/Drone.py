import os

from parapy.core import Input,Attribute,Part, child, action, validate
from parapy.geom import GeomBase
from Pythonfiles.Components.Liftingsurface import Wing
from Components.Payload import Payload
from Components.Fuselage import Fuselage
from Components.Engine import Engine
from Components.Avionics import Avionics
from Components.Empennage import Empennage

class Drone(GeomBase):
    #-------------------Inputs-------------------
    # Engine inputs
    required_power: float  = Input()
    required_TO_thrust: float = Input()
    engine_type: str = Input()
    # Payload inputs
    # Comms inputs
    geometry_type_comms = Input("box", validator=validate.OneOf(['box', 'cylinder']))
    length_comms: float = Input()
    width_comms: float = Input()
    height_box_comms: float = Input()
    diameter_comms: float = Input()
    height_cyl_comms: float = Input()
    # Custom payload inputs
    geometry_type_cust = Input("box", validator=validate.OneOf(['box', 'cylinder']))
    length_cust: float = Input()
    width_cust: float = Input()
    height_box_cust: float = Input()
    diameter_cust: float = Input()
    height_cyl_cust: float = Input()
    # Flight Computer inputs
    geometry_type_flcom = Input("box", validator=validate.OneOf(['box', 'cylinder']))
    length_flcom: float = Input()
    width_flcom: float = Input()
    height_box_flcom: float = Input()
    diameter_flcom: float = Input()
    height_cyl_flcom: float = Input()
    # Radar inputs
    geometry_type_radar = Input("cylinder", validator=validate.OneOf(['box', 'cylinder']))
    length_radar: float = Input()
    width_radar: float = Input()
    height_box_radar: float = Input()
    diameter_radar: float = Input()
    height_cyl_radar: float = Input()
    # Weaponry inputs
    geometry_type_weapon = Input("box", validator=validate.OneOf(['box', 'cylinder']))
    # position? gun in wing or in somewhere else for example
    length_weapon: float = Input()
    width_weapon: float = Input()
    height_box_weapon: float = Input()
    diameter_weapon: float = Input()
    height_cyl_weapon: float = Input()
    #Fuselage inputs
    radius_fuse: float = Input()
    length_fuse : float = Input()
    material_fuse : str = Input()
    retractible: bool = Input()
    #Empennage inputs
    material_emp: str = Input()
    #Wing inputs
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
    #Mission inputs
    mission_range: float = Input()
    mission_endurance: float = Input()
    cruise_speed: float = Input()
    maximum_load_factor = Input()
    cruise_altitude: float = Input()

    @Attribute
    def mission(self):
        'Ik ga wel beginnen met deze functie'
        temp = 1
        return temp














