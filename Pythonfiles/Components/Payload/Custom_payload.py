import os

from parapy.core import Input,Attribute,Part, child, action
from parapy.geom import GeomBase, Box, Cylinder



class Custom_payload(GeomBase):
    # Inputs
    geometry_type = Input("box", validator=validate.OneOf(['box', 'cylinder']))
    length: float = Input()
    width: float = Input()
    height_box: float = Input()
    diameter: float = Input()
    height_cyl: float = Input()

    @child
    def geometry(self):
        if self.geometry_type == 'box':
            return Box(self.length, self.width, self.height_box)
        elif self.geometry_type == 'cylinder':
            return Cylinder(self.diameter / 2, self.height_cyl)
        else:
            raise ValueError(f"Invalid geometry type: {self.geometry_type}")