import math

from parapy.core import Input, Part
from parapy.geom import GeomBase, Cylinder, translate, rotate


class WeaponSolid(GeomBase):
    """Single weapon cylinder, placed at a pre-computed (dy, dz) offset."""

    diameter:   float = Input()
    height_cyl: float = Input()
    dy:         float = Input()
    dz:         float = Input()
    color:      str   = Input("Firebrick")

    @Part(parse=False)
    def solid(self):
        return Cylinder(
            radius=self.diameter / 2.0,
            height=self.height_cyl,
            centered=True,
            color=self.color,
            position=rotate(
                translate(self.position, "y", self.dy, "z", self.dz),
                "y", math.pi / 2,
            ),
        )
