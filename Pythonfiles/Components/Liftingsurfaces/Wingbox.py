import sys
import os
from math import radians, tan, sin
import numpy as np

from parapy.core import Input, Attribute, Part
from parapy.geom import (
    GeomBase, LoftedSolid,
    translate, rotate, Point,
    Polygon
)

if __name__ != "__main__":
    from Liftingsurfaces.Airfoil import Airfoil


class Wingbox(GeomBase):
    """Wingbox fitted inside airfoil using real local thickness."""

    # ------------------------------------------------------------------ #
    # INPUTS — PLANFORM
    # ------------------------------------------------------------------ #

    c_root: float = Input()
    c_tip: float = Input()
    semi_span: float = Input()
    sweep_le: float = Input(25.0)
    dihedral: float = Input(5.0)
    twist: float = Input(-2.0)

    # ------------------------------------------------------------------ #
    # INPUTS — SECTION
    # ------------------------------------------------------------------ #

    front_spar_position: float = Input(0.15)
    rear_spar_position: float = Input(0.60)

    # airfoil objects
    airfoil_root: "Airfoil" = Input()
    airfoil_tip: "Airfoil" = Input()

    # ------------------------------------------------------------------ #
    # INPUTS — STRUCTURAL
    # ------------------------------------------------------------------ #

    material: str = Input("aluminium")
    mesh_deflection: float = Input(1e-4)

    # ------------------------------------------------------------------ #
    # WIDTH (UNCHANGED)
    # ------------------------------------------------------------------ #

    @Attribute
    def width_root(self):
        return (self.rear_spar_position - self.front_spar_position) * self.c_root

    @Attribute
    def width_tip(self):
        return (self.rear_spar_position - self.front_spar_position) * self.c_tip

    # ------------------------------------------------------------------ #
    # SPAR POSITIONS
    # ------------------------------------------------------------------ #

    @Attribute
    def front_spar_x_root(self):
        return self.front_spar_position * self.c_root

    @Attribute
    def front_spar_x_tip(self):
        return self.front_spar_position * self.c_tip

    # ------------------------------------------------------------------ #
    # CORNERS
    # ------------------------------------------------------------------ #

    @Attribute
    def _root_corners(self):
        x_front = self.front_spar_position
        x_rear = self.rear_spar_position

        zf_u, zf_l = self.airfoil_root.surface_z_at(x_front)
        zr_u, zr_l = self.airfoil_root.surface_z_at(x_rear)

        xf = x_front * self.c_root
        xr = x_rear * self.c_root

        pos = self.airfoil_root.position.location

        return [
            Point(pos.x + xf, pos.y, pos.z + zf_l),
            Point(pos.x + xr, pos.y, pos.z + zr_l),
            Point(pos.x + xr, pos.y, pos.z + zr_u),
            Point(pos.x + xf, pos.y, pos.z + zf_u),
        ]

    @Attribute
    def _tip_corners(self):
        x_front = self.front_spar_position
        x_rear = self.rear_spar_position

        zf_u, zf_l = self.airfoil_tip.surface_z_at(x_front)
        zr_u, zr_l = self.airfoil_tip.surface_z_at(x_rear)

        xf = x_front * self.c_tip
        xr = x_rear * self.c_tip

        # 1. Base spanwise position ONLY (NO dihedral here)
        base_pos = rotate(
            translate(
                self.airfoil_root.position,
                "x", abs(self.semi_span) * tan(radians(abs(self.sweep_le))),
                "y", self.semi_span,
                "z", abs(self.semi_span) * sin(radians(abs(self.dihedral))),
            ), "y", radians(self.twist)
        )

        p0 = base_pos.location

        # 2. Local section (airfoil box in its own frame)
        corners = [
            Point(xf, 0, zf_l),
            Point(xr, 0, zr_l),
            Point(xr, 0, zr_u),
            Point(xf, 0, zf_u),
        ]

        # 3. Apply twist first (local rotation)
        twisted = [
            rotate(corner, "y", radians(self.twist))
            for corner in corners
        ]

        # 4. Apply dihedral as global rotation around root X-axis
        dihedraled = [
            rotate(c, "x", radians(self.dihedral))
            for c in twisted
        ]

        # 5. Translate into position
        return [
            Point(p0.x + c.x, p0.y, p0.z + c.z)
            for c in dihedraled
        ]

    # ------------------------------------------------------------------ #
    # SECTIONS
    # ------------------------------------------------------------------ #

    @Part
    def root_section(self):
        return Polygon(
            points=self._root_corners,
            color="SteelBlue",
            mesh_deflection=self.mesh_deflection,
        )

    @Part
    def tip_section(self):
        return Polygon(
            points=self._tip_corners,
            color="SteelBlue",
            mesh_deflection=self.mesh_deflection,
        )

    # ------------------------------------------------------------------ #
    # SOLID
    # ------------------------------------------------------------------ #

    @Part
    def solid(self):
        return LoftedSolid(
            profiles=[self.root_section, self.tip_section],
            color="SteelBlue",
            mesh_deflection=self.mesh_deflection,
        )


# ---------------------------------------------------------------------- #
# TEST
# ---------------------------------------------------------------------- #

if __name__ == "__main__":
    from parapy.gui import display
    
    if __name__ == "__main__":
        from Airfoil import Airfoil

    root_af = Airfoil(
        chord=5.0,
        airfoil_name="simm_airfoil"
    )

    tip_af = Airfoil(
        chord=2.0,
        airfoil_name="simm_airfoil"
    )

    wb = Wingbox(
        c_root=5.0,
        c_tip=2.0,
        semi_span=15.0,
        airfoil_root=root_af,
        airfoil_tip=tip_af,
    )
    display(wb)