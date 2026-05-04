import sys
import os
import numpy as np

from parapy.geom import GeomBase, FittedCurve, Point
from parapy.core import Input, Attribute, Part

from Pythonfiles.Components.Frame import Frame


class Airfoil(GeomBase):
    """Airfoil curve with thickness evaluation capability."""

    # ------------------------------------------------------------------ #
    # INPUTS
    # ------------------------------------------------------------------ #

    chord: float = Input(1.0)
    airfoil_name: str = Input("simm_airfoil")
    thickness_factor: float = Input(1.0)
    mesh_deflection: float = Input(1e-4)

    # ------------------------------------------------------------------ #
    # RAW POINTS (3D GEOMETRY)
    # ------------------------------------------------------------------ #

    @Attribute
    def raw_points(self):
        base_dir = os.path.dirname(__file__)
        path = os.path.join(base_dir, "Airfoils", self.airfoil_name + ".dat")

        pts = []
        with open(path, "r") as f:
            for line in f:
                x, z = line.split()
                pts.append(
                    self.position.translate(
                        "x", float(x) * self.chord,
                        "z", float(z) * self.chord * self.thickness_factor,
                    )
                )
        return pts

    # ------------------------------------------------------------------ #
    # NORMALIZED COORDINATES (FOR ANALYSIS)
    # ------------------------------------------------------------------ #

    @Attribute
    def normalized_coordinates(self):
        """Return (x, z) in normalized chord coordinates."""
        base_dir = os.path.dirname(__file__)
        path = os.path.join(base_dir, "Airfoils", self.airfoil_name + ".dat")

        coords = []
        with open(path, "r") as f:
            for line in f:
                x, z = line.split()
                coords.append((float(x), float(z)))

        return np.array(coords)

    # ------------------------------------------------------------------ #
    # SPLIT UPPER / LOWER
    # ------------------------------------------------------------------ #

    @Attribute
    def upper_lower_surfaces(self):
        coords = self.normalized_coordinates

        le_idx = np.argmin(coords[:, 0])

        upper = coords[:le_idx + 1]
        lower = coords[le_idx:]

        upper = upper[::-1]  # ensure increasing x

        return upper, lower

    # ------------------------------------------------------------------ #
    # THICKNESS FUNCTION
    # ------------------------------------------------------------------ #

    def surface_z_at(self, x_c: float):
        """Return (z_upper, z_lower) at given x/c."""
        upper, lower = self.upper_lower_surfaces

        x_u, z_u = upper[:, 0], upper[:, 1]
        x_l, z_l = lower[:, 0], lower[:, 1]

        z_upper = np.interp(x_c, x_u, z_u)
        z_lower = np.interp(x_c, x_l, z_l)

        return (
            z_upper * self.thickness_factor * self.chord,
            z_lower * self.thickness_factor * self.chord
        )

    # ------------------------------------------------------------------ #
    # GEOMETRY
    # ------------------------------------------------------------------ #

    @Part
    def geometry(self):
        return FittedCurve(
            points=self.raw_points,
            mesh_deflection=self.mesh_deflection,
        )

    # ------------------------------------------------------------------ #
    # FRAME
    # ------------------------------------------------------------------ #

    @Part
    def frame(self):
        return Frame(pos=self.position, hidden=False)
    
    # ------------------------------------------------------------------ #
    # PLACEHOLDERS
    # ------------------------------------------------------------------ #
    
    @Attribute
    def calculate_CST_coefficients(self):
        temp = 1
        return temp


# ---------------------------------------------------------------------- #
# TEST
# ---------------------------------------------------------------------- #

if __name__ == "__main__":
    from parapy.gui import display
    
    airfoil = Airfoil()
    display(airfoil)