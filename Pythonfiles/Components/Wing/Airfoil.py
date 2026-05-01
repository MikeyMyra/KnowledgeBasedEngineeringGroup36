import sys
import os


from parapy.geom import GeomBase, FittedCurve, Point, translate
from parapy.core import Input, Attribute, Part, child

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
from Components.Frame import Frame


class Airfoil(GeomBase):
    """Clean parametric airfoil (consistent aircraft component style)."""

    # ------------------------------------------------------------------ #
    # INPUTS
    # ------------------------------------------------------------------ #

    chord: float = Input(1.0)
    airfoil_name: str = Input("whitcomb")
    thickness_factor: float = Input(1.0)

    mesh_deflection: float = Input(1e-4)

    airfoil_dir: str = Input(
        os.path.abspath("Pythonfiles/Testfiles/test_aircraft")
    )

    # ------------------------------------------------------------------ #
    # AIRFOIL POINT GENERATION
    # ------------------------------------------------------------------ #

    @Attribute
    def raw_points(self):
        path = os.path.join(self.airfoil_dir, self.airfoil_name + ".dat")

        with open(path, "r") as f:
            pts = []
            for line in f:
                x, z = line.split(" ", 1)

                pts.append(
                    self.position.translate(
                        "x", float(x) * self.chord,
                        "z", float(z) * self.chord * self.thickness_factor,
                    )
                )

        return pts

    # ------------------------------------------------------------------ #
    # GEOMETRY (CLEAN FITTED CURVE WRAPPER)
    # ------------------------------------------------------------------ #

    @Part
    def geometry(self):
        return FittedCurve(
            points=self.raw_points,
            mesh_deflection=self.mesh_deflection,
        )

    # ------------------------------------------------------------------ #
    # FRAME VISUALISATION
    # ------------------------------------------------------------------ #

    @Part
    def frame(self):
        return Frame(
            pos=self.position,
            hidden=False
        )
    
    
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