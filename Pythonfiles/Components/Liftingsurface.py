from math import radians, tan

import numpy as np
from parapy.core import Input, Attribute, Part, child
from parapy.geom import GeomBase, LoftedSolid, translate, rotate

from Wing.Airfoil import Airfoil
from Components.Frame import Frame


class LiftingSurface(GeomBase):
    """Lifting surface geometry: root/tip airfoils + lofted solid + frame."""

    # ------------------------------------------------------------------ #
    # INPUTS
    # ------------------------------------------------------------------ #

    airfoil_root_name: str = Input("whitcomb")
    airfoil_tip_name: str = Input("simm_airfoil")
    airfoil_dir: str = Input("Pythonfiles/Testfiles/test_aircraft")

    c_root: float = Input()
    c_tip: float = Input()
    t_factor_root: float = Input(1.0)
    t_factor_tip: float = Input(1.0)

    semi_span: float = Input()
    sweep: float = Input(0.0)
    twist: float = Input(0.0)
    dihedral: float = Input(0.0)

    taper_ratio: float = Input(0.5)
    front_spar_position: float = Input(0.25)
    rear_spar_position: float = Input(0.75)
    thickness_to_chord: float = Input(0.12)
    maximum_camber: float = Input(0.04)
    maximum_camber_position: float = Input(0.4)

    material_wing: str = Input("aluminium")
    material_wingbox: str = Input("aluminium")

    mesh_deflection: float = Input(1e-4)

    # ------------------------------------------------------------------ #
    # SIZING / DERIVED GEOMETRY
    # ------------------------------------------------------------------ #

    @Attribute
    def wing_area(self) -> float:
        return (self.c_root + self.c_tip) * self.semi_span

    @Attribute
    def aspect_ratio(self) -> float:
        return (2 * self.semi_span) ** 2 / self.wing_area

    @Attribute
    def mean_aerodynamic_chord(self) -> float:
        tr = self.c_tip / self.c_root
        return (2 / 3) * self.c_root * (1 + tr + tr ** 2) / (1 + tr)

    @Attribute
    def mac_spanwise_position(self) -> float:
        tr = self.c_tip / self.c_root
        return self.semi_span * (1 + 2 * tr) / (3 * (1 + tr))

    @Attribute
    def sweep_angle_rad(self) -> float:
        return radians(self.sweep)

    @Attribute
    def dihedral_rad(self) -> float:
        return radians(self.dihedral)

    @Attribute
    def twist_rad(self) -> float:
        return radians(self.twist)

    # ------------------------------------------------------------------ #
    # MASS ESTIMATE (placeholder — fill in structural model)
    # ------------------------------------------------------------------ #

    @Attribute
    def mass(self) -> float:
        return 1.0  # TODO: implement structural mass model

    # ------------------------------------------------------------------ #
    # POSITION HELPERS
    # ------------------------------------------------------------------ #

    @Attribute
    def _tip_position(self):
        """Position of the tip airfoil, accounting for sweep, dihedral and twist."""
        return translate(
            rotate(self.position, "y", self.twist_rad),
            "y", self.semi_span,
            "x", self.semi_span * tan(self.sweep_angle_rad),
            "z", self.semi_span * np.sin(self.dihedral_rad),
        )

    # ------------------------------------------------------------------ #
    # AIRFOIL PROFILES
    # ------------------------------------------------------------------ #

    @Part
    def root_airfoil(self):
        return Airfoil(
            airfoil_name=self.airfoil_root_name,
            chord=self.c_root,
            thickness_factor=self.t_factor_root,
            airfoil_dir=self.airfoil_dir,
            mesh_deflection=self.mesh_deflection,
            position=self.position,
        )

    @Part
    def tip_airfoil(self):
        return Airfoil(
            airfoil_name=self.airfoil_tip_name,
            chord=self.c_tip,
            thickness_factor=self.t_factor_tip,
            airfoil_dir=self.airfoil_dir,
            mesh_deflection=self.mesh_deflection,
            position=self._tip_position,
        )

    # ------------------------------------------------------------------ #
    # LOFTED SOLID
    # ------------------------------------------------------------------ #

    @Attribute
    def _loft_profiles(self):
        return [self.root_airfoil.geometry, self.tip_airfoil.geometry]

    @Part
    def solid(self):
        return LoftedSolid(
            profiles=self._loft_profiles,
            color="LightBlue",
            mesh_deflection=self.mesh_deflection,
        )

    # ------------------------------------------------------------------ #
    # FRAME VISUALISATION
    # ------------------------------------------------------------------ #

    @Part
    def frame(self):
        return Frame(
            pos=self.position,
            hidden=False,
        )


# ---------------------------------------------------------------------- #
# TEST
# ---------------------------------------------------------------------- #

if __name__ == "__main__":
    from parapy.gui import display

    ls = LiftingSurface(
        label="lifting_surface",
        c_root=5.0,
        c_tip=2.5,
        semi_span=27.0,
        sweep=25.0,
        twist=-2.0,
        dihedral=5.0,
        mesh_deflection=1e-4,
    )
    display(ls)