import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from parapy.core import Input, Attribute, Part
from parapy.geom import GeomBase

from Liftingsurfaces.Liftingsurface import LiftingSurface
from Fuselage.Fuselage import Fuselage


class Aircraft(GeomBase):
    """Top-level aircraft assembly: fuselage (+ undercarriage) + main wing + tail."""

    # ------------------------------------------------------------------ #
    # INPUTS — GLOBAL
    # ------------------------------------------------------------------ #

    aircraft_mass: float = Input()
    mesh_deflection: float = Input(1e-4)

    # ------------------------------------------------------------------ #
    # INPUTS — FUSELAGE
    # ------------------------------------------------------------------ #

    fuselage_radius: float = Input()
    fuselage_length: float = Input()
    fuselage_cylinder_start: float = Input(10.0)    # % of length
    fuselage_cylinder_end: float = Input(70.0)      # % of length
    fuselage_material_skin: str = Input("aluminium")
    undercarriage_retractible: bool = Input(True)

    # ------------------------------------------------------------------ #
    # INPUTS — MAIN WING
    # ------------------------------------------------------------------ #

    wing_airfoil_root: str = Input("whitcomb")
    wing_airfoil_tip: str = Input("whitcomb")
    wing_area: float = Input()          # one side [m²]
    wing_semi_span: float = Input()
    wing_taper_ratio: float = Input(0.3)
    wing_sweep_le: float = Input(25.0)
    wing_twist: float = Input(-2.0)
    wing_dihedral: float = Input(5.0)
    wing_t_factor_root: float = Input(1.0)
    wing_t_factor_tip: float = Input(1.0)
    wing_thickness_to_chord: float = Input(0.12)
    wing_front_spar_position: float = Input(0.15)
    wing_rear_spar_position: float = Input(0.60)

    # ------------------------------------------------------------------ #
    # INPUTS — TAIL
    # ------------------------------------------------------------------ #

    tail_airfoil_root: str = Input("whitcomb")
    tail_airfoil_tip: str = Input("whitcomb")
    tail_area: float = Input()          # one side [m²]
    tail_semi_span: float = Input()
    tail_taper_ratio: float = Input(0.4)
    tail_sweep_le: float = Input(35.0)
    tail_twist: float = Input(0.0)
    tail_dihedral: float = Input(0.0)
    tail_t_factor_root: float = Input(1.0)
    tail_t_factor_tip: float = Input(1.0)
    tail_thickness_to_chord: float = Input(0.10)
    tail_front_spar_position: float = Input(0.15)
    tail_rear_spar_position: float = Input(0.60)

    # ------------------------------------------------------------------ #
    # PARTS
    # ------------------------------------------------------------------ #

    @Part
    def fuselage(self):
        return Fuselage(
            aircraft_mass=self.aircraft_mass,
            radius=self.fuselage_radius,
            length=self.fuselage_length,
            cylinder_start=self.fuselage_cylinder_start,
            cylinder_end=self.fuselage_cylinder_end,
            material_skin=self.fuselage_material_skin,
            undercarriage_retractible=self.undercarriage_retractible,
            mesh_deflection=self.mesh_deflection,
        )

    @Part
    def main_wing(self):
        return LiftingSurface(
            airfoil_root_name=self.wing_airfoil_root,
            airfoil_tip_name=self.wing_airfoil_tip,
            wing_area=self.wing_area,
            semi_span=self.wing_semi_span,
            taper_ratio=self.wing_taper_ratio,
            sweep_le=self.wing_sweep_le,
            twist=self.wing_twist,
            dihedral=self.wing_dihedral,
            t_factor_root=self.wing_t_factor_root,
            t_factor_tip=self.wing_t_factor_tip,
            thickness_to_chord=self.wing_thickness_to_chord,
            front_spar_position=self.wing_front_spar_position,
            rear_spar_position=self.wing_rear_spar_position,
            fuselage_length=self.fuselage_length,
            fuselage_radius=self.fuselage_radius,
            is_tail=False,
            mesh_deflection=self.mesh_deflection,
        )

    @Part
    def tail(self):
        return LiftingSurface(
            airfoil_root_name=self.tail_airfoil_root,
            airfoil_tip_name=self.tail_airfoil_tip,
            wing_area=self.tail_area,
            semi_span=self.tail_semi_span,
            taper_ratio=self.tail_taper_ratio,
            sweep_le=self.tail_sweep_le,
            twist=self.tail_twist,
            dihedral=self.tail_dihedral,
            t_factor_root=self.tail_t_factor_root,
            t_factor_tip=self.tail_t_factor_tip,
            thickness_to_chord=self.tail_thickness_to_chord,
            front_spar_position=self.tail_front_spar_position,
            rear_spar_position=self.tail_rear_spar_position,
            fuselage_length=self.fuselage_length,
            fuselage_radius=self.fuselage_radius,
            is_tail=True,
            mesh_deflection=self.mesh_deflection,
        )


# ---------------------------------------------------------------------- #
# TEST
# ---------------------------------------------------------------------- #

if __name__ == "__main__":
    from parapy.gui import display

    ac = Aircraft(
        label="aircraft",
        aircraft_mass=50000,
        # fuselage
        fuselage_radius=2.0,
        fuselage_length=37.0,
        fuselage_cylinder_start=10.0,
        fuselage_cylinder_end=70.0,
        undercarriage_retractible=True,
        # main wing
        wing_area=122.0,
        wing_semi_span=17.0,
        wing_taper_ratio=0.3,
        wing_sweep_le=27.0,
        wing_twist=-2.0,
        wing_dihedral=5.0,
        # tail
        tail_area=31.0,
        tail_semi_span=7.0,
        tail_taper_ratio=0.4,
        tail_sweep_le=35.0,
        tail_twist=0.0,
        tail_dihedral=0.0,
    )
    display(ac)