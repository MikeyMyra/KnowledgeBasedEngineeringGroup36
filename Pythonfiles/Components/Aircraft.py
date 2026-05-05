import sys
import os
from math import sqrt, pi

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from parapy.core import Input, Attribute, Part
from parapy.geom import GeomBase

from Pythonfiles.Components.Liftingsurfaces.Liftingsurface import LiftingSurface
from Pythonfiles.Components.Fuselage.Fuselage import Fuselage
from Pythonfiles.Components.Engines.Engine import Engine


class Aircraft(GeomBase):
    """Top-level aircraft assembly: fuselage + wing + tails + engines.

    Area convention (used EVERYWHERE in this file):
    ─────────────────────────────────────────────────────────────────────
    wing_area   — TOTAL reference area, both halves combined  [m²]
    semi_span   — half-span of ONE side                       [m]

    Roskam defaults (Vol. I):
    ─────────────────────────────────────────────────────────────────────
    Fuselage
      length          §3.3 / Table 3.4 : L = 0.59 * MTOW^0.30  [kg→m]
      radius          §3.3             : R = L / (2 * 8)  (fineness l/d=8)

    Horizontal tail   §3.4 / Table 3.4
      V_h = S_h * l_h / (S_w * MAC) = 0.35
      → S_h_total = V_h * S_w_total * MAC / l_h

    Engine            Vol. I §3.2 / Vol. V §4 — fully inside Engine class
    ─────────────────────────────────────────────────────────────────────
    """

    # ------------------------------------------------------------------ #
    # INPUTS — GLOBAL
    # ------------------------------------------------------------------ #

    aircraft_mass: float = Input()          # MTOW [kg]
    mesh_deflection: float = Input(1e-4)

    # ------------------------------------------------------------------ #
    # INPUTS — FUSELAGE
    # ------------------------------------------------------------------ #

    fuselage_length: float = Input(None)    # None → Roskam estimate
    fuselage_radius: float = Input(None)    # None → Roskam estimate

    fuselage_cylinder_start: float = Input(10.0)
    fuselage_cylinder_end: float = Input(70.0)

    undercarriage_retractible: bool = Input(False)

    # ------------------------------------------------------------------ #
    # INPUTS — MAIN WING
    # wing_area = TOTAL area (both sides) [m²]
    # wing_semi_span = half-span [m]
    # ------------------------------------------------------------------ #

    wing_area: float = Input()              # total area, both sides [m²]
    wing_semi_span: float = Input()         # half-span [m]

    wing_taper_ratio: float = Input(0.40)
    wing_sweep_le: float = Input(5.0)
    wing_twist: float = Input(-2.0)
    wing_dihedral: float = Input(5.0)
    wing_thickness_to_chord: float = Input(0.15)
    wing_t_factor_root: float = Input(1.0)
    wing_t_factor_tip: float = Input(1.0)
    wing_front_spar_position: float = Input(0.15)
    wing_rear_spar_position: float = Input(0.60)

    # ------------------------------------------------------------------ #
    # INPUTS — HORIZONTAL TAIL
    # ------------------------------------------------------------------ #

    tail_area: float = Input(None)          # total [m²]; None → Roskam
    tail_semi_span: float = Input(None)     # [m]; None → Roskam

    tail_taper_ratio: float = Input(0.40)
    tail_sweep_le: float = Input(10.0)
    tail_twist: float = Input(0.0)
    tail_dihedral: float = Input(0.0)
    tail_thickness_to_chord: float = Input(0.10)
    tail_t_factor_root: float = Input(1.0)
    tail_t_factor_tip: float = Input(1.0)
    tail_front_spar_position: float = Input(0.15)
    tail_rear_spar_position: float = Input(0.60)


    # ------------------------------------------------------------------ #
    # INPUTS — ENGINE
    # ------------------------------------------------------------------ #

    engine_type: str = Input("propeller")
    n_engines: int = Input(1)
    thrust_to_weight: float = Input(0.35)

    nacelle_length_override: float = Input(None)
    nacelle_radius_override: float = Input(None)
    n_blades: int = Input(3)
    blade_length_override: float = Input(None)
    blade_root_chord_override: float = Input(None)

    rho: float = Input(1.225)

    # ------------------------------------------------------------------ #
    # ROSKAM FUSELAGE ESTIMATES
    # ------------------------------------------------------------------ #

    @Attribute
    def _fus_length(self) -> float:
        if self.fuselage_length is not None:
            return self.fuselage_length

        # Roskam-style UAV approximation:
        # fuselage length driven by tail moment arm (~0.55–0.65 span)
        span = 2.0 * self.wing_semi_span

        tail_arm = 0.6 * span          # MALE UAV typical (Roskam-style stat. ratio)
        fuselage_length = tail_arm / 0.85  # tail at ~85% fuselage length

        return fuselage_length

    @Attribute
    def _fus_radius(self) -> float:
        if self.fuselage_radius is not None:
            return self.fuselage_radius

        fineness_ratio = 10.0  # UAV / MALE typical (Roskam GA upper adaptation)
        diameter = self._fus_length / fineness_ratio
        return diameter / 2.0

    # ------------------------------------------------------------------ #
    # WING MAC — needed here for tail sizing
    # wing_area is TOTAL, wing_semi_span is half-span
    #   S_total = semi_span * c_root * (1 + taper)
    #   → c_root = S_total / (semi_span * (1 + taper))
    # ------------------------------------------------------------------ #

    @Attribute
    def _wing_mac(self) -> float:
        tr = self.wing_taper_ratio
        c_root = self.wing_area / (self.wing_semi_span * (1 + tr))
        return (2 / 3) * c_root * (1 + tr + tr ** 2) / (1 + tr)

    # ------------------------------------------------------------------ #
    # ENGINE POSITION
    # ------------------------------------------------------------------ #

    @Attribute
    def _engine_wing_root_x(self) -> float:
        return self.main_wing.attach_x

    @Attribute
    def _engine_wing_root_z(self) -> float:
        return self.main_wing.attach_z

    # ------------------------------------------------------------------ #
    # PARTS
    # ------------------------------------------------------------------ #

    @Part
    def fuselage(self):
        return Fuselage(
            aircraft_mass=self.aircraft_mass,
            length=self._fus_length,
            radius=self._fus_radius,
            cylinder_start=self.fuselage_cylinder_start,
            cylinder_end=self.fuselage_cylinder_end,
            undercarriage_retractible=self.undercarriage_retractible,
            mesh_deflection=self.mesh_deflection,
        )

    @Part
    def main_wing(self):
        return LiftingSurface(
            label="main_wing",

            wing_area=self.wing_area,           # total [m²]
            semi_span=self.wing_semi_span,       # half-span [m]

            is_tail=False,
            is_vertical_tail=False,

            taper_ratio=self.wing_taper_ratio,
            sweep_le=self.wing_sweep_le,
            twist=self.wing_twist,
            dihedral=self.wing_dihedral,
            thickness_to_chord=self.wing_thickness_to_chord,
            t_factor_root=self.wing_t_factor_root,
            t_factor_tip=self.wing_t_factor_tip,
            front_spar_position=self.wing_front_spar_position,
            rear_spar_position=self.wing_rear_spar_position,

            fuselage_length=self._fus_length,
            fuselage_radius=self._fus_radius,
        )

    @Part
    def horizontal_tail(self):
        return LiftingSurface(
            label="horizontal_tail",

            wing_ref=self.main_wing,
            is_tail=True,
            is_vertical_tail=False,

            taper_ratio=self.tail_taper_ratio,
            sweep_le=self.tail_sweep_le,
            twist=self.tail_twist,
            dihedral=self.tail_dihedral,
            thickness_to_chord=self.tail_thickness_to_chord,
            t_factor_root=self.tail_t_factor_root,
            t_factor_tip=self.tail_t_factor_tip,
            front_spar_position=self.tail_front_spar_position,
            rear_spar_position=self.tail_rear_spar_position,

            fuselage_length=self._fus_length,
            fuselage_radius=self._fus_radius,
        )

    @Part
    def vertical_tail(self):
        return LiftingSurface(
            label="vertical_tail",

            wing_ref=self.main_wing,
            is_tail=True,
            is_vertical_tail=True,

            taper_ratio=self.tail_taper_ratio,
            sweep_le=35.0,

            thickness_to_chord=self.tail_thickness_to_chord,
            t_factor_root=self.tail_t_factor_root,
            t_factor_tip=self.tail_t_factor_tip,
            front_spar_position=self.tail_front_spar_position,
            rear_spar_position=self.tail_rear_spar_position,

            fuselage_length=self._fus_length,
            fuselage_radius=self._fus_radius,

            wing_area=1.0,      # dummy
            semi_span=1.0,      # dummy
        )

    @Part
    def engines(self):
        return Engine(
            engine_type=self.engine_type,
            mtow=self.aircraft_mass,
            n_engines=self.n_engines,
            thrust_to_weight=self.thrust_to_weight,
            semi_span=self.wing_semi_span,
            sweep_le=self.wing_sweep_le,
            dihedral=self.wing_dihedral,
            wing_root_x=self._engine_wing_root_x,
            wing_root_z=self._engine_wing_root_z,
            nacelle_length_override=self.nacelle_length_override,
            nacelle_radius_override=self.nacelle_radius_override,
            blade_length_override=self.blade_length_override,
            blade_root_chord_override=self.blade_root_chord_override,
            rho=self.rho,
            mesh_deflection=self.mesh_deflection,
        )


# ---------------------------------------------------------------------- #
# TEST
# ---------------------------------------------------------------------- #

if __name__ == "__main__":
    from parapy.gui import display

    ac = Aircraft(
        label="drone",

        # --- MASS ---
        aircraft_mass=1000,        # kg (realistic MALE lower-end)

        # --- WING ---
        wing_area=20,             # m²  
        wing_semi_span=8,         # m 

        # --- PROPULSION ---
        engine_type="propeller",
        thrust_to_weight=0.35,    # efficient cruise UAV
    )

    display(ac)