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
    """Top-level aircraft assembly: fuselage + wing + horizontal tail + engines.

    Roskam defaults (Vol. I):
    ─────────────────────────────────────────────────────────────────────
    Fuselage
      length          §3.3 / Table 3.4 : L = 0.59 * MTOW^0.30  [kg→m]
      radius          §3.3             : R = L / (2 * fineness), l/d = 8

    Horizontal tail   §3.4 / Table 3.4
      tail area       V_h = S_h*l_h / (S_w*MAC) = 0.35  → S_h = V_h*S_w*MAC/l_h
      tail semi-span  AR_h = 4.5  → b_h = sqrt(AR_h * S_h_total)
      tail moment arm l_h = 0.50 * L_fus  (qc-to-qc approximation)
      All tail planform angles default inside LiftingSurface (Roskam Table 3.6)

    Engine            Vol. I §3.2 / Vol. V §4 — fully inside Engine class
    ─────────────────────────────────────────────────────────────────────

    Minimum required inputs for a complete 3-D model:
        aircraft_mass, wing_area, wing_semi_span
    Everything else has a Roskam-based default or is derived.
    """

    # ------------------------------------------------------------------ #
    # INPUTS — GLOBAL (always required)
    # ------------------------------------------------------------------ #

    aircraft_mass: float = Input()          # MTOW [kg]
    mesh_deflection: float = Input(1e-4)

    # ------------------------------------------------------------------ #
    # INPUTS — FUSELAGE (all optional — Roskam defaults apply)
    # ------------------------------------------------------------------ #

    # Pass explicit values to override Roskam estimates inside Fuselage.
    # Roskam Vol. I, §3.3: L = 0.59*m^0.30, R = L/(2*8).
    # NOTE (Roskam): these are None by default; Fuselage resolves them internally.
    fuselage_length: float = Input(None)
    fuselage_radius: float = Input(None)

    # Roskam Vol. I, §3.3, Fig. 3.7: 10% nosecone, 70% tailcone start.
    fuselage_cylinder_start: float = Input(10.0)
    fuselage_cylinder_end: float = Input(70.0)

    undercarriage_retractible: bool = Input(False)

    # ------------------------------------------------------------------ #
    # INPUTS — MAIN WING (area + semi_span always required)
    # ------------------------------------------------------------------ #

    wing_area: float = Input()          # one-side reference area [m²]
    wing_semi_span: float = Input()     # half-span [m]

    # Everything below has Roskam-based defaults inside LiftingSurface.
    # Only set these if you want to override a specific value.
    wing_airfoil_root: str = Input("whitcomb")
    wing_airfoil_tip: str = Input("whitcomb")
    wing_taper_ratio: float = Input(0.40)       # Roskam Vol. I, Table 3.6
    wing_sweep_le: float = Input(5.0)           # Roskam Vol. I, Table 3.6
    wing_twist: float = Input(-2.0)             # Roskam Vol. I, Table 3.6
    wing_dihedral: float = Input(5.0)           # Roskam Vol. I, Table 3.6
    wing_thickness_to_chord: float = Input(0.15)  # Roskam Vol. I, Table 3.5
    wing_t_factor_root: float = Input(1.0)
    wing_t_factor_tip: float = Input(1.0)
    wing_front_spar_position: float = Input(0.15)   # Roskam Vol. I, §4.1
    wing_rear_spar_position: float = Input(0.60)    # Roskam Vol. I, §4.1

    # ------------------------------------------------------------------ #
    # INPUTS — HORIZONTAL TAIL (all optional — Roskam defaults apply)
    # ------------------------------------------------------------------ #

    # Roskam Vol. I, §3.4: tail area and span are derived from wing geometry
    # via horizontal tail volume coefficient. Set these only to override.
    tail_area: float = Input(None)          # one-side [m²]; None → Roskam
    tail_semi_span: float = Input(None)     # [m]; None → Roskam

    tail_airfoil_root: str = Input("whitcomb")
    tail_airfoil_tip: str = Input("whitcomb")
    # Roskam Vol. I, Table 3.6: tail taper ~ 0.40, sweep ~ 10°, no twist/dihedral.
    tail_taper_ratio: float = Input(0.40)
    tail_sweep_le: float = Input(10.0)
    tail_twist: float = Input(0.0)
    tail_dihedral: float = Input(0.0)
    tail_thickness_to_chord: float = Input(0.10)    # Roskam Vol. I, Table 3.5
    tail_t_factor_root: float = Input(1.0)
    tail_t_factor_tip: float = Input(1.0)
    tail_front_spar_position: float = Input(0.15)
    tail_rear_spar_position: float = Input(0.60)

    # Roskam Vol. I, §3.4, Table 3.4: V_h = 0.35 for GA/UAV fixed-wing.
    # NOTE (Roskam): increase to 0.40 for aircraft with large CG travel.
    tail_volume_coeff: float = Input(0.35)

    # Roskam Vol. I, §3.4: horizontal tail AR ~ 4-5 for GA/UAV.
    # NOTE (Roskam): 4.5 is the statistical midpoint; taper ratio also affects b_h.
    tail_aspect_ratio: float = Input(4.5)

    # ------------------------------------------------------------------ #
    # INPUTS — ENGINE
    # ------------------------------------------------------------------ #

    engine_type: str = Input("propeller")       # "jet" or "propeller"
    n_engines: int = Input(1)

    # Roskam Vol. I, §3.2: T/W for UAV fixed-wing typically 0.30-0.50.
    # NOTE (Roskam): 0.35 is a conservative starting point; increase for
    # high-climb-rate or short-field UAV missions.
    thrust_to_weight: float = Input(0.35)

    attach_spanwise_pct: float = Input(0.35)
    attach_x_offset: float = Input(0.0)
    attach_z_offset: float = Input(-0.3)

    # Optional geometry overrides (None = Roskam estimate from Engine class)
    nacelle_length_override: float = Input(None)
    nacelle_radius_override: float = Input(None)
    n_blades: int = Input(3)
    blade_length_override: float = Input(None)
    blade_root_chord_override: float = Input(None)

    rho: float = Input(1.225)

    # ------------------------------------------------------------------ #
    # ROSKAM FUSELAGE ESTIMATES  (mirrors Fuselage._roskam_* but accessible
    # here for tail moment arm computation before Fuselage is instantiated)
    # ------------------------------------------------------------------ #

    @Attribute
    def _fus_length(self) -> float:
        """Resolved fuselage length [m] — user override or Roskam estimate."""
        if self.fuselage_length is not None:
            return self.fuselage_length
        # Roskam Vol. I, Table 3.4
        return 0.59 * (self.aircraft_mass ** 0.30)

    @Attribute
    def _fus_radius(self) -> float:
        """Resolved fuselage radius [m] — user override or Roskam estimate."""
        if self.fuselage_radius is not None:
            return self.fuselage_radius
        # Roskam Vol. I, §3.3: l/d = 8
        return self._fus_length / (2.0 * 8.0)

    # ------------------------------------------------------------------ #
    # ROSKAM TAIL SIZING  (Vol. I, §3.4)
    # ------------------------------------------------------------------ #

    @Attribute
    def _tail_moment_arm(self) -> float:
        """Tail moment arm l_h: qc of wing MAC to qc of tail MAC [m].

        Roskam Vol. I, §3.4: l_h ≈ 0.50 * L_fus for initial sizing.
        NOTE (Roskam): refine once wing and tail positions are fixed.
        """
        return 0.50 * self._fus_length

    @Attribute
    def _wing_mac(self) -> float:
        """Wing mean aerodynamic chord [m], derived from LiftingSurface formula."""
        tr = self.wing_taper_ratio
        c_root = (2 * self.wing_area) / (self.wing_semi_span * (1 + tr))
        return (2 / 3) * c_root * (1 + tr + tr ** 2) / (1 + tr)

    @Attribute
    def _tail_area(self) -> float:
        """Horizontal tail one-side area [m²].

        Roskam Vol. I, §3.4: V_h = S_h * l_h / (S_w * MAC)
        → S_h_total = V_h * S_w_total * MAC / l_h
        NOTE (Roskam): V_h = 0.35 for GA/UAV; increase for aft-CG designs.
        """
        if self.tail_area is not None:
            return self.tail_area
        S_w_total = 2.0 * self.wing_area
        S_h_total = self.tail_volume_coeff * S_w_total * self._wing_mac / self._tail_moment_arm
        return S_h_total / 2.0

    @Attribute
    def _tail_semi_span(self) -> float:
        """Horizontal tail semi-span [m].

        Roskam Vol. I, §3.4: AR_h = b_h² / S_h_total → b_h = sqrt(AR_h * S_h_total).
        NOTE (Roskam): AR_h = 4.5 is the statistical midpoint for GA/UAV.
        """
        if self.tail_semi_span is not None:
            return self.tail_semi_span
        S_h_total = 2.0 * self._tail_area
        b_h_total = sqrt(self.tail_aspect_ratio * S_h_total)
        return b_h_total / 2.0

    # ------------------------------------------------------------------ #
    # ENGINE POSITION DERIVED FROM WING
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
            length=self.fuselage_length,            # None → Roskam inside Fuselage
            radius=self.fuselage_radius,            # None → Roskam inside Fuselage
            cylinder_start=self.fuselage_cylinder_start,
            cylinder_end=self.fuselage_cylinder_end,
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
            fuselage_length=self._fus_length,
            fuselage_radius=self._fus_radius,
            is_tail=False,
            mesh_deflection=self.mesh_deflection,
        )

    @Part
    def tail(self):
        return LiftingSurface(
            airfoil_root_name=self.tail_airfoil_root,
            airfoil_tip_name=self.tail_airfoil_tip,
            wing_area=self._tail_area,              # Roskam Vol. I §3.4
            semi_span=self._tail_semi_span,         # Roskam Vol. I §3.4
            taper_ratio=self.tail_taper_ratio,
            sweep_le=self.tail_sweep_le,
            twist=self.tail_twist,
            dihedral=self.tail_dihedral,
            t_factor_root=self.tail_t_factor_root,
            t_factor_tip=self.tail_t_factor_tip,
            thickness_to_chord=self.tail_thickness_to_chord,
            front_spar_position=self.tail_front_spar_position,
            rear_spar_position=self.tail_rear_spar_position,
            fuselage_length=self._fus_length,
            fuselage_radius=self._fus_radius,
            is_tail=True,
            mesh_deflection=self.mesh_deflection,
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
            attach_spanwise_pct=self.attach_spanwise_pct,
            attach_x_offset=self.attach_x_offset,
            attach_z_offset=self.attach_z_offset,
            nacelle_length_override=self.nacelle_length_override,
            nacelle_radius_override=self.nacelle_radius_override,
            n_blades=self.n_blades,
            blade_length_override=self.blade_length_override,
            blade_root_chord_override=self.blade_root_chord_override,
            rho=self.rho,
            mesh_deflection=self.mesh_deflection,
        )


# ---------------------------------------------------------------------- #
# TEST — absolute minimum inputs for a 25 kg fixed-wing UAV
# ---------------------------------------------------------------------- #

if __name__ == "__main__":
    from parapy.gui import display

    ac = Aircraft(
        label="drone",
        # ── The only three inputs you truly must provide ──────────────
        aircraft_mass=25.0,         # MTOW [kg]
        wing_area=1.2,              # one-side wing area [m²]
        wing_semi_span=1.5,         # half-span [m]
        # ── Engine: type + T/W is all Roskam needs ────────────────────
        engine_type="propeller",
        thrust_to_weight=0.45,
        attach_spanwise_pct=0.0,    # centreline tractor
        attach_x_offset=-0.3,
        attach_z_offset=0.0,
        n_blades=3,
        # ── Everything else derives from Roskam ───────────────────────
    )
    display(ac)