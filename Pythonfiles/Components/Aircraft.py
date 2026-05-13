import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from parapy.core import Input, Attribute, Part, action
from parapy.geom import GeomBase

from Pythonfiles.Components.Liftingsurfaces.Liftingsurface import LiftingSurface
from Pythonfiles.Components.Fuselage.Fuselage import Fuselage
from Pythonfiles.Components.Engines.Engine import Engine


class Aircraft(GeomBase):

    # ============================================================ #
    # MISSION  — always required
    # ============================================================ #

    cruise_speed:    float = Input()   # [m/s]
    cruise_altitude: float = Input()
    aircraft_mass:   float = Input()   # [kg]  MTOW
    ld_required:     float = Input(None)
    maximum_load_factor: float = Input(1)

    # ============================================================ #
    # WING SIZING
    # ============================================================ #

    effective_wing_area:      float = Input()   # [m²]
    effective_wing_semi_span: float = Input()   # [m]

    # ============================================================ #
    # FUSELAGE
    # ============================================================ #

    fuselage_cylinder_start: float = Input(10.0)
    fuselage_cylinder_end:   float = Input(70.0)
    payload_object = Input(None)

    # Fuel and engine mass fractions for CG computation
    # Roskam Vol. I §8.1: fuel CG assumed at wing AC (tanks in centre wing box).
    # Engine mass ~10% MTOW (Roskam Table 8.1 "propulsion group").
    fuel_mass:   float = Input(0.0)   # [kg] — passed in from Drone.fuel_weight
    engine_mass: float = Input(None)  # [kg] — None = 10% MTOW (Roskam default)

    # ============================================================ #
    # WING AERODYNAMICS
    # ============================================================ #

    wing_taper_ratio:             float = Input(0.40)
    wing_sweep_le:                float = Input(5.0)
    wing_twist:                   float = Input(0.0)
    wing_dihedral:                float = Input(5.0)
    wing_thickness_to_chord:      float = Input(0.15)
    wing_maximum_camber:          float = Input(0.04)
    wing_maximum_camber_position: float = Input(0.40)
    wing_t_factor_root:           float = Input(1.0)
    wing_t_factor_tip:            float = Input(1.0)

    # ============================================================ #
    # TAIL AERODYNAMICS
    # ============================================================ #

    tail_taper_ratio:             float = Input(0.40)
    tail_sweep_le:                float = Input(10.0)
    tail_twist:                   float = Input(0.0)
    tail_dihedral:                float = Input(0.0)
    tail_thickness_to_chord:      float = Input(0.15)
    tail_maximum_camber:          float = Input(0.0)
    tail_maximum_camber_position: float = Input(0.0)
    tail_t_factor_root:           float = Input(1.0)
    tail_t_factor_tip:            float = Input(1.0)

    # ============================================================ #
    # ENGINE / PROPULSION
    # ============================================================ #

    thrust_to_weight: float = Input(0.35)
    rho:              float = Input()

    # ============================================================ #
    # UNDERCARRIAGE
    # ============================================================ #

    undercarriage_retractible: bool = Input(False)

    # ============================================================ #
    # COLORS
    # ============================================================ #

    fuselage_cones_color:      object = Input("steelblue")
    fuselage_cylinder_color:   object = Input("blue")
    undercarriage_color_tyre:  str    = Input("black")
    undercarriage_color_axle:  str    = Input("white")
    undercarriage_color_strut: str    = Input("silver")

    main_wing_color_wingbox:        str = Input("black")
    main_wing_color_liftingsurface: str = Input("yellow")

    tail_h_color_wingbox:        str = Input("black")
    tail_h_color_liftingsurface: str = Input("silver")
    tail_v_color_wingbox:        str = Input("black")
    tail_v_color_liftingsurface: str = Input("white")

    engine_color_nacelle: str = Input("silver")

    # ============================================================ #
    # ROSKAM / RAYMER EMPIRICAL CONSTANTS
    # ============================================================ #

    tail_volume_coefficient_h: float = Input(0.60)
    tail_volume_coefficient_v: float = Input(0.04)
    tail_aspect_ratio_h:       float = Input(4.50)
    tail_aspect_ratio_v:       float = Input(1.80)

    disk_loading_uav:  float = Input(80.0)
    target_solidity:   float = Input(0.15)
    blade_sweep:       float = Input(5.0)

    nacelle_length_override:   float = Input(None)
    nacelle_radius_override:   float = Input(None)
    n_blades_override:         int   = Input(None)
    blade_length_override:     float = Input(None)
    blade_root_chord_override: float = Input(None)

    g: float = Input(9.81)

    wing_front_spar_position: float = Input(0.15)
    wing_rear_spar_position:  float = Input(0.60)
    tail_front_spar_position: float = Input(0.15)
    tail_rear_spar_position:  float = Input(0.60)

    inlet_radius_ratio:  float = Input(0.85)
    nozzle_radius_ratio: float = Input(0.70)

    mesh_deflection: float = Input(1e-4)

    # ============================================================ #
    # PARTS
    # ============================================================ #

    @Part
    def fuselage(self):
        return Fuselage(
            aircraft_mass=self.aircraft_mass,
            cylinder_start=self.fuselage_cylinder_start,
            cylinder_end=self.fuselage_cylinder_end,
            color_taper=self.fuselage_cones_color,
            color_cylinder=self.fuselage_cylinder_color,
            undercarriage_retractible=self.undercarriage_retractible,
            undercarriage_color_tyre=self.undercarriage_color_tyre,
            undercarriage_color_axle=self.undercarriage_color_axle,
            undercarriage_color_strut=self.undercarriage_color_strut,
            payload=self.payload_object,
            length_min_override=self._min_fuselage_length_from_wing,
            radius_min_override=self._min_fuselage_radius_from_wing,
        )

    @Part
    def main_wing(self):
        return LiftingSurface(
            label="main_wing",
            effective_area=self.effective_wing_area,
            effective_semi_span=self.effective_wing_semi_span,

            fuselage_length=self.fuselage.length,
            fuselage_radius=self.fuselage.radius,

            is_tail=False,
            is_vertical_tail=False,

            taper_ratio=self.wing_taper_ratio,
            sweep_le=self.wing_sweep_le,
            twist=self.wing_twist,
            dihedral=self.wing_dihedral,

            thickness_to_chord_input=self.wing_thickness_to_chord,
            maximum_camber_input=self.wing_maximum_camber,
            maximum_camber_position_input=self.wing_maximum_camber_position,

            t_factor_root=self.wing_t_factor_root,
            t_factor_tip=self.wing_t_factor_tip,

            front_spar_position=self.wing_front_spar_position,
            rear_spar_position=self.wing_rear_spar_position,

            wing_ref=None,

            color_wingbox=self.main_wing_color_wingbox,
            color_liftingsurface=self.main_wing_color_liftingsurface,

            ld_required=self.ld_required,
            weight=self.aircraft_mass,
            velocity=self.cruise_speed,
            altitude=self.cruise_altitude,
            maximum_load_factor=self.maximum_load_factor,
        )

    @Part
    def horizontal_tail(self):
        return LiftingSurface(
            label="horizontal_tail",
            wing_ref=self.main_wing,
            is_tail=True,
            is_vertical_tail=False,

            fuselage_length=self.fuselage.length,
            fuselage_radius=self.fuselage.radius,
            fuselage_cone_radius_fn=self.fuselage.local_radius_at,

            taper_ratio=self.tail_taper_ratio,
            sweep_le=self.tail_sweep_le,
            twist=self.tail_twist,
            dihedral=self.tail_dihedral,

            thickness_to_chord_input=self.tail_thickness_to_chord,
            maximum_camber_input=self.tail_maximum_camber,
            maximum_camber_position_input=self.tail_maximum_camber_position,

            t_factor_root=self.tail_t_factor_root,
            t_factor_tip=self.tail_t_factor_tip,

            front_spar_position=self.tail_front_spar_position,
            rear_spar_position=self.tail_rear_spar_position,

            tail_volume_coefficient_h=self.tail_volume_coefficient_h,
            tail_volume_coefficient_v=self.tail_volume_coefficient_v,
            tail_aspect_ratio_h=self.tail_aspect_ratio_h,
            tail_aspect_ratio_v=self.tail_aspect_ratio_v,

            color_wingbox=self.tail_h_color_wingbox,
            color_liftingsurface=self.tail_h_color_liftingsurface,
        )

    @Part
    def vertical_tail(self):
        return LiftingSurface(
            label="vertical_tail",
            wing_ref=self.main_wing,
            is_tail=True,
            is_vertical_tail=True,

            fuselage_length=self.fuselage.length,
            fuselage_radius=self.fuselage.radius,
            fuselage_cone_radius_fn=self.fuselage.local_radius_at,

            taper_ratio=self.tail_taper_ratio,
            sweep_le=35.0,
            twist=0.0,
            dihedral=0.0,

            thickness_to_chord_input=self.tail_thickness_to_chord,
            maximum_camber_input=0.0,
            maximum_camber_position_input=self.tail_maximum_camber_position,

            t_factor_root=self.tail_t_factor_root,
            t_factor_tip=self.tail_t_factor_tip,

            front_spar_position=self.tail_front_spar_position,
            rear_spar_position=self.tail_rear_spar_position,

            tail_volume_coefficient_h=self.tail_volume_coefficient_h,
            tail_volume_coefficient_v=self.tail_volume_coefficient_v,
            tail_aspect_ratio_h=self.tail_aspect_ratio_h,
            tail_aspect_ratio_v=self.tail_aspect_ratio_v,

            color_wingbox=self.tail_v_color_wingbox,
            color_liftingsurface=self.tail_v_color_liftingsurface,
        )

    @Part
    def engines(self):
        return Engine(
            cruise_speed=self.cruise_speed,
            mtow=self.aircraft_mass,
            thrust_to_weight=self.thrust_to_weight,

            rho=self.main_wing.density,
            g=self.g,

            semi_span=self.main_wing.effective_semi_span,
            sweep_le=self.main_wing.sweep_le,
            dihedral=self.main_wing.dihedral,

            fuselage_length=self.fuselage.length,
            fuselage_radius=self.fuselage.radius,

            wing_root_x=self.main_wing._root_position.x,
            wing_root_z=self.main_wing._root_position.z,

            disk_loading_uav=self.disk_loading_uav,
            target_solidity=self.target_solidity,

            nacelle_length_override=self.nacelle_length_override,
            nacelle_radius_override=self.nacelle_radius_override,

            inlet_radius_ratio=self.inlet_radius_ratio,
            nozzle_radius_ratio=self.nozzle_radius_ratio,

            n_blades_override=self.n_blades_override,
            blade_length_override=self.blade_length_override,
            blade_root_chord_override=self.blade_root_chord_override,

            blade_sweep=self.blade_sweep,

            color_nacelle=self.engine_color_nacelle,
        )
    
    @Attribute
    def _min_fuselage_length_from_wing(self) -> float:
        """
        Fuselage length lower bound from wing span (Raymer §4.2).
        Prevents fuselage becoming aerodynamically negligible at high altitude
        where wing area — and therefore span — grows very large.
        """
        wingspan = self.effective_wing_semi_span * 2
        return 0.75 * wingspan

    @Attribute  
    def _min_fuselage_radius_from_wing(self) -> float:
        """
        Fuselage radius lower bound from wing root chord.
        Raymer §4.2: r_fus >= 8% root chord to limit interference drag.
        """
        # root chord from area and span using taper ratio
        S    = self.effective_wing_area
        b    = self.effective_wing_semi_span * 2
        lamb = self.wing_taper_ratio
        c_root = 2 * S / (b * (1 + lamb))
        return 0.08 * c_root

    # ============================================================ #
    # STABILITY ANALYSIS  (Roskam Vol. II §3.2–3.3)
    # ============================================================ #

    # ------------------------------------------------------------ #
    # Component masses
    # ------------------------------------------------------------ #

    @Attribute
    def _engine_mass(self) -> float:
        """
        Engine group mass [kg].
        Roskam Vol. I Table 8.1: propulsion group ~9–11% MTOW.
        Default 10% when not supplied.
        """
        return self.engine_mass if self.engine_mass is not None else 0.10 * self.aircraft_mass

    @Attribute
    def mass_breakdown(self) -> dict:
        """
        Dictionary of {component_label: mass [kg]} for all major groups.

        Sources
        -------
        - Fuselage  : Roskam Vol. I §8.3 Eq. (8.5)
        - Wing/tails: Roskam Vol. I §8.4 / §8.5 Table 8.1
        - Engine    : Roskam Vol. I Table 8.1 (10% MTOW default)
        - Fuel      : from mission sizing (passed in as fuel_mass)
        - Payload   : from Payload.total_mass
        """
        d = {
            "fuselage":        self.fuselage.calculate_mass,
            "main_wing":       self.main_wing.calculate_mass,
            "horizontal_tail": self.horizontal_tail.calculate_mass,
            "vertical_tail":   self.vertical_tail.calculate_mass,
            "engine":          self._engine_mass,
            "fuel":            self.fuel_mass,
        }
        if self.payload_object is not None:
            d["payload"] = self.payload_object.total_mass
        return d

    @Attribute
    def total_structural_mass(self) -> float:
        """Sum of all component masses [kg] — should be ≈ MTOW as a sanity check."""
        return sum(self.mass_breakdown.values())

    # ------------------------------------------------------------ #
    # Component CG x-positions (from nose)
    # ------------------------------------------------------------ #

    @Attribute
    def cg_breakdown(self) -> dict:
        """
        Dictionary of {component_label: cg_x [m]} for all major groups.

        Assumptions
        -----------
        - Engine CG  : at wing AC (tractor/pusher mounted on wing leading edge)
                       Roskam Vol. I §8.1 statistical for UAV tractor configs.
        - Fuel CG    : at wing AC — fuel stored in centre wing box
                       Roskam Vol. I §8.1.
        - Payload CG : from Payload.cg_x (mass-weighted item positions).
        """
        d = {
            "fuselage":        self.fuselage.cg_x,
            "main_wing":       self.main_wing.cg_x,
            "horizontal_tail": self.horizontal_tail.cg_x,
            "vertical_tail":   self.vertical_tail.cg_x,
            "engine":          self.main_wing.x_ac,   # engine at wing AC
            "fuel":            self.main_wing.x_ac,   # fuel in centre wing box
        }
        if self.payload_object is not None:
            d["payload"] = self.payload_object.cg_x
        return d

    @Attribute
    def cg_x(self) -> float:
        """
        Aircraft CG x-position from nose [m].

        Computed as mass-weighted average of all component CGs:
            CG = Σ(m_i * x_cg_i) / Σ(m_i)

        Roskam Vol. I §8.1, Eq. (8.1).
        """
        masses = self.mass_breakdown
        cgs    = self.cg_breakdown
        total  = sum(masses.values())
        if total == 0:
            return 0.0
        return sum(masses[k] * cgs[k] for k in masses) / total

    # ------------------------------------------------------------ #
    # Neutral Point  (Roskam Vol. II §3.2)
    # ------------------------------------------------------------ #

    @Attribute
    def neutral_point_x(self) -> float:
        """
        Aircraft neutral point x-position from nose [m].

        Roskam Vol. II §3.2, Eq. (3.15) — simplified for straight-tapered
        wing + horizontal tail, subsonic incompressible:

            NP = (CL_alpha_w * x_ac_w  +  eta_h * (S_h/S_w) * CL_alpha_h * x_ac_h)
                 -----------------------------------------------------------------------
                        CL_alpha_w  +  eta_h * (S_h/S_w) * CL_alpha_h

        Assumptions
        -----------
        - CL_alpha_w = CL_alpha_h = 2π [1/rad]  (thin-aerofoil theory, subsonic)
          Roskam Vol. II §3.2: adequate for conceptual design; Q3D sweep
          would give a more accurate value per surface.
        - eta_h = 0.90 (tail efficiency, Roskam Vol. II Table 3.1 — accounts
          for fuselage wake / boundary layer losses at tail location).
        - Fuselage contribution ignored at conceptual level (conservative;
          fuselage destabilises, so true NP is slightly aft of this estimate).
        """
        cl_alpha   = 2.0 * 3.14159265   # 2π [1/rad] — thin-aerofoil theory
        eta_h      = 0.90               # tail efficiency (Roskam Vol. II Table 3.1)
        S_w        = self.main_wing._effective_area
        S_h        = self.horizontal_tail._effective_area
        x_ac_w     = self.main_wing.x_ac
        x_ac_h     = self.horizontal_tail.x_ac

        tail_factor = eta_h * (S_h / S_w)   # dimensionless tail contribution weight

        return (cl_alpha * x_ac_w + tail_factor * cl_alpha * x_ac_h) / \
               (cl_alpha + tail_factor * cl_alpha)

    # ------------------------------------------------------------ #
    # Static Margin  (Roskam Vol. II §3.3)
    # ------------------------------------------------------------ #

    @Attribute
    def static_margin(self) -> float:
        """
        Longitudinal static margin as fraction of MAC [-].

            SM = (NP - CG) / MAC_wing

        Roskam Vol. II §3.3:
        - SM > 0  → statically stable (NP aft of CG)
        - SM = 0  → neutral (NP coincides with CG)
        - SM < 0  → unstable (NP forward of CG)
        - Target  : 0.05–0.15 (5–15 % MAC) for UAVs without FBW stability
                    augmentation.  FBW aircraft may accept SM down to –0.10.

        Roskam Vol. II §3.3, Eq. (3.1): SM = (x_np - x_cg) / c_bar
        """
        return (self.neutral_point_x - self.cg_x) / self.main_wing.mean_aerodynamic_chord

    @Attribute
    def static_margin_percent(self) -> float:
        """Static margin as a percentage of MAC [%]."""
        return self.static_margin * 100.0

    @Attribute
    def is_stable(self) -> bool:
        """True when static margin > 0 (NP aft of CG)."""
        return self.static_margin > 0.0

    @Attribute
    def stability_status(self) -> str:
        """Human-readable stability assessment."""
        sm = self.static_margin_percent
        if sm >= 5.0:
            return f"STABLE — SM = {sm:.1f}% MAC (target 5–15%)"
        elif sm >= 0.0:
            return f"MARGINAL — SM = {sm:.1f}% MAC (below 5% target)"
        else:
            return f"UNSTABLE — SM = {sm:.1f}% MAC (requires FBW or redesign)"

    # ------------------------------------------------------------ #
    # Summary action
    # ------------------------------------------------------------ #

    @action(label="Print stability report")
    def print_stability_report(self):
        print("=" * 70)
        print("STABILITY REPORT")
        print("=" * 70)
        print("\nCOMPONENT MASSES & CG POSITIONS")
        print(f"  {'Component':<20s}  {'Mass [kg]':>10s}  {'CG x [m]':>10s}  {'m*x [kg·m]':>12s}")
        print("  " + "-" * 58)
        masses = self.mass_breakdown
        cgs    = self.cg_breakdown
        for k in masses:
            m  = masses[k]
            cx = cgs[k]
            print(f"  {k:<20s}  {m:>10.2f}  {cx:>10.3f}  {m*cx:>12.2f}")
        print("  " + "-" * 58)
        print(f"  {'TOTAL':<20s}  {self.total_structural_mass:>10.2f}  "
              f"{self.cg_x:>10.3f}  "
              f"{sum(masses[k]*cgs[k] for k in masses):>12.2f}")

        print("\nAERODYNAMIC REFERENCE")
        print(f"  Wing MAC              : {self.main_wing.mean_aerodynamic_chord:.3f} m")
        print(f"  Wing AC (x_ac_w)      : {self.main_wing.x_ac:.3f} m")
        print(f"  HT AC   (x_ac_h)      : {self.horizontal_tail.x_ac:.3f} m")
        print(f"  Neutral Point         : {self.neutral_point_x:.3f} m")

        print("\nLONGITUDINAL STABILITY")
        print(f"  Aircraft CG           : {self.cg_x:.3f} m from nose")
        print(f"  Neutral Point         : {self.neutral_point_x:.3f} m from nose")
        print(f"  Static Margin         : {self.static_margin_percent:.1f}% MAC")
        print(f"  Assessment            : {self.stability_status}")
        print("=" * 70)


# ================================================================ #
# ENTRY POINT
# Only mission-specific values are required here.
# Everything with a Raymer/Roskam or structural default is omitted
# unless you want to override it.
# ================================================================ #

if __name__ == "__main__":
    from parapy.gui import display

    ac = Aircraft(

        # --------------------------------------------------------- #
        # MISSION  — required
        # --------------------------------------------------------- #
        cruise_speed=220.0,          # [m/s]
        aircraft_mass=2000,          # [kg]

        # --------------------------------------------------------- #
        # WING SIZING  — required
        # --------------------------------------------------------- #
        effective_wing_area=20.0,         # [m²]  from W/S requirement
        effective_wing_semi_span=8.0,     # [m]

        # --------------------------------------------------------- #
        # TAIL AERODYNAMICS  — required
        # --------------------------------------------------------- #
        # (all tail geometry uses Raymer defaults — override below if needed)

        # --------------------------------------------------------- #
        # PROPULSION  — required
        # --------------------------------------------------------- #
        thrust_to_weight=0.35,
        #rho=1.225,                       # [kg/m³]  ISA sea level

        # --------------------------------------------------------- #
        # FUSELAGE  — required (payload-dependent)
        # --------------------------------------------------------- #
        fuselage_cylinder_start=10.0,
        fuselage_cylinder_end=70.0,

        # --------------------------------------------------------- #
        # UNDERCARRIAGE  — required
        # --------------------------------------------------------- #
        #undercarriage_retractible=False,

        # --------------------------------------------------------- #
        # OPTIONAL OVERRIDES  (uncomment to change from Raymer default)
        # --------------------------------------------------------- #
        # --- Wing geometry (Raymer subsonic baseline) ---
        # wing_taper_ratio=0.40,
        # wing_sweep_le=5.0,
        # wing_dihedral=5.0,
        # wing_twist=0.0,
        # wing_thickness_to_chord=0.15,
        # wing_maximum_camber=0.04,
        # wing_maximum_camber_position=0.4,
        # wing_t_factor_root=1.0,
        # wing_t_factor_tip=1.0,

        # --- Tail geometry (Raymer empennage baseline) ---
        # tail_taper_ratio=0.40,
        # tail_sweep_le=10.0,
        # tail_dihedral=0.0,
        # tail_twist=0.0,
        # tail_thickness_to_chord=0.15,
        # tail_maximum_camber=0.0,
        # tail_maximum_camber_position=0.0,
        # tail_t_factor_root=1.0,
        # tail_t_factor_tip=1.0,

        # --- Roskam tail sizing ---
        # tail_volume_coefficient_h=0.60,
        # tail_volume_coefficient_v=0.04,
        # tail_aspect_ratio_h=4.5,
        # tail_aspect_ratio_v=1.8,

        # --- Propeller ---
        # disk_loading_uav=80.0,
        # target_solidity=0.15,
        # blade_sweep=5.0,

        # --- Structural spar positions ---
        # wing_front_spar_position=0.15,
        # wing_rear_spar_position=0.60,
        # tail_front_spar_position=0.15,
        # tail_rear_spar_position=0.60,

        # --- Inlet / nozzle ---
        # inlet_radius_ratio=0.85,
        # nozzle_radius_ratio=0.70,

        # --- Colors ---
        # fuselage_cones_color="steelblue",
        # fuselage_cylinder_color="blue",
        # main_wing_color_liftingsurface="yellow",
        # engine_color_nacelle="silver",
    )

    display(ac)