import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from parapy.core import Input, Attribute, Part
from parapy.geom import GeomBase

from Pythonfiles.Components.Liftingsurfaces.Liftingsurface import LiftingSurface
from Pythonfiles.Components.Fuselage.Fuselage import Fuselage
from Pythonfiles.Components.Engines.Engine import Engine


class Aircraft(GeomBase):

    # ============================================================ #
    # MISSION  — always required, no sensible universal default
    # ============================================================ #

    cruise_speed:   float = Input()   # [m/s]
    cruise_altitude: float = Input()
    aircraft_mass:  float = Input()   # [kg]  MTOW
    ld_required: float = Input(None)
    maximum_load_factor: float = Input(1)

    # ============================================================ #
    # WING SIZING  — mission-driven, required
    # ============================================================ #

    effective_wing_area:      float = Input()   # [m²]  from wing loading W/S
    effective_wing_semi_span: float = Input()   # [m]   from AR / airport constraint

    # ============================================================ #
    # FUSELAGE  — mission/payload driven, required
    # ============================================================ #

    fuselage_cylinder_start: float = Input()   # [% of fuselage length]
    fuselage_cylinder_end:   float = Input()   # [% of fuselage length]

    # ============================================================ #
    # WING AERODYNAMICS
    # Defaults: Raymer subsonic conventional fixed-wing UAV baseline.
    # Override for swept/high-speed/unconventional designs.
    # ============================================================ #

    wing_taper_ratio:             float = Input(0.40)   # Raymer: efficient subsonic 0.3–0.5
    wing_sweep_le:                float = Input(5.0)    # [deg]  low-speed, near-straight
    wing_twist:                   float = Input(0.0)    # [deg]  0 = no washout (simple UAV)
    wing_dihedral:                float = Input(5.0)    # [deg]  Raymer: stability rule-of-thumb
    wing_thickness_to_chord:      float = Input(0.15)   # Raymer: subsonic structural/aero compromise
    wing_maximum_camber:          float = Input(0.04)   # typical cambered NACA section
    wing_maximum_camber_position: float = Input(0.40)   # NACA 4-series default
    wing_t_factor_root:           float = Input(1.0)    # no thickness scaling
    wing_t_factor_tip:            float = Input(1.0)

    # ============================================================ #
    # TAIL AERODYNAMICS
    # Defaults: Raymer conventional empennage baseline.
    # ============================================================ #

    tail_taper_ratio:             float = Input(0.40)   # same logic as wing
    tail_sweep_le:                float = Input(10.0)   # [deg]  slightly more swept for stability
    tail_twist:                   float = Input(0.0)
    tail_dihedral:                float = Input(0.0)
    tail_thickness_to_chord:      float = Input(0.15)
    tail_maximum_camber:          float = Input(0.0)    # symmetric section (standard tail)
    tail_maximum_camber_position: float = Input(0.0)
    tail_t_factor_root:           float = Input(1.0)
    tail_t_factor_tip:            float = Input(1.0)

    # ============================================================ #
    # ENGINE / PROPULSION  — required
    # ============================================================ #

    thrust_to_weight: float = Input()   # performance requirement
    rho:              float = Input()   # [kg/m³] at cruise altitude

    # ============================================================ #
    # UNDERCARRIAGE
    # ============================================================ #

    undercarriage_retractible: bool = Input(False)

    # ============================================================ #
    # COLORS  — visual only, all have defaults
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
    # Defaults are the standard textbook values; override if needed.
    # ============================================================ #

    # --- Tail sizing (Roskam) ---
    tail_volume_coefficient_h: float = Input(0.60)   # typical transport/UAV HT
    tail_volume_coefficient_v: float = Input(0.04)   # typical transport/UAV VT
    tail_aspect_ratio_h:       float = Input(4.50)   # Roskam range 3–5
    tail_aspect_ratio_v:       float = Input(1.80)   # Roskam range 1.2–2.5

    # --- Propeller/rotor sizing (Roskam / UAV empirical) ---
    disk_loading_uav:  float = Input(80.0)    # [N/m²]  UAV empirical
    target_solidity:   float = Input(0.15)    # Roskam: ~0.10–0.20

    # --- Blade geometry ---
    blade_sweep: float = Input(5.0)   # [deg]  empirical aero smoothing

    # --- Nacelle / blade overrides (None = auto-sized) ---
    nacelle_length_override:    float = Input(None)
    nacelle_radius_override:    float = Input(None)
    n_blades_override:          int   = Input(None)
    blade_length_override:      float = Input(None)
    blade_root_chord_override:  float = Input(None)

    # ============================================================ #
    # FIXED PHYSICAL / STRUCTURAL CONSTANTS
    # Override only for exotic configurations.
    # ============================================================ #

    g: float = Input(9.81)   # [m/s²]

    # Spar positions as fraction of local chord (structural convention)
    wing_front_spar_position: float = Input(0.15)
    wing_rear_spar_position:  float = Input(0.60)
    tail_front_spar_position: float = Input(0.15)
    tail_rear_spar_position:  float = Input(0.60)

    # Inlet / nozzle geometry ratios
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