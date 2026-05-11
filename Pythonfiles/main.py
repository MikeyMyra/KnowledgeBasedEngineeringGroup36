import sys
import os
import math
from typing import Optional

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from parapy.core import Input, Attribute, Part
from parapy.geom import GeomBase

from Pythonfiles.Components.Aircraft import Aircraft
from mission import Mission
from Pythonfiles.Components.Payload import Payload


class Drone(GeomBase):
    """Top-level drone class.  Mission sizing feeds directly into Aircraft geometry."""

    # ============================================================ #
    # MISSION  — always required
    # ============================================================ #

    cruise_speed:      float = Input()   # [m/s]
    loiter_speed:      float = Input()   # [m/s]
    mission_altitude:  float = Input()   # [m]
    mission_range:     float = Input()   # [km]
    mission_endurance: float = Input()   # [hr]
    mission_objective: str   = Input()   # e.g. "High Endurance", "High Speed"
    maximum_load_factor: float = Input()

    # ============================================================ #
    # PAYLOAD  — required
    # ============================================================ #

    uav_class:      int = Input()
    payload_config: int = Input()
    weapon_count:   int = Input()

    # ============================================================ #
    # PROPULSION / ATMOSPHERE
    # ============================================================ #

    specific_fuel:    float = Input(0.5)    # [1/hr]  SFC
    prop_efficiency:  float = Input(0.8)
    maximum_mach:     float = Input(0.5)
    rho:              float = Input(1.225)  # [kg/m³]  ISA sea level; override for altitude

    # ============================================================ #
    # PERFORMANCE
    # ============================================================ #

    oswald_factor:   float = Input(0.8)
    reserve_time:    float = Input(0.5)    # [hr]

    # ============================================================ #
    # WING GEOMETRY
    # wing_aspect_ratio: Raymer Table 4.1 for subsonic UAV/endurance
    #   endurance UAV:  8–12  (high AR for L/D)
    #   trainer/utility: 6–8
    #   Default 8 is a conservative subsonic endurance baseline.
    # Semi-span is then derived: b = sqrt(AR * S)  →  b/2 = sqrt(AR*S)/2
    # ============================================================ #

    wing_aspect_ratio: float = Input(8.0)   # Raymer subsonic endurance UAV default

    # All wing/tail aero inputs below mirror Aircraft defaults exactly.
    # They are declared here so Drone's __main__ block (and any parent
    # class) can override them without touching Aircraft directly.

    wing_taper_ratio:             float = Input(0.40)
    wing_sweep_le:                float = Input(5.0)    # [deg]
    wing_twist:                   float = Input(0.0)    # [deg]
    wing_dihedral:                float = Input(5.0)    # [deg]
    wing_thickness_to_chord:      float = Input(0.15)
    wing_maximum_camber:          float = Input(0.04)
    wing_maximum_camber_position: float = Input(0.40)
    wing_t_factor_root:           float = Input(1.0)
    wing_t_factor_tip:            float = Input(1.0)

    # ============================================================ #
    # TAIL GEOMETRY  — mirrors Aircraft defaults
    # ============================================================ #

    tail_taper_ratio:             float = Input(0.40)
    tail_sweep_le:                float = Input(10.0)   # [deg]
    tail_twist:                   float = Input(0.0)
    tail_dihedral:                float = Input(0.0)
    tail_thickness_to_chord:      float = Input(0.15)
    tail_maximum_camber:          float = Input(0.0)
    tail_maximum_camber_position: float = Input(0.0)
    tail_t_factor_root:           float = Input(1.0)
    tail_t_factor_tip:            float = Input(1.0)

    # ============================================================ #
    # FUSELAGE
    # ============================================================ #

    fuselage_cylinder_start: float = Input(10.0)   # [% of fuselage length]
    fuselage_cylinder_end:   float = Input(70.0)   # [% of fuselage length]
    undercarriage_retractible: bool = Input(False)

    # ============================================================ #
    # ROSKAM / RAYMER EMPIRICAL CONSTANTS  — override if needed
    # ============================================================ #

    tail_volume_coefficient_h: float = Input(0.60)
    tail_volume_coefficient_v: float = Input(0.04)
    tail_aspect_ratio_h:       float = Input(4.50)
    tail_aspect_ratio_v:       float = Input(1.80)

    disk_loading_uav:  float = Input(80.0)
    target_solidity:   float = Input(0.15)
    blade_sweep:       float = Input(5.0)    # [deg]

    thrust_to_weight:  float = Input(0.35)   # Raymer: UAV typical 0.25–0.5

    # ============================================================ #
    # STRUCTURAL CONSTANTS  — override only for exotic configs
    # ============================================================ #

    wing_front_spar_position: float = Input(0.15)
    wing_rear_spar_position:  float = Input(0.60)
    tail_front_spar_position: float = Input(0.15)
    tail_rear_spar_position:  float = Input(0.60)

    inlet_radius_ratio:  float = Input(0.85)
    nozzle_radius_ratio: float = Input(0.70)

    # ============================================================ #
    # NACELLE / BLADE OVERRIDES  (None = auto-sized)
    # ============================================================ #

    nacelle_length_override:   float = Input(None)
    nacelle_radius_override:   float = Input(None)
    n_blades_override:         int   = Input(None)
    blade_length_override:     float = Input(None)
    blade_root_chord_override: float = Input(None)

    # ============================================================ #
    # COLORS  — visual only
    # ============================================================ #

    fuselage_cones_color:           object = Input("steelblue")
    fuselage_cylinder_color:        object = Input("blue")
    undercarriage_color_tyre:       str    = Input("black")
    undercarriage_color_axle:       str    = Input("white")
    undercarriage_color_strut:      str    = Input("silver")
    main_wing_color_wingbox:        str    = Input("black")
    main_wing_color_liftingsurface: str    = Input("yellow")
    tail_h_color_wingbox:           str    = Input("black")
    tail_h_color_liftingsurface:    str    = Input("silver")
    tail_v_color_wingbox:           str    = Input("black")
    tail_v_color_liftingsurface:    str    = Input("white")
    engine_color_nacelle:           str    = Input("silver")

    # ============================================================ #
    # PAYLOAD
    # ============================================================ #

    @Attribute
    def payload(self) -> Payload:
        return Payload(
            uav_class=self.uav_class,
            payload_config=self.payload_config,
            weapon_count=self.weapon_count,
        )

    @Attribute
    def payload_weight(self) -> float:
        return self.payload.total_mass

    # ============================================================ #
    # MISSION SIZING
    # ============================================================ #

    @Attribute
    def mission(self) -> Mission:
        return Mission(
            mission_altitude=self.mission_altitude,
            mission_range=self.mission_range,
            mission_endurance=self.mission_endurance,
            payload_weight=self.payload_weight,
            specific_fuel=self.specific_fuel,
            maximum_mach=self.maximum_mach,
            prop_efficiency=self.prop_efficiency,
            cruise_speed=self.cruise_speed,
            loiter_speed=self.loiter_speed,
            mission_objective=self.mission_objective,
            oswald_factor=self.oswald_factor,
            reserve_time=self.reserve_time,
        )

    @Attribute
    def MTOW(self) -> float:
        MTOW, _, _ = self.mission.fuel_weight_sizing
        return MTOW

    @Attribute
    def empty_weight(self) -> float:
        _, empty_weight, _ = self.mission.fuel_weight_sizing
        return empty_weight

    @Attribute
    def fuel_weight(self) -> float:
        _, _, fuel_weight = self.mission.fuel_weight_sizing
        return fuel_weight

    @Attribute
    def wing_loading(self) -> float:
        """Wing loading W/S [N/m²] from Mission sizing."""
        W_S, _ = self.mission.thrust_and_wing_loading
        return W_S

    @Attribute
    def power_loading(self) -> Optional[float]:
        """Power loading W/P for turboprop / piston (None for jets)."""
        if self.mission.engine_type in ("Turboprop", "Piston"):
            _, W_P = self.mission.thrust_and_wing_loading
            return W_P
        return None

    @Attribute
    def thrust_loading(self) -> Optional[float]:
        """Thrust loading T/W for jets (None for props/pistons)."""
        if self.mission.engine_type == "Jet":
            _, T_W = self.mission.thrust_and_wing_loading
            return T_W
        return None

    # ============================================================ #
    # WING SIZING
    # ============================================================ #

    @Attribute
    def wing_area(self) -> float:
        """Wing reference area from MTOW and wing loading [m²]."""
        return (self.MTOW * 9.80665) / self.wing_loading

    @Attribute
    def wing_semi_span(self) -> float:
        """
        Wing semi-span derived from Raymer's AR definition:
            AR = b² / S  →  b = sqrt(AR · S)  →  b/2 = sqrt(AR · S) / 2
        Uses the input wing_aspect_ratio (default 8, Raymer subsonic endurance UAV).
        """
        return math.sqrt(self.wing_aspect_ratio * self.wing_area) / 2.0

    @Attribute
    def ld_cruise(self) -> float:
        """
        Critical L/D at the cruise design point, taken from Mission.
        This is the value the airfoil sweep must be designed to achieve.
        Cruise is the critical segment because:
          - it sets the required CL (hence camber and thickness)
          - it is the longest segment, so drag directly drives fuel weight
        """
        return self.mission.ld_cruise 

    # ============================================================ #
    # GEOMETRY
    # ============================================================ #

    @Part
    def aircraft(self) -> Aircraft:
        return Aircraft(
            # --- mission ---
            cruise_speed=self.cruise_speed,
            aircraft_mass=self.MTOW,
            cruise_altitude=self.mission_altitude,
            thrust_to_weight=self.thrust_to_weight,
            ld_required=self.ld_cruise,
            maximum_load_factor=self.maximum_load_factor,

            # --- wing sizing (mission-derived) ---
            effective_wing_area=self.wing_area,
            effective_wing_semi_span=self.wing_semi_span,

            # --- wing geometry ---
            wing_taper_ratio=self.wing_taper_ratio,
            wing_sweep_le=self.wing_sweep_le,
            wing_twist=self.wing_twist,
            wing_dihedral=self.wing_dihedral,
            wing_thickness_to_chord=self.wing_thickness_to_chord,
            wing_maximum_camber=self.wing_maximum_camber,
            wing_maximum_camber_position=self.wing_maximum_camber_position,
            wing_t_factor_root=self.wing_t_factor_root,
            wing_t_factor_tip=self.wing_t_factor_tip,

            # --- tail geometry ---
            tail_taper_ratio=self.tail_taper_ratio,
            tail_sweep_le=self.tail_sweep_le,
            tail_twist=self.tail_twist,
            tail_dihedral=self.tail_dihedral,
            tail_thickness_to_chord=self.tail_thickness_to_chord,
            tail_maximum_camber=self.tail_maximum_camber,
            tail_maximum_camber_position=self.tail_maximum_camber_position,
            tail_t_factor_root=self.tail_t_factor_root,
            tail_t_factor_tip=self.tail_t_factor_tip,

            # --- fuselage ---
            fuselage_cylinder_start=self.fuselage_cylinder_start,
            fuselage_cylinder_end=self.fuselage_cylinder_end,
            undercarriage_retractible=self.undercarriage_retractible,

            # --- Roskam / Raymer empirical ---
            tail_volume_coefficient_h=self.tail_volume_coefficient_h,
            tail_volume_coefficient_v=self.tail_volume_coefficient_v,
            tail_aspect_ratio_h=self.tail_aspect_ratio_h,
            tail_aspect_ratio_v=self.tail_aspect_ratio_v,
            disk_loading_uav=self.disk_loading_uav,
            target_solidity=self.target_solidity,
            blade_sweep=self.blade_sweep,

            # --- structural constants ---
            wing_front_spar_position=self.wing_front_spar_position,
            wing_rear_spar_position=self.wing_rear_spar_position,
            tail_front_spar_position=self.tail_front_spar_position,
            tail_rear_spar_position=self.tail_rear_spar_position,
            inlet_radius_ratio=self.inlet_radius_ratio,
            nozzle_radius_ratio=self.nozzle_radius_ratio,

            # --- nacelle / blade overrides ---
            nacelle_length_override=self.nacelle_length_override,
            nacelle_radius_override=self.nacelle_radius_override,
            n_blades_override=self.n_blades_override,
            blade_length_override=self.blade_length_override,
            blade_root_chord_override=self.blade_root_chord_override,

            # --- colors ---
            fuselage_cones_color=self.fuselage_cones_color,
            fuselage_cylinder_color=self.fuselage_cylinder_color,
            undercarriage_color_tyre=self.undercarriage_color_tyre,
            undercarriage_color_axle=self.undercarriage_color_axle,
            undercarriage_color_strut=self.undercarriage_color_strut,
            main_wing_color_wingbox=self.main_wing_color_wingbox,
            main_wing_color_liftingsurface=self.main_wing_color_liftingsurface,
            tail_h_color_wingbox=self.tail_h_color_wingbox,
            tail_h_color_liftingsurface=self.tail_h_color_liftingsurface,
            tail_v_color_wingbox=self.tail_v_color_wingbox,
            tail_v_color_liftingsurface=self.tail_v_color_liftingsurface,
            engine_color_nacelle=self.engine_color_nacelle,
        )


# TODO: ADD PROPER CD AND CM PLOTS
# TODO: ADD PROPER .DAT FILE OUTPUTS
# TODO: CHECK IF AIRFOIL SWEEP SHOULD INTRODUCE ANGLE OF ATTACK IN THE EQUATION
# TODO: CONNECT THE PAYLOAD CHOICES TO MISSION CHARACTERISTICS BY ENGINEERING RULES
# TODO: CREATE PAYLOAD FITTING METHODS AND SHOW THEM IN THE GEOMETRY
# TODO: CLEAN UP THE CODEBASE OVERALL
# TODO: MAKE THE OVERALL UI BETTER


# ================================================================ #
# ENTRY POINT
# Only the three payload inputs are truly required.
# Everything else has a mission/Raymer default — uncomment to override.
# ================================================================ #

if __name__ == "__main__":
    from parapy.gui import display

    d = Drone(
        # --------------------------------------------------------- #
        # PAYLOAD  — required
        # --------------------------------------------------------- #
        uav_class="large",
        payload_config=[
            ("flight_computer", "flight_computer_cube_orange"),
            ("battery",         "battery_large_lipo"),
            ("eo_ir",           "eo_ir_gimbal_hd"),
            ("radar",           "unknown_radar_xyz"),
            ("weapon",          "weapon_gbu12"),
        ],
        weapon_count=2,

        # --------------------------------------------------------- #
        # MISSION  — required
        # --------------------------------------------------------- #
        cruise_speed=80.0,          # [m/s]
        loiter_speed=57.0,          # [m/s]
        mission_altitude=6000,      # [m]
        mission_range=10,           # [km]
        mission_endurance=8,        # [hr]
        mission_objective="High Endurance",
        maximum_load_factor=2.5,

        # --------------------------------------------------------- #
        # OPTIONAL OVERRIDES  (uncomment to change from default)
        # --------------------------------------------------------- #
        # --- Mission tweaks ---
        # specific_fuel=0.5,
        # prop_efficiency=0.8,
        # maximum_mach=0.5,
        # oswald_factor=0.8,
        # reserve_time=0.5,

        # --- Propulsion / atmosphere ---
        # rho=1.225,
        # thrust_to_weight=0.35,

        # --- Wing sizing ---
        # wing_aspect_ratio=8.0,        # Raymer: b = sqrt(AR * S) / 2

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

        # --- Fuselage ---
        # fuselage_cylinder_start=10.0,
        # fuselage_cylinder_end=70.0,
        # undercarriage_retractible=False,

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

    display(d)