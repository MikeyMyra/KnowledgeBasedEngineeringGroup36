"""
drone.py
========
Top-level drone class.  Engineering rules derive UAV class,
mission objective, and payload variant selection automatically from the
mission performance inputs, so the user only needs to specify:

  REQUIRED
  --------
  cruise_speed, mission_altitude, mission_range, mission_endurance
  maximum_load_factor

  OPTIONAL  (sensible defaults or inferred)
  -----------------------------------------
  payload_categories     : ["ISR"] / ["Strike"] / ["Mapping"] / etc.
                      If omitted → inferred from mission parameters.
  weapon_count      : int (default 0)
  mission_objective : override string (default → inferred)
  uav_class         : override string (default → inferred)
  ... all geometry / propulsion / structural constants
"""

import math
from typing import Optional

from parapy.core import Input, Attribute, Part, action
from parapy.geom import GeomBase

from Pythonfiles.Components.Aircraft import Aircraft
from mission import Mission
from Pythonfiles.Components.Payload.Payload import Payload
from Pythonfiles.Components.Payload.Payloadrules import PayloadRules
from ISA_calculator import ISA_calculator


class Drone(GeomBase):

    # ================================================================ #
    # REQUIRED MISSION INPUTS
    # ================================================================ #

    cruise_speed:        float = Input()   # [m/s]
    mission_altitude:    float = Input()   # [m]
    mission_range:       float = Input()   # [km]
    mission_endurance:   float = Input()   # [hr]

    # ================================================================ #
    # PAYLOAD INTENT  — only what you want, not how to build it
    # ================================================================ #

    payload_role: str = Input("ISR")
    weapon_count: int = Input(0)

    # ================================================================ #
    # ENGINEERING RULE OVERRIDES  (optional — inferred when not set)
    # ================================================================ #

    uav_class_override:         Optional[str] = Input(None)
    mission_objective_override: Optional[str] = Input(None)

    # ================================================================ #
    # PROPULSION / ATMOSPHERE
    # ================================================================ #

    specific_fuel:   float = Input(0.5)    # [1/hr]  SFC
    prop_efficiency: float = Input(0.8)
    oswald_factor:   float = Input(0.8)
    reserve_time:    float = Input(0.5)    # [hr]

    # ================================================================ #
    # WING GEOMETRY
    # ================================================================ #

    wing_taper_ratio:             float = Input(0.40)
    wing_sweep_le:                float = Input(5.0)    # [deg]
    wing_twist:                   float = Input(0.0)    # [deg]
    wing_dihedral:                float = Input(5.0)    # [deg]
    wing_thickness_to_chord:      float = Input(0.15)
    wing_maximum_camber:          float = Input(0.04)
    wing_maximum_camber_position: float = Input(0.40)
    wing_t_factor_root:           float = Input(1.0)
    wing_t_factor_tip:            float = Input(1.0)

    # ================================================================ #
    # TAIL GEOMETRY
    # ================================================================ #

    tail_taper_ratio:             float = Input(0.40)
    tail_sweep_le:                float = Input(10.0)
    tail_twist:                   float = Input(0.0)
    tail_dihedral:                float = Input(0.0)
    tail_thickness_to_chord:      float = Input(0.15)
    tail_maximum_camber:          float = Input(0.0)
    tail_maximum_camber_position: float = Input(0.0)
    tail_t_factor_root:           float = Input(1.0)
    tail_t_factor_tip:            float = Input(1.0)

    # ================================================================ #
    # FUSELAGE
    # ================================================================ #

    fuselage_cylinder_start:   float = Input(10.0)
    fuselage_cylinder_end:     float = Input(70.0)
    undercarriage_retractible: bool  = Input(False)

    # ================================================================ #
    # ROSKAM / RAYMER EMPIRICAL CONSTANTS
    # ================================================================ #

    tail_volume_coefficient_h: float = Input(0.60)
    tail_volume_coefficient_v: float = Input(0.04)
    tail_aspect_ratio_h:       float = Input(4.50)
    tail_aspect_ratio_v:       float = Input(1.80)

    disk_loading_uav:  float = Input(80.0)
    target_solidity:   float = Input(0.15)
    blade_sweep:       float = Input(5.0)

    thrust_to_weight:  float = Input(0.35)

    # ================================================================ #
    # STRUCTURAL CONSTANTS
    # ================================================================ #

    wing_front_spar_position: float = Input(0.15)
    wing_rear_spar_position:  float = Input(0.60)
    tail_front_spar_position: float = Input(0.15)
    tail_rear_spar_position:  float = Input(0.60)

    inlet_radius_ratio:  float = Input(0.85)
    nozzle_radius_ratio: float = Input(0.70)

    # ================================================================ #
    # NACELLE / BLADE OVERRIDES  (None = auto-sized)
    # ================================================================ #

    nacelle_length_override:   float = Input(None)
    nacelle_radius_override:   float = Input(None)
    n_blades_override:         int   = Input(None)
    blade_length_override:     float = Input(None)
    blade_root_chord_override: float = Input(None)

    # ================================================================ #
    # COLORS
    # ================================================================ #

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

    # ================================================================ #
    # ATMOSPHERE  (owned here — used by both Drone and passed to Mission)
    # ================================================================ #

    @Attribute
    def speed_of_sound(self) -> float:
        """ISA speed of sound at mission altitude [m/s]."""
        return ISA_calculator(self.mission_altitude)[3]

    @Attribute
    def air_density(self) -> float:
        """ISA density at mission altitude [kg/m³]."""
        return ISA_calculator(self.mission_altitude)[2]

    # ================================================================ #
    # ENGINE TYPE  (needed to pick aspect ratio; mirrors Mission logic)
    # ================================================================ #

    @Attribute
    def engine_type(self) -> str:
        mach = self.cruise_speed / self.speed_of_sound
        if mach > 0.4:
            return "Jet"
        if self.payload_rules.mission_objective == "High Endurance":
            return "Turboprop"
        return "Piston"

    # ================================================================ #
    # WING ASPECT RATIO  (Raymer, owned here, passed into Mission)
    # ================================================================ #

    @Attribute
    def wing_aspect_ratio(self) -> float:
        """
        Raymer aspect-ratio estimate based on engine type and Mach number.
        Jet:      AR = 4.737 · M^-0.979
        Turboprop: AR = 9.2
        Piston:    AR = 7.6
        """
        if self.engine_type == "Jet":
            mach = self.cruise_speed / self.speed_of_sound
            return 4.737 * mach ** -0.979
        if self.engine_type == "Turboprop":
            return 9.2
        return 7.6   # Piston

    # ================================================================ #
    # MACH NUMBERS
    # ================================================================ #

    @Attribute
    def mach(self) -> float:
        return (self.cruise_speed) / self.speed_of_sound

    @Attribute
    def maximum_mach(self) -> float:
        """1.5 × cruise — conservative never-exceed Mach for sizing."""
        return (1.5 * self.cruise_speed) / self.speed_of_sound

    @Attribute
    def loiter_speed_seed(self) -> float:
        """
        Initial loiter speed estimate: 0.7 × cruise.
        Mission.fuel_weight_sizing refines this via
        find_optimal_loiter_speed_sizing(), so this is only a seed value.
        """
        return 0.7 * self.cruise_speed

    # ================================================================ #
    # ENGINEERING RULES  (single source of truth for all inferences)
    # ================================================================ #

    @Attribute
    def payload_rules(self) -> PayloadRules:
        return PayloadRules(
            cruise_speed=self.cruise_speed,
            mission_altitude=self.mission_altitude,
            mission_range=self.mission_range,
            mission_endurance=self.mission_endurance,
            payload_categories=[self.payload_role],
            weapon_count=self.weapon_count,
            uav_class_override=self.uav_class_override,
            mission_objective_override=self.mission_objective_override,
        )
    
    @Attribute
    def maximum_load_factor(self) -> float:
        """
        Conceptual-design estimate of positive limit load factor.
        """
        uav_class = self.payload_rules.uav_class
        mission = self.payload_rules.mission_objective
        cats = self.payload_rules._active_categories

        # Base by UAV class
        n = {
            "small": 4.0,
            "medium": 3.5,
            "large": 2.5,
        }.get(uav_class, 3.0)

        # Mission modifiers
        if mission == "High Speed":
            n += 1.0
        elif mission == "High Endurance":
            n -= 0.5

        # Payload modifiers
        if "weapon" in cats:
            n += 1.0
        elif "radar" in cats:
            n += 0.5

        return n

    # ================================================================ #
    # DERIVED MISSION PARAMETERS
    # ================================================================ #

    @Attribute
    def uav_class(self) -> str:
        return self.payload_rules.uav_class

    @Attribute
    def mission_objective(self) -> str:
        return self.payload_rules.mission_objective

    # ================================================================ #
    # PAYLOAD  (config derived from engineering rules)
    # ================================================================ #

    @Attribute
    def payload(self) -> Payload:
        return Payload(
            uav_class=self.uav_class,
            payload_config=self.payload_rules.payload_config,
            weapon_count=self.weapon_count,
        )

    @Attribute
    def payload_weight(self) -> float:
        return self.payload.total_mass

    # ================================================================ #
    # MISSION SIZING
    # ================================================================ #

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
            loiter_speed=self.loiter_speed_seed,
            mission_objective=self.mission_objective,
            oswald_factor=self.oswald_factor,
            reserve_time=self.reserve_time,
            maximum_load_factor=self.maximum_load_factor,
            # pre-computed — Mission no longer derives these itself
            wing_aspect_ratio=self.wing_aspect_ratio,
            speed_of_sound=self.speed_of_sound,
            air_density=self.air_density,
            engine_type=self.engine_type,
        )

    @action(label="Show Design Point")
    def WP_WS_diagram(self):
        self.mission.thrust_and_wing_loading_plot()

    # ================================================================ #
    # WEIGHT OUTPUTS
    # ================================================================ #

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

    # ================================================================ #
    # LOADING OUTPUTS
    # ================================================================ #

    @Attribute
    def wing_loading(self) -> float:
        W_S, _ = self.mission.thrust_and_wing_loading
        return W_S

    @Attribute
    def power_loading(self) -> Optional[float]:
        if self.engine_type in ("Turboprop", "Piston"):
            _, W_P = self.mission.thrust_and_wing_loading
            return W_P
        return None

    @Attribute
    def thrust_loading(self) -> Optional[float]:
        if self.engine_type == "Jet":
            _, T_W = self.mission.thrust_and_wing_loading
            return T_W
        return None

    # ================================================================ #
    # WING SIZING
    # ================================================================ #

    @Attribute
    def wing_area(self) -> float:
        return (self.MTOW * 9.80665) / self.wing_loading

    @Attribute
    def wing_semi_span(self) -> float:
        return math.sqrt(self.wing_aspect_ratio * self.wing_area) / 2.0

    @Attribute
    def ld_cruise(self) -> float:
        return self.mission.ld_cruise

    # ================================================================ #
    # GEOMETRY
    # ================================================================ #

    @Part
    def aircraft(self) -> Aircraft:
        return Aircraft(
            cruise_speed=self.cruise_speed,
            aircraft_mass=self.MTOW,
            cruise_altitude=self.mission_altitude,
            thrust_to_weight=self.thrust_to_weight,
            ld_required=self.ld_cruise,
            maximum_load_factor=self.maximum_load_factor,
            effective_wing_area=self.wing_area,
            effective_wing_semi_span=self.wing_semi_span,
            wing_taper_ratio=self.wing_taper_ratio,
            wing_sweep_le=self.wing_sweep_le,
            wing_twist=self.wing_twist,
            wing_dihedral=self.wing_dihedral,
            wing_thickness_to_chord=self.wing_thickness_to_chord,
            wing_maximum_camber=self.wing_maximum_camber,
            wing_maximum_camber_position=self.wing_maximum_camber_position,
            wing_t_factor_root=self.wing_t_factor_root,
            wing_t_factor_tip=self.wing_t_factor_tip,
            tail_taper_ratio=self.tail_taper_ratio,
            tail_sweep_le=self.tail_sweep_le,
            tail_twist=self.tail_twist,
            tail_dihedral=self.tail_dihedral,
            tail_thickness_to_chord=self.tail_thickness_to_chord,
            tail_maximum_camber=self.tail_maximum_camber,
            tail_maximum_camber_position=self.tail_maximum_camber_position,
            tail_t_factor_root=self.tail_t_factor_root,
            tail_t_factor_tip=self.tail_t_factor_tip,
            fuselage_cylinder_start=self.fuselage_cylinder_start,
            fuselage_cylinder_end=self.fuselage_cylinder_end,
            undercarriage_retractible=self.undercarriage_retractible,
            tail_volume_coefficient_h=self.tail_volume_coefficient_h,
            tail_volume_coefficient_v=self.tail_volume_coefficient_v,
            tail_aspect_ratio_h=self.tail_aspect_ratio_h,
            tail_aspect_ratio_v=self.tail_aspect_ratio_v,
            disk_loading_uav=self.disk_loading_uav,
            target_solidity=self.target_solidity,
            blade_sweep=self.blade_sweep,
            wing_front_spar_position=self.wing_front_spar_position,
            wing_rear_spar_position=self.wing_rear_spar_position,
            tail_front_spar_position=self.tail_front_spar_position,
            tail_rear_spar_position=self.tail_rear_spar_position,
            inlet_radius_ratio=self.inlet_radius_ratio,
            nozzle_radius_ratio=self.nozzle_radius_ratio,
            nacelle_length_override=self.nacelle_length_override,
            nacelle_radius_override=self.nacelle_radius_override,
            n_blades_override=self.n_blades_override,
            blade_length_override=self.blade_length_override,
            blade_root_chord_override=self.blade_root_chord_override,
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


# ================================================================ #
# ENTRY POINT
# ================================================================ #

if __name__ == "__main__":
    from parapy.gui import display

    d_isr = Drone(
        cruise_speed=80.0,
        mission_altitude=6000,
        mission_range=500,
        mission_endurance=8,
        payload_role="ISR",
    )

    d_strike = Drone(
        cruise_speed=250.0,
        mission_altitude=8000,
        mission_range=1000,
        mission_endurance=5,
        payload_role="Strike",
        weapon_count=2,
    )

    display([d_isr, d_strike])