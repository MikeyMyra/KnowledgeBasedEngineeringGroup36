"""
drone.py
========
Top-level drone class.
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

from Pythonfiles.metric_imperial_conversions import kilograms_to_pounds, feet_to_meters


class Drone(GeomBase):

    # ================================================================ #
    # REQUIRED MISSION INPUTS
    # ================================================================ #

    cruise_speed:      float = Input()   # [m/s]
    mission_altitude:  float = Input()   # [m]
    mission_range:     float = Input()   # [km]
    mission_endurance: float = Input()   # [hr]

    # ================================================================ #
    # PAYLOAD INTENT
    # ================================================================ #

    payload_role: str = Input("ISR")
    weapon_count: int = Input(0)

    # ================================================================ #
    # ENGINEERING RULE OVERRIDES
    # ================================================================ #

    uav_class_override:          Optional[str] = Input(None)
    mission_objective_override:  Optional[str] = Input(None)

    # ================================================================ #
    # FUSELAGE LAYOUT
    # ================================================================ #

    fuselage_cylinder_start: float = Input(10.0)   # [% of fuselage length]
    fuselage_cylinder_end:   float = Input(70.0)   # [% of fuselage length]

    # ================================================================ #
    # ATMOSPHERE
    # ================================================================ #

    @Attribute
    def speed_of_sound(self) -> float:
        return ISA_calculator(self.mission_altitude)[3]

    @Attribute
    def air_density(self) -> float:
        return ISA_calculator(self.mission_altitude)[2]

    # ================================================================ #
    # ENGINE TYPE
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
    # WING ASPECT RATIO
    # ================================================================ #

    @Attribute
    def wing_aspect_ratio(self) -> float:
        if self.engine_type == "Jet":
            mach_cruise = self.cruise_speed / self.speed_of_sound
            ar = 4.737 * mach_cruise ** -0.979
            # Raymer §4.4: practical AR bounds for jet UAVs
            return max(6.0, min(ar, 12.0))
        if self.engine_type == "Turboprop":
            return 9.2
        return 7.6

    # ================================================================ #
    # MACH NUMBERS
    # ================================================================ #

    @Attribute
    def mach(self) -> float:
        return self.cruise_speed / self.speed_of_sound

    @Attribute
    def maximum_mach(self) -> float:
        return (1.5 * self.cruise_speed) / self.speed_of_sound

    @Attribute
    def loiter_speed_seed(self) -> float:
        return 0.7 * self.cruise_speed

    # ================================================================ #
    # ENGINEERING RULES
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
        uav_class = self.payload_rules.uav_class
        mission   = self.payload_rules.mission_objective
        cats      = self.payload_rules._active_categories

        n = {"small": 4.0, "medium": 3.5, "large": 2.5}.get(uav_class, 3.0)
        if mission == "High Speed":
            n += 1.0
        elif mission == "High Endurance":
            n -= 0.5
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
    # PAYLOAD
    #
    # payload_start_x is set to the fuselage cylinder start + 10 mm margin.
    # The cylinder start fraction is known from fuselage_cylinder_start and
    # the Roskam length estimate (0.23 * MTOW^0.5).  We use the Roskam
    # estimate here because payload is instantiated before fuselage; the
    # fuselage then grows to accommodate via max(roskam, payload_min).
    # ================================================================ #

    @Attribute
    def _roskam_fuselage_length_estimate(self) -> float:
        """
        Quick Roskam length estimate [m] used only to set payload_start_x
        before the Fuselage part is instantiated.

        Roskam Vol. I Table 3.4: L = 0.23 * MTOW^0.5
        MTOW comes from the mission sizing result.
        """
        mtow_lbs  = kilograms_to_pounds(self.MTOW)
        length_ft = 0.23 * (mtow_lbs ** 0.50)
        return feet_to_meters(length_ft)

    @Attribute
    def payload_start_x(self) -> float:
        """
        X-position where the payload bay begins [m].

        Set to the fuselage cylinder start station + 10 mm clearance.
        Cylinder start = fuselage_cylinder_start% of total fuselage length.
        """
        cylinder_start_x = (self.fuselage_cylinder_start / 100.0) * \
                            self._roskam_fuselage_length_estimate
        return cylinder_start_x + 0.01   # 10 mm margin past nosecone junction

    @Part
    def payload(self) -> Payload:
        return Payload(
            uav_class=self.uav_class,
            payload_config=self.payload_rules.payload_config,
            weapon_count=self.weapon_count,
            payload_start_x=self.payload_start_x,
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
            maximum_mach=self.maximum_mach,
            cruise_speed=self.cruise_speed,
            loiter_speed=self.loiter_speed_seed,
            mission_objective=self.mission_objective,
            maximum_load_factor=self.maximum_load_factor,
            wing_aspect_ratio=self.wing_aspect_ratio,
            speed_of_sound=self.speed_of_sound,
            air_density=self.air_density,
            engine_type=self.engine_type,
        )

    # ================================================================ #
    # ACTIONS
    # ================================================================ #

    @action(label="Show Design Point")
    def WP_WS_diagram(self):
        self.mission.thrust_and_wing_loading_plot()

    @action(label="Run Wing Airfoil Sweep")
    def run_wing_sweep(self):
        self.aircraft.main_wing.run_sweep()

    @action(label="Plot Wing XFoil polars")
    def plot_wing_cl_alpha(self):
        self.aircraft.main_wing.plot_cl_alpha()

    @action(label="Print Stability Report")
    def print_stability_report(self):
        self.aircraft.print_stability_report()

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
            ld_required=self.ld_cruise,
            maximum_load_factor=self.maximum_load_factor,
            effective_wing_area=self.wing_area,
            effective_wing_semi_span=self.wing_semi_span,
            payload_object=self.payload,
            fuselage_cylinder_start=self.fuselage_cylinder_start,
            fuselage_cylinder_end=self.fuselage_cylinder_end,
            # Pass fuel mass for CG computation — Roskam Vol. I §8.1
            fuel_mass=self.fuel_weight,
        )

    # ================================================================ #
    # STABILITY SHORTCUTS  (surface the key numbers at drone level)
    # ================================================================ #

    @Attribute
    def cg_x(self) -> float:
        """Aircraft CG x-position from nose [m]."""
        return self.aircraft.cg_x

    @Attribute
    def neutral_point_x(self) -> float:
        """Neutral point x-position from nose [m]."""
        return self.aircraft.neutral_point_x

    @Attribute
    def static_margin(self) -> float:
        """Longitudinal static margin [% MAC]."""
        return self.aircraft.static_margin_percent

    @Attribute
    def stability_status(self) -> str:
        """Human-readable stability assessment."""
        return self.aircraft.stability_status


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
        cruise_speed=100.0,
        mission_altitude=200,
        mission_range=1000,
        mission_endurance=5,
        payload_role="Strike",
        weapon_count=2,
    )
    
    print(f"shaft_power       : {d_strike.aircraft.engines.prop_starboard.shaft_power:.1f} W")
    print(f"D_roskam          : {0.658 * (d_strike.aircraft.engines.prop_starboard.shaft_power/1000)**0.25:.3f} m")
    print(f"blade_length      : {d_strike.aircraft.engines.prop_starboard.blade_length:.3f} m")
    print(f"_max_blade_length : {d_strike.aircraft.engines.prop_starboard._max_blade_length:.3f} m")
    print(f"semi_span         : {d_strike.aircraft.engines.prop_starboard.semi_span:.3f} m")
    print(f"rho               : {d_strike.aircraft.engines.prop_starboard.rho:.4f} kg/m³")

    display([d_strike])