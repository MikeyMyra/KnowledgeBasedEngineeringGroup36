import sys
import os
from typing import Optional



from parapy.core import Input, Attribute, Part
from parapy.geom import GeomBase

from Pythonfiles.Components.Aircraft import Aircraft
from mission import Mission
from Pythonfiles.Components.Payload import Payload

# ============================================================ #
# CALCULATIONS
# ============================================================ #

# ADD ALTITUDE CALCULATION FOR MACH 

# ADD PAYLOAD WEIGHT CALCULATION (Mike)

# ADD MTOW ESTIMATION CALL MISSION.PY

# ADD WING AREA ESTIMATION CALL MISSION.PY

# ADD L/D CALCULATION CALL MISSION.PY 

# ADD AIRFOIL FITTING ITERATION WITH Q3D (Mike)

# CALL AIRCRAFT CLASS FOR GEOMETRY VISUALISATION




class Drone(GeomBase):
    """main class for drone"""
    # ==========================
    # Inputs
    # ==========================
    mission_altitude: float = Input(6000)  # [m]
    mission_range: float = Input(10)  # [km]
    mission_endurance: float = Input(8)  # [hr]
    specific_fuel: float = Input(0.5)  # [1/hr]
    maximum_mach: float = Input(0.5)
    prop_efficiency: float = Input(0.8)
    cruise_speed: float = Input(80)  # [m/s]
    loiter_speed: float = Input(57)
    mission_objective: str = Input("High Endurance")
    oswald_factor: float = Input(0.8)
    reserve_time: float = Input(0.5)  # [hr]
    uav_class: int = Input()
    payload_config: int = Input()


    # ============================================================ #
    # CALCULATIONS
    # ============================================================ #

    # ADD ALTITUDE CALCULATION FOR MACH

    # ADD PAYLOAD WEIGHT CALCULATION (Mike)
    @Attribute
    def payload(self) -> Payload:
        return Payload(
            uav_class=self.uav_class,
            payload_config=self.payload_config,
        )

    @Attribute
    def payload_weight(self) -> float:
        return self.payload.total_mass

    # ADD MTOW ESTIMATION CALL MISSION.PY

    # ==========================
    # Mission object + results
    # ==========================

    @Attribute
    def mission(self) -> Mission:
        """Mission sizing object, built from the Inputs above."""
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
        """Maximum take-off weight from mission sizing."""
        MTOW, _, _ = self.mission.fuel_weight_sizing
        return MTOW

    @Attribute
    def empty_weight(self) -> float:
        """Empty weight from mission sizing."""
        _, empty_weight, _ = self.mission.fuel_weight_sizing
        return empty_weight

    @Attribute
    def fuel_weight(self) -> float:
        """Fuel weight from mission sizing."""
        _, _, fuel_weight = self.mission.fuel_weight_sizing
        return fuel_weight

    # ADD WING AREA ESTIMATION CALL MISSION.PY

    @Attribute
    def wing_area(self) -> float:
        """Get wing area from MTOW and wing_loading"""
        return (self.MTOW * 9.80665) / self.wing_loading

    # ADD L/D CALCULATION CALL MISSION.PY

    @Attribute
    def wing_loading(self) -> float:
        """Wing loading W/S from Mission.thrust_and_wing_loading."""
        W_S, _ = self.mission.thrust_and_wing_loading
        return W_S

    @Attribute
    def power_loading(self) -> Optional[float]:
        """Power loading W/P for prop / piston engines (None for jets)."""
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

    # ADD AIRFOIL FITTING ITERATION WITH Q3D (Mike)

    # CALL AIRCRAFT CLASS FOR GEOMETRY VISUALISATION

    # ==========================
    # Geometry
    # ==========================

    @Part
    def aircraft(self) -> Aircraft:
        """Aircraft geometry.

        Hook mission results into Aircraft here once its Inputs are known.
        Example (if your Aircraft class supports it):

            return Aircraft(
                mtow=self.MTOW,
                wing_loading=self.wing_loading,
                ...
            )
        """
        return Aircraft()


if __name__ == "__main__":

