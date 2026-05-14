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
    weapon_count: int = Input()


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
            weapon_count = self.weapon_count
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
        return Aircraft(

        # ========================================================= #
        # MISSION
        # ========================================================= #
        
        cruise_speed=220.0,
        aircraft_mass=2000,          # ✔ mission sizing driver (payload + fuel + structure assumption)

        effective_wing_area=20.0,              # ✔ driven by wing loading requirement (W/S)
        effective_wing_semi_span=8.0,          # ✔ aspect ratio / airport constraint / mission geometry

        thrust_to_weight=0.35,      # ✔ performance requirement (takeoff/climb requirement)
        
        rho=1.225,                  # ✔ ISA sea level (or mission altitude if refined later)

        # ========================================================= #
        # ROSKAM
        # ========================================================= #
        disk_loading_uav=80.0,      # ✔ Roskam / UAV empirical disk loading range
        target_solidity=0.15,       # ✔ Roskam propeller design rule (~0.1–0.2)

        tail_volume_coefficient_h=0.6,   # ✔ Roskam horizontal tail sizing
        tail_volume_coefficient_v=0.04,  # ✔ Roskam vertical tail sizing

        tail_aspect_ratio_h=4.5,         # ✔ Roskam typical HT range (3–5)
        tail_aspect_ratio_v=1.8,         # ✔ Roskam vertical tail range (1.2–2.5)

        wing_taper_ratio=0.40,           # ✔ typical efficient subsonic wing (0.3–0.5)
        wing_sweep_le=5.0,               # ✔ low-speed aircraft assumption (almost straight wing)
        wing_dihedral=5.0,               # ✔ stability rule-of-thumb
        wing_twist=0.0,
        wing_thickness_to_chord=0.15,    # ✔ subsonic structural/aero compromise
        wing_maximum_camber=0.04,        # ✔ typical cambered airfoil range
        wing_maximum_camber_position=0.4,# ✔ NACA-style default

        tail_taper_ratio=0.40,           # ✔ same logic as wing
        tail_sweep_le=10.0,              # ✔ slightly more swept tail (stability margin)
        tail_thickness_to_chord=0.15,
        tail_maximum_camber_position=0,
        tail_maximum_camber=0,
        tail_dihedral=0,
        tail_twist=0,

        blade_sweep=5.0,                 # ✔ propeller/rotor empirical aero smoothing

        # ========================================================= #
        # USER SET
        # ========================================================= #
        fuselage_cylinder_start=10.0,# To be set based on payload size
        fuselage_cylinder_end=70.0,  # Same

        undercarriage_retractible=False,  # User specified

        # ---------------- COLORS (PURE VISUAL ONLY) ----------------
        fuselage_cones_color="steelblue",
        fuselage_cylinder_color="blue",
        undercarriage_color_tyre="black",
        undercarriage_color_axle="white",
        undercarriage_color_strut="silver",

        main_wing_color_wingbox="black",
        main_wing_color_liftingsurface="yellow",

        tail_h_color_wingbox="black",
        tail_h_color_liftingsurface="silver",
        tail_v_color_wingbox="black",
        tail_v_color_liftingsurface="white",

        engine_color_nacelle="Silver",

        # ========================================================= #
        # FIXED
        # ========================================================= #
        wing_front_spar_position=0.15,   # structural convention
        wing_rear_spar_position=0.60,    # structural convention

        tail_front_spar_position=0.15,    # structural convention
        tail_rear_spar_position=0.60,     # structural convention
        
        inlet_radius_ratio=0.85,
        nozzle_radius_ratio=0.7,
        
        g=9.81,                      # ✔ physical constant (always fixed on Earth)
    )


if __name__ == "__main__":

    from parapy.gui import display

    d = Drone(
        uav_class="large",
        payload_config=[
            ("flight_computer", "flight_computer_cube_orange"),
            ("battery",         "battery_large_lipo"),
            ("eo_ir",           "eo_ir_gimbal_hd"),
            ("radar",           "unknown_radar_xyz"),   # → radar_maritime_large (large default)
            ("weapon",          "weapon_gbu12"),
        ],
        weapon_count=2)
    display(d)