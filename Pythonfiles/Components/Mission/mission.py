"""
mission.py
==========
Pure-Python mission sizing — no ParaPy dependency.
All methods are plain functions; no @Attribute caching.

Unit conventions:
  range      : km
  speed      : m/s  (converted to km/hr inside Breguet)
  SFC        : [1/hr] for jet  (thrust-specific, dimensionless fuel fraction/hr)
               [1/hr] for prop (interpreted as fuel fraction per hr)
  endurance  : hr

Breguet equations used:
  Jet range:      R[km] = V[km/hr] / SFC[1/hr] * L/D * ln(Wi/Wf)
  Prop range:     R[km] = eta * V[km/hr] / SFC[1/hr] * L/D * ln(Wi/Wf)
  Jet endurance:  E[hr] = 1/SFC[1/hr] * L/D * ln(Wi/Wf)
  Prop endurance: E[hr] = eta / SFC[1/hr] * L/D * ln(Wi/Wf)   (Raymer §3.5)

Speed convention:
  cruise_speed is used for ALL flight legs (cruise and loiter).
  This gives a single consistent L/D across the mission and avoids the
  ambiguity of a separate loiter speed input.

Mission profile:
  taxi -> climb -> cruise -> loiter -> cruise back -> reserve -> land -> taxi

Performance margins:
  - L/D        : single value, evaluated at cruise_speed and actual wing loading.
  - Fuel driver: which leg (cruise combined or loiter) consumed more fuel fraction.
"""

import numpy as np
import metric_imperial_conversions as m2i
from Pythonfiles.Components.Mission.WP_WS_diagram import WP_WS_Diagram


class Mission:

    def __init__(
        self,
        mission_altitude:    float,
        mission_range:       float,   # [km]
        mission_endurance:   float,   # [hr]
        payload_weight:      float,   # [kg]
        maximum_mach:        float,
        cruise_speed:        float,   # [m/s]  — used for all flight legs
        loiter_speed:        float,
        mission_objective:   str,
        maximum_load_factor: float,
        wing_aspect_ratio:   float,
        speed_of_sound:      float,   # [m/s]
        air_density:         float,   # [kg/m3]
        engine_type:         str,     # "Jet" | "Turboprop" | "Piston"
        specific_fuel:       float = 0.5,   # [1/hr]
        prop_efficiency:     float = 0.8,
        oswald_factor:       float = 0.8,
        reserve_time:        float = 0.5,   # [hr]
    ):
        self.mission_altitude    = mission_altitude
        self.mission_range       = mission_range
        self.mission_endurance   = mission_endurance
        self.payload_weight      = payload_weight
        self.specific_fuel       = specific_fuel
        self.maximum_mach        = maximum_mach
        self.prop_efficiency     = prop_efficiency
        self.cruise_speed        = cruise_speed
        self.loiter_speed        = loiter_speed
        self.mission_objective   = mission_objective
        self.oswald_factor       = oswald_factor
        self.reserve_time        = reserve_time
        self.maximum_load_factor = maximum_load_factor
        self.wing_aspect_ratio   = wing_aspect_ratio
        self.speed_of_sound      = speed_of_sound
        self.air_density         = air_density
        self.engine_type         = engine_type

    # ================================================================ #
    # WING PARAMETERS
    # ================================================================ #

    def _wing_parameters(self):
        if self.engine_type == "Jet":
            return 1.2, 1.5, 1.7, 0.015
        if self.engine_type == "Turboprop":
            return 1.4, 1.7, 2.0, 0.020
        return 1.3, 1.5, 1.8, 0.020

    # ================================================================ #
    # BREGUET HELPERS
    # ================================================================ #

    def _cruise_fraction(self, range_km: float, ld: float) -> float:
        """
        Weight fraction for one cruise leg at cruise_speed.
        Jet:  w = exp(-R * SFC / (V_kmh * L/D))
        Prop: w = exp(-R * SFC / (eta * V_kmh * L/D))
        """
        sfc   = self.specific_fuel
        v_kmh = self.cruise_speed * 3.6
        if self.engine_type == "Jet":
            return np.exp(-range_km * sfc / (v_kmh * ld))
        else:
            return np.exp(-range_km * sfc / (self.prop_efficiency * v_kmh * ld))

    def _loiter_fraction(self, endurance_hr: float, ld: float) -> float:
        """
        Weight fraction for one loiter segment at cruise_speed.
        Jet:  w = exp(-E * SFC / L/D)
        Prop: w = exp(-E * SFC / (eta * L/D))   (Raymer §3.5)
        """
        sfc = self.specific_fuel
        if self.engine_type == "Jet":
            return np.exp(-endurance_hr * sfc / ld)
        else:
            return np.exp(-endurance_hr * sfc / (self.prop_efficiency * ld))

    # ================================================================ #
    # THRUST / WING LOADING DIAGRAM
    # ================================================================ #

    def _make_wp_ws_diagram(self, plot: bool = False, save_path: str = None):
        CL_clean, CL_TO, CL_land, CD0 = self._wing_parameters()
        return WP_WS_Diagram(
            plot=plot,
            save_path=save_path,
            aspect_ratio=self.wing_aspect_ratio,
            CL_max_TO=CL_TO,
            CL_max_land=CL_land,
            CL_max_clean=CL_clean,
            cruise_speed=self.cruise_speed,
            CD0=CD0,
            prop_eff=self.prop_efficiency,
            engine_type=self.engine_type,
            oswald_factor=self.oswald_factor,
            n_max=self.maximum_load_factor,
            x_max=2000,
            y_max=1,
        )

    def thrust_and_wing_loading(self) -> tuple[float, float]:
        tw_ws = self._make_wp_ws_diagram(plot=False)
        W_S_design, y_design = tw_ws.find_design_point(rho=self.air_density)
        return W_S_design, y_design

    def thrust_and_wing_loading_plot(self):
        self._make_wp_ws_diagram(plot=True).find_design_point(rho=self.air_density)

    def save_wp_ws_figure(self, save_path: str):
        self._make_wp_ws_diagram(plot=False, save_path=save_path).find_design_point(
            rho=self.air_density
        )

    # ================================================================ #
    # LIFT-TO-DRAG
    # ================================================================ #

    def _lift_to_drag(self, wing_loading: float, CD0: float) -> float:
        """L/D at cruise_speed and given wing loading [N/m²]."""
        q = 0.5 * self.air_density * self.cruise_speed ** 2
        if q == 0.0 or wing_loading == 0.0:
            return 0.0
        return 1.0 / (
            (q * CD0) / wing_loading +
            wing_loading / (q * np.pi * self.wing_aspect_ratio * self.oswald_factor)
        )

    def ld_cruise(self) -> float:
        W_S, _ = self.thrust_and_wing_loading()
        _, _, _, CD0 = self._wing_parameters()
        return self._lift_to_drag(W_S, CD0)

    # ================================================================ #
    # FUEL FRACTIONS  (full combined mission)
    # ================================================================ #

    def _fuel_fracs(self) -> dict:
        """
        Full combined mission Breguet fractions at cruise_speed for all legs:
          taxi -> climb -> cruise -> loiter -> cruise back -> reserve -> land -> taxi

        L/D is evaluated at the actual wing loading for each leg.
        """
        _, _, _, CD0 = self._wing_parameters()
        W_S, _       = self.thrust_and_wing_loading()
        one_way_km   = self.mission_range / 2.0

        # taxi / start-up
        w1_w0 = 0.98

        # climb
        M     = self.cruise_speed / self.speed_of_sound
        w2_w1 = 1.0065 - 0.0325 * M

        wf_climb = w1_w0 * w2_w1

        # cruise there
        ld_cruise = self._lift_to_drag(W_S * wf_climb, CD0)
        w3_w2     = self._cruise_fraction(one_way_km, ld_cruise)

        # loiter
        wf_after_cruise = wf_climb * w3_w2
        ld_loiter       = self._lift_to_drag(W_S * wf_after_cruise, CD0)
        w4_w3           = self._loiter_fraction(self.mission_endurance, ld_loiter)

        # cruise back
        wf_after_loiter = wf_after_cruise * w4_w3
        ld_back         = self._lift_to_drag(W_S * wf_after_loiter, CD0)
        w5_w4           = self._cruise_fraction(one_way_km, ld_back)

        # reserve loiter
        wf_after_back = wf_after_loiter * w5_w4
        ld_reserve    = self._lift_to_drag(W_S * wf_after_back, CD0)
        w6_w5         = self._loiter_fraction(self.reserve_time, ld_reserve)

        # landing / taxi
        w7_w6 = 0.99
        w8_w7 = 0.992

        wf_w0 = w1_w0 * w2_w1 * w3_w2 * w4_w3 * w5_w4 * w6_w5 * w7_w6 * w8_w7

        # fuel burned per variable leg (as fraction of total)
        fuel_frac_cruise = (1.0 - w3_w2) + (1.0 - w5_w4)
        fuel_frac_loiter = 1.0 - w4_w3

        return {
            "wf_w0":            wf_w0,
            "w1_w0":            w1_w0,
            "w2_w1":            w2_w1,
            "w3_w2":            w3_w2,
            "w4_w3":            w4_w3,
            "w5_w4":            w5_w4,
            "w6_w5":            w6_w5,
            "w7_w6":            w7_w6,
            "w8_w7":            w8_w7,
            "ld_cruise":        ld_cruise,
            "ld_loiter":        ld_loiter,
            "ld_reserve":       ld_reserve,
            "fuel_frac_cruise": fuel_frac_cruise,
            "fuel_frac_loiter": fuel_frac_loiter,
        }

    # ================================================================ #
    # FUEL / WEIGHT SIZING
    # ================================================================ #

    def fuel_weight_sizing(self) -> tuple[float, float, float]:
        """
        Sizes the aircraft for the full combined mission.
        Returns (MTOW [kg], empty_weight [kg], fuel_weight [kg]).
        """
        fr    = self._fuel_fracs()
        wf_w0 = fr["wf_w0"]

        print(f"taxi:        {fr['w1_w0']:.4f}\n"
              f"climb:       {fr['w2_w1']:.4f}\n"
              f"cruise:      {fr['w3_w2']:.4f}\n"
              f"loiter:      {fr['w4_w3']:.4f}\n"
              f"cruise back: {fr['w5_w4']:.4f}\n"
              f"reserve:     {fr['w6_w5']:.4f}\n"
              f"landing:     {fr['w7_w6']:.4f}\n"
              f"taxi out:    {fr['w8_w7']:.4f}\n"
              f"wf_w0:       {wf_w0:.4f}\n")

        # Raymer empty-weight fraction coefficients
        if self.engine_type == "Jet":
            A, C_exp = 1.67, -0.16
        elif self.engine_type == "Turboprop":
            A, C_exp = 2.75, -0.18
        else:
            A, C_exp = 0.97, -0.06

        payload_lbs  = m2i.kilograms_to_pounds(self.payload_weight)

        # Initial guess from a nominal empty-weight fraction
        We_frac_init = 0.55 if self.engine_type == "Turboprop" else \
                       0.50 if self.engine_type == "Jet" else 0.60
        denom_init   = 1.0 - wf_w0 - We_frac_init
        if denom_init <= 0.0:
            print("Warning: mission infeasible at initial estimate — "
                  f"wf_w0={wf_w0:.3f} + We_frac_est={We_frac_init:.3f} >= 1")
            print(self.performance_margins_summary())
            return float("nan"), float("nan"), float("nan")

        W0_guess = payload_lbs / denom_init

        for i in range(100):
            if W0_guess <= 0.0:
                W0_guess = m2i.kilograms_to_pounds(100.0)
            We_frac = A * (W0_guess ** C_exp)
            denom   = 1.0 - wf_w0 - We_frac
            if denom <= 0.05:
                print("Warning: mission infeasible — "
                      f"wf_w0={wf_w0:.3f} + We_frac={We_frac:.3f} >= 1")
                print(self.performance_margins_summary())
                return float("nan"), float("nan"), float("nan")
            W0_lbs = payload_lbs / denom
            if abs(W0_lbs - W0_guess) / max(W0_guess, 1.0) < 0.01:
                print(f"Converged in {i + 1} iterations")
                break
            W0_guess = 0.5 * W0_guess + 0.5 * W0_lbs
        else:
            print("Warning: did not converge after 100 iterations")

        MTOW         = m2i.pounds_to_kilograms(W0_lbs)
        empty_weight = MTOW * We_frac
        fuel_weight  = MTOW * (1.0 - wf_w0) * 1.01

        print(self.performance_margins_summary())

        return MTOW, empty_weight, fuel_weight

    # ================================================================ #
    # PERFORMANCE MARGINS
    # ================================================================ #

    def performance_margins(self) -> dict:
        """
        From the full combined mission:

        1. L/D at cruise_speed for each leg — since the same speed is used
           throughout, differences reflect only the wing loading change as
           fuel burns off. The higher L/D leg is the aerodynamic design driver.

        2. Fuel fraction per leg — which of cruise or loiter consumed more
           fuel, i.e. which drives the fuel weight sizing.
        """
        fr = self._fuel_fracs()

        # Higher L/D = aerodynamic design driver (harder to achieve)
        ld_limiting_leg   = "cruise" if fr["ld_cruise"] >= fr["ld_loiter"] else "loiter"
        # More fuel burned = fuel weight driver
        fuel_dominant_leg = "cruise" if fr["fuel_frac_cruise"] >= fr["fuel_frac_loiter"] else "loiter"

        return {
            "ld_cruise":           fr["ld_cruise"],
            "ld_loiter":           fr["ld_loiter"],
            "ld_limiting_leg":     ld_limiting_leg,
            "ld_limiting_value":   max(fr["ld_cruise"], fr["ld_loiter"]),
            "fuel_frac_cruise":    fr["fuel_frac_cruise"],
            "fuel_frac_loiter":    fr["fuel_frac_loiter"],
            "fuel_dominant_leg":   fuel_dominant_leg,
            "wf_w0":               fr["wf_w0"],
            "specified_range_km":  self.mission_range,
            "specified_endurance_hr": self.mission_endurance,
        }

    def performance_margins_summary(self) -> str:
        m = self.performance_margins()
        lines = [
            "── Performance Margins ─────────────────────────────────────",
            "",
            "  ① Aerodynamic efficiency (L/D)  [cruise_speed used for all legs]",
            f"     Cruise  L/D : {m['ld_cruise']:.2f}",
            f"     Loiter  L/D : {m['ld_loiter']:.2f}",
            f"     Limiting leg (highest L/D, design driver) : "
            f"{m['ld_limiting_leg'].upper()}  (L/D = {m['ld_limiting_value']:.2f})",
            "",
            "  ② Fuel consumption per leg",
            f"     Cruise legs (both) : {m['fuel_frac_cruise']:.4f}",
            f"     Loiter leg         : {m['fuel_frac_loiter']:.4f}",
            f"     Fuel weight driver : {m['fuel_dominant_leg'].upper()}",
            "",
            f"  Total wf/w0 : {m['wf_w0']:.4f}",
            f"  Range       : {m['specified_range_km']:.0f} km",
            f"  Endurance   : {m['specified_endurance_hr']:.2f} hr",
            "────────────────────────────────────────────────────────────",
        ]
        return "\n".join(lines)


# ================================================================ #
# STAND-ALONE TEST
# ================================================================ #

if __name__ == "__main__":
    from Pythonfiles.Components.Mission.ISA_calculator import ISA_calculator

    def make_mission(altitude, range_km, endurance, payload,
                     sfc, mach_max, prop_eff, v_cruise,
                     objective, ar, engine):
        a   = ISA_calculator(altitude)[3]
        rho = ISA_calculator(altitude)[2]
        return Mission(
            mission_altitude=altitude,
            mission_range=range_km,
            mission_endurance=endurance,
            payload_weight=payload,
            specific_fuel=sfc,
            maximum_mach=mach_max,
            prop_efficiency=prop_eff,
            cruise_speed=v_cruise,
            mission_objective=objective,
            oswald_factor=0.8,
            reserve_time=0.5,
            maximum_load_factor=2.5,
            wing_aspect_ratio=ar,
            speed_of_sound=a,
            air_density=rho,
            engine_type=engine,
        )

    print("=== Turboprop 500 km / 8 hr ===")
    m1 = make_mission(6000, 500, 8, 100, 0.5, 0.5, 0.8, 80,
                      "High Endurance", 9.2, "Turboprop")
    MTOW, _, fw = m1.fuel_weight_sizing()
    print(f"MTOW={MTOW:.1f} kg  fuel={fw:.1f} kg\n")

    print("=== Turboprop 5000 km / 8 hr ===")
    m2 = make_mission(6000, 5000, 8, 100, 0.5, 0.5, 0.8, 80,
                      "High Endurance", 9.2, "Turboprop")
    MTOW, _, fw = m2.fuel_weight_sizing()
    print(f"MTOW={MTOW:.1f} kg  fuel={fw:.1f} kg\n")

    print("=== Jet 1000 km / 5 hr ===")
    m_jet = make_mission(8000, 1000, 5, 150, 0.8, 0.8, 1.0, 220,
                         "Long range", 7.5, "Jet")
    MTOW, _, fw = m_jet.fuel_weight_sizing()
    WS, TW = m_jet.thrust_and_wing_loading()
    print(f"MTOW={MTOW:.1f} kg  fuel={fw:.1f} kg  W/S={WS:.1f}  T/W={TW:.4f}\n")

    print("=== Piston 100 km / 5 hr ===")
    m_pi = make_mission(3000, 100, 5, 50, 0.4, 0.3, 0.8, 60,
                        "Low cost", 7.6, "Piston")
    MTOW, _, fw = m_pi.fuel_weight_sizing()
    WS, WP = m_pi.thrust_and_wing_loading()
    print(f"MTOW={MTOW:.1f} kg  fuel={fw:.1f} kg  W/S={WS:.1f}  W/P={WP:.4f}")