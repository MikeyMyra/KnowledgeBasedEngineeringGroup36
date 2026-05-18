"""
mission.py — Breguet-based mission analysis and UAV initial sizing.

Estimates MTOW, fuel weight, empty weight, thrust-to-weight, and wing loading
from range, endurance, and payload inputs using Breguet equations and Roskam
empty-weight fraction regressions. Raises a dialog when the mission is infeasible.
"""
import numpy as np
import Pythonfiles.metric_imperial_conversions as m2i
from Pythonfiles.Components.Mission.WP_WS_diagram import WP_WS_Diagram


# ─── Infeasibility warning dialog (matches XFoil Mach pattern) ────────────── #

def _show_infeasible_dialog(title: str, message: str) -> None:
    """
    Show a modal tkinter warning dialog, then destroy the root window.
    """
    try:
        import tkinter as tk
        from tkinter import messagebox
        _root = tk.Tk()
        _root.withdraw()
        messagebox.showwarning(title, message)
        _root.destroy()
    except Exception:
        pass  # headless environment — console print is the fallback


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

        # Raymer empty-weight fraction coefficients (Table 3.1).
        if self.engine_type == "Jet":
            A, C_exp = 1.67, -0.16
            We_frac_max = 0.50   # jet UAV (Global Hawk ≈ 0.47, capped at 0.50)
        elif self.engine_type == "Turboprop":
            A, C_exp = 2.75, -0.18
            We_frac_max = 0.52   # turboprop UAV (Predator A ≈ 0.50, capped at 0.52)
        else:
            A, C_exp = 0.97, -0.06
            We_frac_max = 0.58   # piston UAV (typical small UAV ≈ 0.52–0.58)

        payload_lbs  = m2i.kilograms_to_pounds(self.payload_weight)

        # Fuel fraction of take-off weight (Raymer §3.5 — trapped fuel adds 1%)
        fuel_frac = (1.0 - wf_w0) * 1.01

        # ── absolute feasibility: fuel alone must fit in the aircraft ────── #
        if fuel_frac >= 1.0:
            msg = (
                f"Mission is physically infeasible.\n\n"
                f"The required fuel fraction is {fuel_frac:.3f} ≥ 1.0, meaning\n"
                f"the aircraft would need to be made entirely of fuel.\n\n"
                f"Current inputs:\n"
                f"  Range      : {self.mission_range:.0f} km\n"
                f"  Endurance  : {self.mission_endurance:.2f} hr\n"
                f"  Altitude   : {self.mission_altitude:.0f} m\n\n"
                f"Suggested fixes — reduce one or more of:\n"
                f"  • Mission range\n"
                f"  • Mission endurance\n"
                f"  • Cruise speed  (lowers fuel burn)\n"
                f"  • Mission altitude  (denser air → better L/D)"
            )
            print("Warning: mission physically infeasible — "
                  f"fuel_frac={fuel_frac:.3f} >= 1.0  (wf_w0={wf_w0:.3f})\n"
                  f"  (Try reducing range, endurance, or SFC.)")
            print(self.performance_margins_summary())
            _show_infeasible_dialog("Mission Infeasible — Fuel Fraction ≥ 1", msg)
            return float("nan"), float("nan"), float("nan")

        def _f(W0_lbs_: float) -> float:
            we = min(A * (W0_lbs_ ** C_exp), We_frac_max)
            return W0_lbs_ * (1.0 - fuel_frac - we) - payload_lbs

        _MTOW_MAX_KG  = 100000.0   # [kg] — hard UAV scope limit
        hi_max_lbs    = m2i.kilograms_to_pounds(_MTOW_MAX_KG)

        denom_at_cap = 1.0 - fuel_frac - min(A * hi_max_lbs ** C_exp, We_frac_max)
        if _f(hi_max_lbs) <= 0.0:
            # Identify the dominant driver so the dialog can name it explicitly
            pm = self.performance_margins()
            driver = pm["fuel_dominant_leg"]
            if driver == "cruise":
                driver_detail = (
                    f"Range ({self.mission_range:.0f} km) is the dominant fuel driver.\n"
                    f"  → Try reducing mission_range first."
                )
            else:
                driver_detail = (
                    f"Endurance ({self.mission_endurance:.2f} hr) is the dominant fuel driver.\n"
                    f"  → Try reducing mission_endurance first."
                )

            msg = (
                f"Mission requires MTOW > {_MTOW_MAX_KG / 1000:.0f} t — outside the\n"
                f"scope of this UAV sizing tool.\n\n"
                f"Current inputs:\n"
                f"  Range      : {self.mission_range:.0f} km\n"
                f"  Endurance  : {self.mission_endurance:.2f} hr\n"
                f"  Altitude   : {self.mission_altitude:.0f} m\n"
                f"  Cruise speed: {self.cruise_speed:.1f} m/s\n\n"
                f"Sizing diagnosis:\n"
                f"  fuel_frac = {fuel_frac:.4f}   We_frac_max = {We_frac_max:.3f}\n"
                f"  {driver_detail}\n\n"
                f"Suggested fixes — reduce one or more of:\n"
                f"  • Mission range\n"
                f"  • Mission endurance\n"
                f"  • Cruise speed  (lower speed → better L/D → less fuel)\n"
                f"  • Mission altitude  (denser air → better L/D at same speed)"
            )
            print(
                f"\n[Mission] INFEASIBLE — mission requires MTOW > {_MTOW_MAX_KG/1000:.0f} t,\n"
                f"  which is outside the scope of this UAV sizing tool.\n"
                f"  fuel_frac = {fuel_frac:.3f}   We_frac_max = {We_frac_max:.3f}"
                f"   denom @ cap = {denom_at_cap:.4f}\n"
                f"  Root cause: very low L/D from high cruise speed causes excessive fuel burn.\n"
                f"  Fixes: ↓ cruise speed  |  ↓ range  |  ↓ endurance  |  ↑ cruise altitude"
            )
            print(self.performance_margins_summary())
            _show_infeasible_dialog(
                f"Mission Infeasible — MTOW > {_MTOW_MAX_KG / 1000:.0f} t", msg
            )
            return float("nan"), float("nan"), float("nan")

        lo, hi = payload_lbs * 0.5, min(payload_lbs * 200.0, hi_max_lbs)
        for _ in range(40):
            if _f(hi) > 0:
                break
            hi = min(hi * 5.0, hi_max_lbs)
        else:
            # Should not reach here (we verified f(hi_max) > 0 above).
            print("[Mission] Bracket search failed unexpectedly — "
                  "mission likely infeasible.")
            return float("nan"), float("nan"), float("nan")

        if _f(lo) > 0:
            # Both ends positive → payload is very small; shrink lo
            lo = payload_lbs * 0.01

        # Bisection
        for i in range(60):
            mid = 0.5 * (lo + hi)
            if _f(mid) < 0:
                lo = mid
            else:
                hi = mid
            if (hi - lo) / max(hi, 1.0) < 1e-4:
                break

        W0_lbs  = 0.5 * (lo + hi)
        We_frac = min(A * (W0_lbs ** C_exp), We_frac_max)
        denom   = 1.0 - fuel_frac - We_frac
        print(f"Converged in {i + 1} bisection steps  "
              f"(We_frac={We_frac:.3f}, payload_frac={denom:.3f})")

        if denom < 0.02:
            print(f"Warning: payload fraction is very small ({denom:.1%}) — "
                  f"mission is technically feasible but very demanding.\n"
                  f"  Consider reducing range/endurance or payload weight.")

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

        1. L/D at cruise_speed for each leg

        2. Fuel fraction per leg
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
     