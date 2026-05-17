from parapy.core import Base, Input, Attribute
from Pythonfiles.Components.Mission.ISA_calculator import ISA_calculator
import numpy as np
import metric_imperial_conversions as m2i
from Pythonfiles.Components.Mission.WP_WS_diagram import WP_WS_Diagram


class Mission(Base):

    # ================================================================ #
    # MISSION INPUTS
    # ================================================================ #

    mission_altitude:    float = Input()
    mission_range:       float = Input()
    mission_endurance:   float = Input()
    payload_weight:      float = Input()
    specific_fuel:       float = Input(0.5)    # [1/hr]  SFC
    maximum_mach:        float = Input()
    prop_efficiency:     float = Input(0.8)
    cruise_speed:        float = Input()
    loiter_speed:        float = Input()
    mission_objective:   str   = Input()
    oswald_factor:       float = Input(0.8)
    reserve_time:        float = Input(0.5)
    maximum_load_factor: float = Input()

    # ================================================================ #
    # PRE-COMPUTED INPUTS FROM DRONE
    # These are derived at the Drone level (single source of truth) and
    # passed in, so Mission never needs to call ISA_calculator itself.
    # ================================================================ #

    wing_aspect_ratio: float = Input()   # Raymer AR, computed by Drone
    speed_of_sound:    float = Input()   # ISA a at mission altitude [m/s]
    air_density:       float = Input()   # ISA ρ at mission altitude [kg/m³]
    engine_type:       str   = Input()   # "Jet" | "Turboprop" | "Piston"

    # ================================================================ #
    # WING PARAMETERS  (set as a group; called once before sizing)
    # ================================================================ #

    def _wing_parameters(self):
        """
        Returns (CL_max_clean, CL_max_TO, CL_max_land, CD0) for the
        current engine type.  Pure function — no side effects.
        """
        if self.engine_type == "Jet":
            return 1.2, 1.5, 1.7, 0.015
        if self.engine_type == "Turboprop":
            return 1.4, 1.7, 2.0, 0.020
        # Piston
        return 1.3, 1.5, 1.8, 0.020

    # ================================================================ #
    # THRUST / WING LOADING DIAGRAM
    # ================================================================ #

    def _make_wp_ws_diagram(self, plot: bool = False, save_path: str = None):
        """Construct a WP_WS_Diagram for the current mission parameters."""
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

    @Attribute
    def thrust_and_wing_loading(self) -> tuple[float, float]:
        tw_ws = self._make_wp_ws_diagram(plot=False)
        W_S_design, y_design = tw_ws.find_design_point(rho=self.air_density)
        return W_S_design, y_design

    def thrust_and_wing_loading_plot(self):
        """Show the W/P–W/S diagram exactly once. Called by the Drone action."""
        self._make_wp_ws_diagram(plot=True).find_design_point(rho=self.air_density)

    def save_wp_ws_figure(self, save_path: str):
        """Save the W/P–W/S diagram to a PNG file without displaying it."""
        self._make_wp_ws_diagram(plot=False, save_path=save_path).find_design_point(
            rho=self.air_density
        )

    # ================================================================ #
    # FUEL / WEIGHT SIZING
    # ================================================================ #

    @Attribute
    def fuel_weight_sizing(self) -> tuple[float, float, float]:
        """
        Breguet-based mission profile:
          taxi → takeoff → climb → cruise → loiter →
          cruise back → reserve → descent → landing → taxi
        Returns (MTOW [kg], empty_weight [kg], fuel_weight [kg]).
        """
        CL_clean, CL_TO, CL_land, CD0 = self._wing_parameters()

        specific_fuel  = self.specific_fuel
        cruise_range   = self.mission_range / 2
        loiter_time    = self.mission_endurance

        W_S_design, y_design = self.thrust_and_wing_loading
        wing_loading = W_S_design

        # start-up / taxi
        w1_w0 = 0.98

        # climb
        optimal_cruise_speed = self.cruise_speed
        if self.engine_type == "Jet":
            for _ in range(10):
                old = optimal_cruise_speed
                M = optimal_cruise_speed / self.speed_of_sound
                w2_w1 = 1.0065 - 0.0325 * M
                optimal_cruise_speed = self._find_optimal_cruise_speed(
                    wing_loading * w1_w0 * w2_w1, CD0
                )
                if abs(optimal_cruise_speed - old) < 0.1:
                    break
            cruise_speed = optimal_cruise_speed
        else:
            M = self.cruise_speed / self.speed_of_sound
            w2_w1 = 1.0065 - 0.0325 * M
            cruise_speed = self.cruise_speed

        # cruise to loiter
        wf_cruise = w1_w0 * w2_w1
        ld_cruise = self._lift_to_drag(wing_loading * wf_cruise, CD0, cruise_speed)
        if self.engine_type == "Jet":
            w3_w2 = np.exp((-cruise_range * specific_fuel) /
                           (cruise_speed * ld_cruise))
        else:
            w3_w2 = np.exp((-cruise_range * specific_fuel) /
                           (cruise_speed * self.prop_efficiency * ld_cruise))

        # loiter
        wf_loiter = wf_cruise * w3_w2
        loiter_speed = self._find_optimal_loiter_speed(wing_loading * wf_loiter, CD0)
        ld_loiter = self._lift_to_drag(wing_loading * wf_loiter, CD0, loiter_speed)
        if self.engine_type == "Jet":
            w4_w3 = np.exp(-loiter_time * (specific_fuel / 3600) / ld_loiter)
        else:
            w4_w3 = np.exp(-loiter_time * loiter_speed * (specific_fuel / 3600) /
                           (self.prop_efficiency * ld_loiter))

        # cruise back
        wf_back = wf_loiter * w4_w3
        ld_back = self._lift_to_drag(wing_loading * wf_back, CD0, cruise_speed)
        if self.engine_type == "Jet":
            w5_w4 = np.exp((-cruise_range * specific_fuel) /
                           (cruise_speed * ld_back))
        else:
            w5_w4 = np.exp((-cruise_range * specific_fuel) /
                           (cruise_speed * self.prop_efficiency * ld_back))

        # reserve
        wf_reserve = wf_back * w5_w4
        ld_reserve = self._lift_to_drag(wing_loading * wf_reserve, CD0, loiter_speed)
        if self.engine_type == "Jet":
            w6_w5 = np.exp(-self.reserve_time * (specific_fuel / 3600) / ld_reserve)
        else:
            w6_w5 = np.exp(-self.reserve_time * loiter_speed * (specific_fuel / 3600) /
                           (self.prop_efficiency * ld_reserve))

        # landing / taxi
        w7_w6 = 0.99
        w8_w7 = 0.992

        wf_w0 = (w1_w0 * w2_w1 * w3_w2 * w4_w3 *
                 w5_w4 * w6_w5 * w7_w6 * w8_w7)

        print(f"taxi:        {w1_w0:.4f}\n"
              f"climb:       {w2_w1:.4f}\n"
              f"cruise:      {w3_w2:.4f}\n"
              f"loiter:      {w4_w3:.4f}\n"
              f"cruise back: {w5_w4:.4f}\n"
              f"reserve:     {w6_w5:.4f}\n"
              f"landing:     {w7_w6:.4f}\n"
              f"taxi out:    {w8_w7:.4f}\n")

        # Raymer empty-weight fraction coefficients
        if self.engine_type == "Jet":
            A, C_exp = 1.67, -0.16
        elif self.engine_type == "Turboprop":
            A, C_exp = 2.75, -0.18
        else:  # Piston
            A, C_exp = 0.97, -0.06

        payload_lbs  = m2i.kilograms_to_pounds(self.payload_weight)
        W0_guess     = m2i.kilograms_to_pounds(
            600 if self.engine_type == "Jet" else
            400 if self.engine_type == "Turboprop" else 100
        )

        for i in range(100):
            We_frac  = A * (W0_guess ** C_exp)
            W0_lbs   = payload_lbs / (1 - wf_w0 - We_frac)
            if abs(W0_lbs - W0_guess) / W0_guess < 0.01:
                print(f"Converged in {i + 1} iterations")
                break
            W0_guess = 0.5 * W0_guess + 0.5 * W0_lbs
        else:
            print("Warning: did not converge after 100 iterations")

        MTOW         = m2i.pounds_to_kilograms(W0_lbs)
        empty_weight = MTOW * We_frac
        fuel_weight  = MTOW * (1.0 - wf_w0) * 1.01

        print(self.performance_margins_summary)

        return MTOW, empty_weight, fuel_weight

    # ================================================================ #
    # CRUISE L/D  (called by Drone after sizing)
    # ================================================================ #

    @Attribute
    def ld_cruise(self) -> float:
        """L/D at cruise, available after fuel_weight_sizing has run."""
        _, _, _ = self.fuel_weight_sizing          # ensure sizing ran
        W_S, _ = self.thrust_and_wing_loading
        _, _, _, CD0 = self._wing_parameters()
        return self._lift_to_drag(W_S, CD0, self.cruise_speed)

    # ================================================================ #
    # PRIVATE HELPERS
    # ================================================================ #

    def _lift_to_drag(self, wing_loading: float, CD0: float, speed: float) -> float:
        """Raymer ch. 6 L/D from dynamic pressure, CD0, and induced drag."""
        q = 0.5 * self.air_density * speed ** 2
        return 1.0 / (
            (q * CD0) / wing_loading +
            wing_loading / (q * np.pi * self.wing_aspect_ratio * self.oswald_factor)
        )

    def _find_optimal_cruise_speed(self, weight_fraction: float, CD0: float) -> float:
        """Speed giving maximum cruise fuel fraction over a coarse sweep."""
        v_range = np.arange(1, 400, 10)
        fracs = [
            np.exp((-self.mission_range / 2 * self.specific_fuel) /
                   (v * self._lift_to_drag(self.thrust_and_wing_loading[0] * weight_fraction,
                                           CD0, v)))
            for v in v_range
        ]
        return float(v_range[int(np.argmax(fracs))])

    def _find_optimal_loiter_speed(self, weight_fraction: float, CD0: float) -> float:
        """Speed giving maximum loiter fuel fraction over a coarse sweep."""
        v_range = np.arange(1, 400, 10)
        W_S = self.thrust_and_wing_loading[0]

        if self.engine_type == "Jet":
            fracs = [
                np.exp(-self.mission_endurance * (self.specific_fuel / 3600) /
                       self._lift_to_drag(W_S * weight_fraction, CD0, v))
                for v in v_range
            ]
        else:
            fracs = [
                np.exp(-self.mission_endurance * v * (self.specific_fuel / 3600) /
                       (self.prop_efficiency *
                        self._lift_to_drag(W_S * weight_fraction, CD0, v)))
                for v in v_range
            ]
        return float(v_range[int(np.argmax(fracs))])

    def _fuel_fracs_parametric(self, cruise_range_km: float,
                                loiter_time_hr: float) -> dict:
        """
        Compute Breguet weight fractions for arbitrary (range, endurance) values
        using the same wing-loading design point as the main sizing.

        Parameters
        ----------
        cruise_range_km  : total mission range [km]  (0 → no cruise legs)
        loiter_time_hr   : loiter / endurance [hr]   (0 → no loiter)

        Returns
        -------
        dict with keys
            wf_w0        – overall remaining-fuel fraction (1 = no fuel burned)
            w_fixed      – taxi + climb + landing fractions combined
            w_cruise     – w3_w2 * w5_w4  (both cruise legs)
            w_loiter     – w4_w3
            w_reserve    – w6_w5
            loiter_speed – optimal loiter speed used [m/s]
            cruise_speed – cruise speed used [m/s]
            ld_cruise    – L/D at cruise
            ld_loiter    – L/D at loiter
        """
        _, _, _, CD0 = self._wing_parameters()
        sfc           = self.specific_fuel
        one_way_km    = cruise_range_km / 2.0

        W_S, _ = self.thrust_and_wing_loading

        # ── taxi / start-up ──────────────────────────────────────────── #
        w1_w0 = 0.98

        # ── climb ────────────────────────────────────────────────────── #
        if self.engine_type == "Jet":
            v_cruise = self.cruise_speed
            for _ in range(10):
                v_old = v_cruise
                M     = v_cruise / self.speed_of_sound
                w2_w1 = 1.0065 - 0.0325 * M
                v_cruise = self._find_optimal_cruise_speed(
                    W_S * w1_w0 * w2_w1, CD0)
                if abs(v_cruise - v_old) < 0.1:
                    break
        else:
            M     = self.cruise_speed / self.speed_of_sound
            w2_w1 = 1.0065 - 0.0325 * M
            v_cruise = self.cruise_speed

        # ── cruise there ─────────────────────────────────────────────── #
        wf_cr  = w1_w0 * w2_w1
        ld_cr  = self._lift_to_drag(W_S * wf_cr, CD0, v_cruise)
        if cruise_range_km > 0.0:
            if self.engine_type == "Jet":
                w3_w2 = np.exp((-one_way_km * sfc) / (v_cruise * ld_cr))
            else:
                w3_w2 = np.exp((-one_way_km * sfc) /
                               (v_cruise * self.prop_efficiency * ld_cr))
        else:
            w3_w2 = 1.0

        # ── loiter ───────────────────────────────────────────────────── #
        wf_ltr    = wf_cr * w3_w2
        v_loiter  = self._find_optimal_loiter_speed(W_S * wf_ltr, CD0)
        ld_ltr    = self._lift_to_drag(W_S * wf_ltr, CD0, v_loiter)
        if loiter_time_hr > 0.0:
            if self.engine_type == "Jet":
                w4_w3 = np.exp(-loiter_time_hr * (sfc / 3600) / ld_ltr)
            else:
                w4_w3 = np.exp(-loiter_time_hr * v_loiter * (sfc / 3600) /
                               (self.prop_efficiency * ld_ltr))
        else:
            w4_w3 = 1.0

        # ── cruise back ──────────────────────────────────────────────── #
        wf_back = wf_ltr * w4_w3
        ld_back = self._lift_to_drag(W_S * wf_back, CD0, v_cruise)
        if cruise_range_km > 0.0:
            if self.engine_type == "Jet":
                w5_w4 = np.exp((-one_way_km * sfc) / (v_cruise * ld_back))
            else:
                w5_w4 = np.exp((-one_way_km * sfc) /
                               (v_cruise * self.prop_efficiency * ld_back))
        else:
            w5_w4 = 1.0

        # ── reserve ──────────────────────────────────────────────────── #
        wf_rsv = wf_back * w5_w4
        ld_rsv = self._lift_to_drag(W_S * wf_rsv, CD0, v_loiter)
        if self.engine_type == "Jet":
            w6_w5 = np.exp(-self.reserve_time * (sfc / 3600) / ld_rsv)
        else:
            w6_w5 = np.exp(-self.reserve_time * v_loiter * (sfc / 3600) /
                           (self.prop_efficiency * ld_rsv))

        # ── landing / taxi ────────────────────────────────────────────── #
        w7_w6 = 0.99
        w8_w7 = 0.992

        wf_w0 = (w1_w0 * w2_w1 * w3_w2 * w4_w3 *
                 w5_w4 * w6_w5 * w7_w6 * w8_w7)

        return {
            "wf_w0":        wf_w0,
            "w_fixed":      w1_w0 * w2_w1 * w7_w6 * w8_w7,
            "w_cruise":     w3_w2 * w5_w4,
            "w_loiter":     w4_w3,
            "w_reserve":    w6_w5,
            "loiter_speed": v_loiter,
            "cruise_speed": v_cruise,
            "ld_cruise":    ld_cr,
            "ld_loiter":    ld_ltr,
        }

    # ── Breguet inversions ──────────────────────────────────────────── #

    def _endurance_from_fraction(self, w_loiter: float,
                                  v_loiter: float, ld: float) -> float:
        """Return endurance [hr] given a loiter weight fraction."""
        if w_loiter <= 0.0 or w_loiter >= 1.0 or ld <= 0.0 or v_loiter <= 0.0:
            return 0.0
        sfc = self.specific_fuel
        if self.engine_type == "Jet":
            result = -np.log(w_loiter) * ld * 3600 / sfc
        else:
            result = (-np.log(w_loiter) * self.prop_efficiency * ld
                      / (v_loiter * sfc / 3600))
        return float(result) if np.isfinite(result) and result >= 0.0 else 0.0

    def _range_from_fraction(self, w_cruise: float,
                              v_cruise: float, ld: float) -> float:
        """
        Return total mission range [km] given combined cruise weight fraction
        (w3_w2 * w5_w4).  Assumes symmetric outbound / return legs.
        """
        if w_cruise <= 0.0 or w_cruise >= 1.0 or ld <= 0.0:
            return 0.0
        sfc = self.specific_fuel
        if self.engine_type == "Jet":
            result = -np.log(w_cruise) * v_cruise * ld / sfc
        else:
            result = -np.log(w_cruise) * v_cruise * self.prop_efficiency * ld / sfc
        return float(result) if np.isfinite(result) and result >= 0.0 else 0.0

    # ================================================================ #
    # PERFORMANCE MARGINS
    # ================================================================ #

    @Attribute
    def performance_margins(self) -> dict:
        """
        Determine which of range or endurance is the primary sizing driver
        and compute the achievable capability of the non-limiting metric.

        Method
        ------
        1. Compute fuel fraction for *range only* (endurance → 0).
        2. Compute fuel fraction for *endurance only* (range → 0).
        3. The case with the lower wf_w0 (more fuel burned) is limiting.
        4. For the limiting case, invert Breguet to find how far the
           non-limiting metric could be pushed if the other requirement
           were removed entirely.

        Returns
        -------
        dict with keys
            limiting              – "range" | "endurance"
            achievable_endurance_hr – float  (populated when range limits)
            achievable_range_km     – float  (populated when endurance limits)
            specified_range_km    – float
            specified_endurance_hr – float
            surplus_endurance_hr  – float  (achievable − specified, ≥ 0)
            surplus_range_km      – float  (achievable − specified, ≥ 0)
        """
        # -- what-if: size for range only (zero loiter) --
        fr_range = self._fuel_fracs_parametric(self.mission_range, 0.0)

        # -- what-if: size for endurance only (zero range) --
        fr_end   = self._fuel_fracs_parametric(0.0, self.mission_endurance)

        # lower wf_w0 → more fuel consumed → that requirement is limiting
        range_is_limiting = fr_range["wf_w0"] <= fr_end["wf_w0"]

        # ── Compute L/D and speeds at a stable nominal weight fraction ── #
        # Using the degenerate post-long-cruise weight gives ld → 0, which
        # causes inf × 0 = NaN in the Breguet inversion.  Instead we
        # evaluate at wf = 0.90 (representative early-mission weight) so
        # the L/D is always well-conditioned regardless of mission length.
        _, _, _, CD0 = self._wing_parameters()
        W_S, _  = self.thrust_and_wing_loading
        nom_wf  = 0.90   # representative mission weight fraction

        v_ltr_nom = self._find_optimal_loiter_speed(W_S * nom_wf, CD0)
        ld_ltr_nom = self._lift_to_drag(W_S * nom_wf, CD0, v_ltr_nom)
        ld_cr_nom  = self._lift_to_drag(W_S * nom_wf, CD0, self.cruise_speed)

        if range_is_limiting:
            # given the range-only fuel budget, how long could we loiter
            # (if we skipped all cruise legs)?
            w_ltr_achievable = (fr_range["wf_w0"]
                                / (fr_range["w_fixed"] * fr_range["w_reserve"]))
            achievable_end   = self._endurance_from_fraction(
                w_ltr_achievable, v_ltr_nom, ld_ltr_nom)
            achievable_rng   = self.mission_range   # by construction
        else:
            # given the endurance-only fuel budget, how far could we cruise
            # (if we skipped loiter)?
            w_cr_achievable  = (fr_end["wf_w0"]
                                / (fr_end["w_fixed"] * fr_end["w_reserve"]))
            achievable_rng   = self._range_from_fraction(
                w_cr_achievable, self.cruise_speed, ld_cr_nom)
            achievable_end   = self.mission_endurance   # by construction

        return {
            "limiting":               "range" if range_is_limiting else "endurance",
            "achievable_endurance_hr": achievable_end,
            "achievable_range_km":     achievable_rng,
            "specified_range_km":      self.mission_range,
            "specified_endurance_hr":  self.mission_endurance,
            "surplus_endurance_hr":    max(0.0, achievable_end - self.mission_endurance),
            "surplus_range_km":        max(0.0, achievable_rng - self.mission_range),
        }

    @Attribute
    def performance_margins_summary(self) -> str:
        """Human-readable summary of which requirement limits and by how much."""
        m   = self.performance_margins
        lim = m["limiting"]

        lines = [
            "── Performance Margins ─────────────────────────────────────",
            f"  Sizing driver : {lim.upper()}",
        ]

        if lim == "range":
            lines += [
                f"  Required range     : {m['specified_range_km']:.0f} km  (limiting)",
                f"  Required endurance : {m['specified_endurance_hr']:.2f} hr",
                f"  Achievable endurance (if no cruise) : "
                f"{m['achievable_endurance_hr']:.2f} hr  "
                f"(+{m['surplus_endurance_hr']:.2f} hr surplus)",
            ]
        else:
            lines += [
                f"  Required endurance : {m['specified_endurance_hr']:.2f} hr  (limiting)",
                f"  Required range     : {m['specified_range_km']:.0f} km",
                f"  Achievable range (if no loiter) : "
                f"{m['achievable_range_km']:.0f} km  "
                f"(+{m['surplus_range_km']:.0f} km surplus)",
            ]

        lines.append("────────────────────────────────────────────────────────────")
        return "\n".join(lines)


# ================================================================ #
# STAND-ALONE TEST
# ================================================================ #

if __name__ == "__main__":
    from Pythonfiles.Components.Mission.ISA_calculator import ISA_calculator

    def make_mission(altitude, range_km, endurance, payload,
                     sfc, mach_max, prop_eff, v_cruise, objective,
                     ar, engine):
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
            loiter_speed=0.7 * v_cruise,
            mission_objective=objective,
            oswald_factor=0.8,
            reserve_time=0.5,
            maximum_load_factor=2.5,
            wing_aspect_ratio=ar,
            speed_of_sound=a,
            air_density=rho,
            engine_type=engine,
        )

    # Turboprop MALE
    m_tp = make_mission(6000, 10, 8, 100, 0.5, 0.5, 0.8, 80,
                        "High Endurance", 9.2, "Turboprop")
    MTOW, _, fw = m_tp.fuel_weight_sizing
    WS, WP = m_tp.thrust_and_wing_loading
    print(f"[Turboprop] MTOW={MTOW:.1f} kg  fuel={fw:.1f} kg  W/S={WS:.1f}  W/P={WP:.4f}")

    # Jet
    m_jet = make_mission(8000, 1000, 5, 150, 0.8, 0.8, 1.0, 220,
                         "Long range", 7.5, "Jet")
    MTOW, _, fw = m_jet.fuel_weight_sizing
    WS, TW = m_jet.thrust_and_wing_loading
    print(f"[Jet]       MTOW={MTOW:.1f} kg  fuel={fw:.1f} kg  W/S={WS:.1f}  T/W={TW:.4f}")

    # Piston
    m_pi = make_mission(3000, 10, 10, 50, 0.4, 0.3, 0.8, 60,
                        "Low cost", 7.6, "Piston")
    MTOW, _, fw = m_pi.fuel_weight_sizing
    WS, WP = m_pi.thrust_and_wing_loading
    print(f"[Piston]    MTOW={MTOW:.1f} kg  fuel={fw:.1f} kg  W/S={WS:.1f}  W/P={WP:.4f}")