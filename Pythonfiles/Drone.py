"""
drone.py
========
Top-level drone class.
"""

import math
from typing import Optional

from parapy.core import Input, Attribute, Part, action
from parapy.geom import GeomBase


# ─── Input-range validator helpers ────────────────────────────────────────── #
# ParaPy's Input(validator=callable) expects a callable that returns True when
# the value is acceptable and raises ValueError (or returns False) otherwise.

def _between(lo: float, hi: float):
    """Closed-interval validator: lo ≤ value ≤ hi."""
    def _check(v):
        if not (lo <= v <= hi):
            raise ValueError(
                f"Value {v} is outside the feasible range [{lo}, {hi}]."
            )
        return True
    return _check


def _positive():
    """Strict positivity validator: value > 0."""
    def _check(v):
        if v <= 0:
            raise ValueError(f"Value must be positive (got {v}).")
        return True
    return _check


def _non_negative_int():
    """Non-negative integer validator."""
    def _check(v):
        if not isinstance(v, int) or v < 0:
            raise ValueError(f"Value must be a non-negative integer (got {v}).")
        return True
    return _check

from Pythonfiles.Components.Aircraft import Aircraft
from mission import Mission
from Pythonfiles.Components.Payload.Payload import Payload
from Pythonfiles.Components.Payload.Payloadrules import PayloadRules
from ISA_calculator import ISA_calculator

from Pythonfiles.metric_imperial_conversions import kilograms_to_pounds, feet_to_meters


class Drone(GeomBase):

    # ================================================================ #
    # REQUIRED MISSION INPUTS
    # Feasibility bounds are enforced by the validator= argument.
    # ParaPy raises ValueError immediately if an out-of-range value is set,
    # both at construction time and when the field is edited in the GUI.
    # ================================================================ #

    # Cruise true airspeed [m/s]
    # Lower bound 10 m/s (minimum controllable UAV speed).
    # Upper bound 350 m/s ≈ Mach 1.03 at sea level — beyond this the
    # Roskam subsonic drag polars and Breguet equations break down.
    cruise_speed: float = Input(validator=_between(10.0, 350.0))   # [m/s]

    # Mission altitude [m]
    # 0 m  = sea level  |  20 000 m = mid-stratosphere (SR-71 ceiling ≈ 26 km;
    # propulsion / atmospheric model is ISA troposphere + lower stratosphere).
    mission_altitude: float = Input(validator=_between(0.0, 20_000.0))  # [m]

    # Mission range [km]
    # 1 km minimum (prevents near-zero loiter-only missions blowing up W0
    # iteration).  25 000 km ≈ once-around-the-globe — extreme but bounded.
    mission_range: float = Input(validator=_between(1.0, 25_000.0))   # [km]

    # Mission endurance [hr]
    # 0.1 hr prevents division-by-zero in loiter fractions.
    # 120 hr ≈ 5-day record endurance (Global Hawk class).
    mission_endurance: float = Input(validator=_between(0.1, 120.0))  # [hr]

    # ================================================================ #
    # PAYLOAD INTENT
    # ================================================================ #

    payload_role: str = Input("ISR")

    # Weapon count [—]  0 = unarmed, 6 = maximum typical hard-point count.
    weapon_count: int = Input(0, validator=_between(0, 6))

    # ================================================================ #
    # ENGINEERING RULE OVERRIDES
    # ================================================================ #

    uav_class_override:          Optional[str] = Input(None)
    mission_objective_override:  Optional[str] = Input(None)

    # ================================================================ #
    # FUSELAGE LAYOUT
    # ================================================================ #

    # Cylinder start as % of total fuselage length.
    # Raymer §4.2: nosecone is typically 5–30 % of fuselage length.
    fuselage_cylinder_start: float = Input(
        10.0, validator=_between(5.0, 30.0))   # [% of fuselage length]

    # Cylinder end as % of total fuselage length.
    # Tail-cone must begin no later than 95 % of length.
    fuselage_cylinder_end: float = Input(
        70.0, validator=_between(50.0, 95.0))   # [% of fuselage length]

    # ================================================================ #
    # FUEL SYSTEM
    # ================================================================ #

    # Fuel type key from fuel_properties.json, or 'auto' to select
    # automatically from engine_type (Piston→Avgas, Turboprop/Jet→Jet-A).
    fuel_type: str = Input("auto")

    # Tank shape: length-to-diameter ratio.
    # AR=3 is a compact wing-box tank; increase to 5+ for slender HALE fuselages.
    fuel_tank_aspect_ratio: float = Input(3.0, validator=_between(1.1, 10.0))

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
    # WING PLANFORM — user-adjustable inputs
    # ================================================================ #

    # Taper ratio λ = c_tip / c_root.
    # Raymer §4.3: 0.20 (high-speed delta) to 1.0 (un-tapered / constant chord).
    # 0.40 is the Raymer subsonic endurance UAV default.
    wing_taper_ratio: float = Input(0.40, validator=_between(0.20, 1.0))

    # ================================================================ #
    # ENGINE TYPE
    # Altitude limits from Roskam Vol. I §3.2 / Raymer §10.2:
    #   Piston practical ceiling    ≈ 4 500 m  (15 000 ft)
    #   Turboprop practical ceiling ≈ 9 000 m  (30 000 ft)
    #   Above 9 000 m               → turbofan / turbojet required
    # ================================================================ #

    @Attribute
    def engine_type(self) -> str:
        """
        Engine type from Mach number and cruise altitude.

        Decision rule (Roskam Vol. I §3.2 / Raymer §10.2 / §13.3):
        ──────────────────────────────────────────────────────────────
        At sea level (h ≤ 9 000 m):
            M ≥ 0.40 → "Jet"      (compressibility drag makes propeller
                                    tip speeds unacceptable above M 0.40)
            High Endurance objective → "Turboprop"
            Otherwise → "Piston"

        At mid altitude (9 000 < h ≤ 15 000 m):
            Jet threshold raised to M ≥ 0.50.
            At these altitudes the jet thrust lapse factor is ~0.25–0.45
            (Raymer §13.3: T_alt/T_sl ≈ σ^0.75).  Below M 0.50 a
            turboprop is more efficient and avoids the extreme sea-level
            thrust rating a jet would need.  Predator B / Reaper operates
            in this band using a turboprop at M ≈ 0.35.

        At high altitude (h > 15 000 m):
            Jet threshold raised to M ≥ 0.60.
            At 20 km the lapse factor drops to ~0.12; a jet selected at
            M 0.40 would need a sea-level rated T/W ≈ 8× the altitude
            value — typically infeasible for a UAV.  Global Hawk uses a
            turbofan but only because it cruises at M 0.60+.
            Below M 0.60 at these altitudes a turboprop is selected.

        Piston ceiling: the engine sub-type (Piston vs Turboprop) for
        propeller missions is further limited to Piston only below
        4 500 m (Roskam Vol. I §3.2 practical piston ceiling).
        ──────────────────────────────────────────────────────────────
        """
        mach = self.cruise_speed / self.speed_of_sound
        h    = self.mission_altitude

        # Altitude-graduated Mach threshold for jet selection
        if h > 15_000.0:
            jet_mach_min = 0.60
        elif h > 9_000.0:
            jet_mach_min = 0.50
        else:
            jet_mach_min = 0.40

        if mach >= jet_mach_min:
            return "Jet"

        # Propeller branch — choose sub-type
        if self.payload_rules.mission_objective == "High Endurance":
            return "Turboprop"
        if h > 4_500.0:
            # Above practical piston ceiling → turboprop
            return "Turboprop"
        return "Piston"

    # ================================================================ #
    # WING ASPECT RATIO — two-stage calculation
    #
    # Stage 1: _wing_ar_roskam  — pure Roskam/Raymer empirical formula.
    #          Used by the Mission object for Breguet sizing (no fuselage
    #          feedback → no circular dependency).
    #
    # Stage 2: wing_aspect_ratio — geometry AR fed to Aircraft.
    #          Starts from _wing_ar_roskam, then increases it if the
    #          resulting root chord would exceed 40 % of the preliminary
    #          fuselage length (Raymer §4.2 rule).  A larger AR reduces
    #          chord without changing wing area or span.
    # ================================================================ #

    @Attribute
    def _wing_ar_roskam(self) -> float:
        """
        Wing AR from Roskam/Raymer empirical formula [—].

        Used exclusively by the Mission sizing object so that the mission
        loop does not depend on any fuselage geometry (which itself depends
        on the mission MTOW result).

        References
        ----------
        Raymer §4.4 (Eq. 4.6) — subsonic jet UAV:
            AR = 4.737 * M_cruise^{-0.979},  clamped to [6, 12]
        Roskam Vol. I Table 3.5 — turboprop MALE:  AR ≈ 9.2
        Roskam Vol. I Table 3.5 — piston GA:       AR ≈ 7.6
        """
        if self.engine_type == "Jet":
            mach_cruise = self.cruise_speed / self.speed_of_sound
            ar = 4.737 * mach_cruise ** -0.979
            return max(6.0, min(ar, 12.0))   # Raymer §4.4 practical bounds
        if self.engine_type == "Turboprop":
            return 9.2
        return 7.6

    @Attribute
    def wing_aspect_ratio(self) -> float:
        """
        Wing AR for geometry [—] — Roskam value adjusted upward when
        the root chord would be too large relative to the fuselage.

        Logic
        -----
        1.  Start with AR_roskam  (from _wing_ar_roskam).
        2.  Compute root chord:
                c_root = 2·S / (sqrt(AR·S) · (1 + λ))
        3.  Compare against Raymer §4.2 limit:
                c_root_max = 0.40 · L_fus_payload   (or Roskam if no payload)
        4.  If c_root > c_root_max, solve for the minimum AR that
            satisfies the constraint:
                AR_min = 4·S / (c_root_max · (1 + λ))²
            Capped at AR = 16 (structural realism; Raymer Fig. 4.10).

        Reference length is payload.min_fuselage_length / cylinder_fraction
        when a payload exists — this is altitude-independent, so AR increases
        correctly at high altitude (thin air → large wing → slender chord).
        Falls back to Roskam estimate when no payload is defined.

        Reference: Raymer §4.2 — "root chord should not exceed ~40 % of
        fuselage length to limit interference drag and aeroelastic coupling."
        """
        ar     = self._wing_ar_roskam
        S      = self.wing_area          # m² — from mission (uses _wing_ar_roskam)
        taper  = self.wing_taper_ratio   # user input

        b      = math.sqrt(ar * S)
        c_root = 2.0 * S / (b * (1.0 + taper))

        # c_root_max: maximum root chord that fits the fuselage cylinder.
        # Use the payload-driven fuselage length (altitude-independent) when a
        # payload is defined, otherwise fall back to the Roskam estimate.
        # Raymer §4.2: root chord ≤ 40% of fuselage length limits interference
        # drag and aeroelastic coupling at the wing-fuselage junction.
        #
        # Why NOT _roskam_fuselage_length_estimate here:
        # That estimate scales with MTOW (which rises with altitude / fuel load),
        # so c_root_max would grow at altitude and the AR would never increase —
        # leaving the wing with a large, inefficient chord.  The payload length
        # is fixed regardless of altitude, giving a stable AR reference.
        if self.payload is not None:
            cylinder_fraction = (self.fuselage_cylinder_end - self.fuselage_cylinder_start) / 100.0
            payload_fus_length = self.payload.min_fuselage_length / cylinder_fraction
            c_root_max = 0.40 * payload_fus_length
        else:
            c_root_max = 0.40 * self._roskam_fuselage_length_estimate

        if c_root <= c_root_max:
            return ar

        # Minimum AR to bring c_root within the fuselage length limit:
        #   c_root_max = 2S / (sqrt(AR·S) · (1+λ))
        #   → AR = 4S / (c_root_max · (1+λ))²
        ar_min  = 4.0 * S / (c_root_max * (1.0 + taper)) ** 2
        ar_geom = min(ar_min, 16.0)   # structural upper bound (Raymer Fig. 4.10)

        print(
            f"[Wing AR] Roskam AR={ar:.2f} → c_root={c_root:.3f} m  "
            f"(limit {c_root_max:.3f} m) → geometry AR adjusted to {ar_geom:.2f}"
        )
        return ar_geom

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
            # Use the pure Roskam AR here — avoids circular dependency:
            # wing_aspect_ratio (geometry) depends on wing_area → mission,
            # so passing it back into mission would create a cycle.
            # _wing_ar_roskam is independent of fuselage geometry.
            wing_aspect_ratio=self._wing_ar_roskam,
            speed_of_sound=self.speed_of_sound,
            air_density=self.air_density,
            engine_type=self.engine_type,
        )

    # ================================================================ #
    # ACTIONS
    # ================================================================ #

    @action(label="Show Design Point")
    def WP_WS_diagram(self):
        import os, datetime
        self.mission.thrust_and_wing_loading_plot()
        save_dir  = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        png_path  = os.path.join(save_dir, f"Outputfiles/design_point_{timestamp}.png")
        try:
            self.mission.save_wp_ws_figure(png_path)
            print(f"✓ Design-point diagram saved: {png_path}")
        except Exception as exc:
            print(f"Design-point PNG save failed: {exc}")

    @action(label="Run Wing Airfoil Sweep")
    def run_wing_sweep(self):
        self.aircraft.main_wing.run_sweep()

    @action(label="Plot Wing XFoil polars")
    def plot_wing_cl_alpha(self):
        import os, datetime
        self.aircraft.main_wing.plot_cl_alpha()
        save_dir  = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        png_path  = os.path.join(save_dir, f"Outputfiles/wing_polars_{timestamp}.png")
        try:
            self.aircraft.main_wing.root_airfoil.save_polar_figure(png_path)
            print(f"✓ Wing polar figure saved: {png_path}")
        except Exception as exc:
            print(f"Wing polars PNG save failed: {exc}")

    @action(label="Print Stability Report")
    def print_stability_report(self):
        self.aircraft.print_stability_report()

    # ================================================================ #
    # EXPORT ACTIONS
    # ================================================================ #

    @action(label="Export PDF Report")
    def export_pdf_report(self):
        """
        Write a design-summary PDF to the same folder as Drone.py.

        The report contains:
          • Mission parameters
          • Weight breakdown (MTOW / empty / fuel / payload)
          • Wing & aerodynamic sizing
          • Engine info (type, thrust/power loading)
          • Longitudinal stability summary
          • The W/P – W/S design-point diagram (saved as a temporary PNG
            and embedded in the PDF)
        """
        import os
        import datetime
        import tempfile

        # ── locate output directory ─────────────────────────────────── #
        save_dir  = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        pdf_path  = os.path.join(save_dir, f"Outputfiles/drone_report_{timestamp}.pdf")

        try:
            from reportlab.lib.pagesizes import A4
            from reportlab.lib.units    import cm
            from reportlab.lib.styles  import getSampleStyleSheet, ParagraphStyle
            from reportlab.lib          import colors
            from reportlab.platypus     import (
                SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle,
                HRFlowable, Image as RLImage,
            )

            doc    = SimpleDocTemplate(pdf_path, pagesize=A4,
                                       leftMargin=2*cm, rightMargin=2*cm,
                                       topMargin=2*cm, bottomMargin=2*cm)
            styles = getSampleStyleSheet()

            # ── custom styles ────────────────────────────────────────── #
            title_style = ParagraphStyle(
                "Title2", parent=styles["Title"],
                fontSize=18, spaceAfter=6, textColor=colors.HexColor("#003366"),
            )
            h1_style = ParagraphStyle(
                "H1", parent=styles["Heading1"],
                fontSize=13, spaceAfter=4, textColor=colors.HexColor("#003366"),
            )
            body_style = styles["Normal"]

            def section(title):
                return [
                    Spacer(1, 0.3*cm),
                    Paragraph(title, h1_style),
                    HRFlowable(width="100%", thickness=1,
                               color=colors.HexColor("#003366"), spaceAfter=4),
                ]

            def data_table(rows, col_widths=None):
                """Two-column label / value table with alternating row shading."""
                if col_widths is None:
                    col_widths = [9*cm, 8*cm]
                tbl = Table(rows, colWidths=col_widths)
                style = TableStyle([
                    ("BACKGROUND",   (0, 0), (-1, 0),  colors.HexColor("#003366")),
                    ("TEXTCOLOR",    (0, 0), (-1, 0),  colors.white),
                    ("FONTNAME",     (0, 0), (-1, 0),  "Helvetica-Bold"),
                    ("FONTSIZE",     (0, 0), (-1, 0),  10),
                    ("ALIGN",        (1, 1), (1, -1),  "RIGHT"),
                    ("FONTSIZE",     (0, 1), (-1, -1), 9),
                    ("ROWBACKGROUNDS", (0, 1), (-1, -1),
                     [colors.HexColor("#EEF3FA"), colors.white]),
                    ("GRID",         (0, 0), (-1, -1), 0.4, colors.grey),
                    ("TOPPADDING",   (0, 0), (-1, -1), 3),
                    ("BOTTOMPADDING",(0, 0), (-1, -1), 3),
                    ("LEFTPADDING",  (0, 0), (-1, -1), 6),
                    ("RIGHTPADDING", (0, 0), (-1, -1), 6),
                ])
                tbl.setStyle(style)
                return tbl

            # ── collect data (trigger lazy attributes once) ─────────── #
            mtow        = self.MTOW
            empty_wt    = self.empty_weight
            fuel_wt     = self.fuel_weight
            payload_wt  = self.payload_weight
            wing_area   = self.wing_area
            wing_span   = self.wing_semi_span * 2.0
            wing_ar     = self.wing_aspect_ratio
            wing_ld     = self.wing_loading
            ld          = self.ld_cruise
            eng_type    = self.engine_type
            cg_x        = self.cg_x
            np_x        = self.neutral_point_x
            sm          = self.static_margin
            stab        = self.stability_status
            fus_len     = self.aircraft.fuselage.length
            fus_rad     = self.aircraft.fuselage.radius

            # ── build W/P–W/S diagram PNG ────────────────────────────── #
            diagram_path = None
            try:
                tmp = tempfile.NamedTemporaryFile(
                    suffix=".png", delete=False, dir=save_dir)
                tmp.close()
                self.mission.save_wp_ws_figure(tmp.name)
                diagram_path = tmp.name
            except Exception as _de:
                print(f"[PDF] diagram embed skipped: {_de}")

            # ── assemble flowables ───────────────────────────────────── #
            story = []

            # Title
            story.append(Paragraph("UAV Initial Sizing — Design Report", title_style))
            story.append(Paragraph(
                f"Generated: {datetime.datetime.now().strftime('%d %b %Y  %H:%M')}",
                body_style,
            ))
            story.append(Spacer(1, 0.4*cm))

            # 1. Mission parameters
            story += section("1 · Mission Parameters")
            story.append(data_table([
                ["Parameter",              "Value"],
                ["Cruise speed",           f"{self.cruise_speed:.1f} m/s  "
                                           f"({self.cruise_speed * 3.6:.0f} km/h)"],
                ["Mission altitude",       f"{self.mission_altitude:.0f} m"],
                ["Mission range",          f"{self.mission_range:.0f} km"],
                ["Mission endurance",      f"{self.mission_endurance:.1f} hr"],
                ["Payload role",           self.payload_role],
                ["Mission objective",      self.mission_objective],
                ["UAV class",              self.uav_class],
                ["Max. load factor",       f"{self.maximum_load_factor:.2f} g"],
                ["Engine type",            eng_type],
            ]))

            # 2. Weights
            story += section("2 · Weight Budget")
            story.append(data_table([
                ["Component",              "Mass [kg]"],
                ["MTOW",                   f"{mtow:.1f}"],
                ["Empty weight",           f"{empty_wt:.1f}"],
                ["Fuel weight",            f"{fuel_wt:.1f}"],
                ["Payload weight",         f"{payload_wt:.1f}"],
                ["Fuel fraction  Wf/W0",   f"{fuel_wt / mtow:.3f}"],
                ["Empty fraction We/W0",   f"{empty_wt / mtow:.3f}"],
            ]))

            # 3. Wing & aero
            story += section("3 · Wing & Aerodynamic Sizing")
            thr_row = (
                ["Thrust loading  T/W",    f"{self.thrust_loading:.3f}"]
                if eng_type == "Jet"
                else ["Power loading  W/P [kg/W]",
                      f"{self.power_loading:.5f}  "
                      f"({1.0/self.power_loading:.1f} W/kg)"]
            )
            story.append(data_table([
                ["Parameter",              "Value"],
                ["Wing area  S",           f"{wing_area:.2f} m²"],
                ["Wing span  b",           f"{wing_span:.2f} m"],
                ["Aspect ratio  AR",       f"{wing_ar:.2f}"],
                ["Wing loading  W/S",      f"{wing_ld:.1f} N/m²"],
                ["Cruise L/D",             f"{ld:.2f}"],
                thr_row,
            ]))

            # 4. Fuselage
            story += section("4 · Fuselage")
            story.append(data_table([
                ["Parameter",              "Value"],
                ["Fuselage length",        f"{fus_len:.2f} m"],
                ["Fuselage radius",        f"{fus_rad:.3f} m"],
                ["Cylinder start",         f"{self.fuselage_cylinder_start:.1f} %"],
                ["Cylinder end",           f"{self.fuselage_cylinder_end:.1f} %"],
            ]))

            # 5. Stability
            story += section("5 · Longitudinal Stability")
            story.append(data_table([
                ["Parameter",              "Value"],
                ["CG position  (from nose)", f"{cg_x:.3f} m"],
                ["Neutral point (from nose)", f"{np_x:.3f} m"],
                ["Static margin  SM",      f"{sm*100:.1f} % MAC"],
                ["Assessment",             stab],
            ]))

            # 6. Diagram
            if diagram_path and os.path.exists(diagram_path):
                story += section("6 · Design Point — W/P vs W/S Diagram")
                story.append(RLImage(diagram_path, width=14*cm, height=8*cm))

            # ── build PDF ────────────────────────────────────────────── #
            doc.build(story)
            print(f"✓ PDF report saved: {pdf_path}")

            # clean up temp diagram
            if diagram_path and os.path.exists(diagram_path):
                try:
                    os.remove(diagram_path)
                except OSError:
                    pass

        except Exception as exc:
            import traceback
            print(f"PDF export failed: {exc}")
            traceback.print_exc()

    # ================================================================ #

    @action(label="Export STP File")
    def export_stp_file(self):
        import os
        import datetime

        save_dir  = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        stp_path  = os.path.join(save_dir, f"Outputfiles/drone_geometry_{timestamp}.stp")

        try:
            from OCC.wrapper.STEPControl import STEPControl_Writer, STEPControl_AsIs
            from OCC.wrapper.IFSelect    import IFSelect_RetDone
            from OCC.wrapper.BRep        import BRep_Builder
            from OCC.wrapper.TopoDS      import TopoDS_Compound
            from parapy.geom             import GeomBase

            builder  = BRep_Builder()
            compound = TopoDS_Compound()
            builder.MakeCompound(compound)
            shapes_added = [0]

            def _add_shape(obj):
                # Try TopoDS_Shape first (ParaPy 1.15), fall back to .shape
                for attr in ("TopoDS_Shape", "shape"):
                    try:
                        s = getattr(obj, attr, None)
                        if s is not None and not s.IsNull():
                            builder.Add(compound, s)
                            shapes_added[0] += 1
                            return True
                    except Exception:
                        pass
                return False

            def _collect(obj):
                if not isinstance(obj, GeomBase):
                    return
                added = _add_shape(obj)
                if not added:
                    # recurse into children
                    try:
                        for child in obj.children:
                            if child is not obj:
                                _collect(child)
                    except Exception:
                        pass

            _collect(self.aircraft)

            if shapes_added[0] == 0:
                print("STP export: no shapes found — geometry may not be "
                    "built yet.  Trigger geometry by opening the 3-D view "
                    "first, then re-run the export.")
                return

            writer = STEPControl_Writer()
            writer.Transfer(compound, STEPControl_AsIs)
            status = writer.Write(stp_path)

            if status == IFSelect_RetDone:
                print(f"✓ STP file saved ({shapes_added[0]} shapes): {stp_path}")
            else:
                print(f"STP writer returned status {status} for: {stp_path}")

        except ImportError as e:
            print(f"OCC import failed: {e}")
        except Exception as exc:
            import traceback
            print(f"STP export failed: {exc}")
            traceback.print_exc()

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
            thrust_to_weight=self.thrust_loading if self.engine_type == "Jet" else self.power_loading,
            rho=self.air_density,
            # Pass taper ratio so Aircraft/LiftingSurface and the chord-constraint
            # check in Drone both use the same value.
            wing_taper_ratio=self.wing_taper_ratio,
            payload_object=self.payload,
            fuselage_cylinder_start=self.fuselage_cylinder_start,
            fuselage_cylinder_end=self.fuselage_cylinder_end,
            # Fuel system — mass drives tank sizing, type selects density
            fuel_mass=self.fuel_weight,
            fuel_tank_type=self.fuel_type,
            fuel_tank_aspect_ratio=self.fuel_tank_aspect_ratio,
            engine_type_str=self.engine_type,
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
        """Longitudinal static margin as fraction of MAC [-]."""
        return self.aircraft.static_margin

    @Attribute
    def stability_status(self) -> str:
        """Plain-English stability assessment."""
        return self.aircraft.stability_status
