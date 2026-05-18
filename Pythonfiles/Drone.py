"""
drone.py
========
Top-level drone class.
"""

import math
import os
import glob
import shutil
import datetime
from typing import Optional

from parapy.core import Input, Attribute, Part, action
from parapy.geom import GeomBase

from Pythonfiles.Components.Mission.vn_diagram import plot_vn_diagram


# ─── Input-range validator helpers ────────────────────────────────────────── #

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


# ─── Output file archiving helper ─────────────────────────────────────────── #

def _archive_previous(output_dir: str, pattern: str) -> None:
    """
    Move any existing files in *output_dir* that match *pattern* into a
    ``data/`` sub-folder, preserving their original filenames.

    Parameters
    ----------
    output_dir : str
        Absolute path to the ``Outputfiles`` directory.
    pattern : str
        Glob pattern relative to *output_dir*, e.g. ``"design_point_*.png"``.

    Example
    -------
    Before saving ``design_point_20260517_120000.png`` call::

        _archive_previous(save_dir, "design_point_*.png")

    Any previously generated ``design_point_*.png`` files sitting in
    ``Outputfiles/`` are moved to ``Outputfiles/data/`` so the root folder
    always contains only the most-recent file.
    """
    archive_dir = os.path.join(output_dir, "data")
    os.makedirs(archive_dir, exist_ok=True)

    for existing in glob.glob(os.path.join(output_dir, pattern)):
        dest = os.path.join(archive_dir, os.path.basename(existing))
        # If a file with the same name already exists in the archive,
        # overwrite it (identical timestamp → same run, safe to replace).
        shutil.move(existing, dest)


from Pythonfiles.Components.Aircraft import Aircraft
from Pythonfiles.Components.Mission.mission import Mission
from Pythonfiles.Components.Payload.Payload import Payload
from Pythonfiles.Components.Payload.Payloadrules import PayloadRules, PayloadRole, ROLE_CATEGORIES
from Pythonfiles.Components.Mission.ISA_calculator import ISA_calculator

from Pythonfiles.metric_imperial_conversions import kilograms_to_pounds, feet_to_meters


class Drone(GeomBase):

    # ================================================================ #
    # REQUIRED MISSION INPUTS
    # ================================================================ #

    cruise_speed: float = Input(
        validator=_between(10.0, 350.0),
        doc="Cruise true airspeed  [m/s]  ·  valid: 10 – 350 m/s\n"
            "Engine type is inferred from Mach number and altitude:\n"
            "  M ≥ 0.40 (sea level) / 0.50 (9 km) / 0.60 (15 km) → Jet\n"
            "  Below threshold, h > 4 500 m → Turboprop, else → Piston",
    )   # [m/s]

    mission_altitude: float = Input(
        validator=_between(0.0, 15_000.0),
        doc="Cruise / loiter altitude  [m]  ·  valid: 0 – 15 000 m\n"
            "Practical ceilings by engine type:\n"
            "  Piston   ≤ 4 500 m  |  Turboprop ≤ 9 000 m  |  Jet up to 15 000 m\n"
            "Above 9 000 m a jet is selected automatically if Mach threshold is met.",
    )  # [m]

    mission_range: float = Input(
        validator=_between(1.0, 25_000.0),
        doc="Total mission range (outbound + return)  [km]  ·  valid: 1 – 25 000 km\n"
            "Breguet fuel sizing: each cruise leg = range / 2.\n"
            "Class floors: < 150 km → small  |  150 – 500 km → medium  |  > 500 km → large",
    )   # [km]

    mission_endurance: float = Input(
        validator=_between(0.1, 120.0),
        doc="Loiter / endurance duration  [hr]  ·  valid: 0.1 – 120 hr\n"
            "Class floors: < 4 hr → small  |  4 – 10 hr → medium  |  > 10 hr → large\n"
            "Objective: > 6 hr → High Endurance (Turboprop preferred)",
    )  # [hr]

    # ================================================================ #
    # PAYLOAD INTENT
    # ================================================================ #

    payload_role: PayloadRole = Input(
        PayloadRole.ISR,
        doc="Mission role — selects the payload sensor/weapon suite automatically.\n"
            "ISR: EO/IR + radar + datalink\n"
            "Strike: EO/IR + weapons + datalink\n"
            "SEAD: EO/IR + radar + weapons + datalink\n"
            "Mapping: EO/IR + LiDAR\n"
            "COMMS relay: comms + datalink\n"
            "Patrol: EO/IR + comms",
    )

    weapon_count: int = Input(
        0,
        validator=_between(0, 6),
        doc="Number of munitions / hard-points  [—]  ·  valid: 0 – 6\n"
            "0 = unarmed (weapon category suppressed from payload).\n"
            "Requires payload_role = 'Strike' or 'SEAD' to carry weapons.",
    )

    # ================================================================ #
    # ENGINEERING RULE OVERRIDES
    # ================================================================ #

    uav_class_override:          Optional[str] = Input(None)
    mission_objective_override:  Optional[str] = Input(None)

    # ================================================================ #
    # FUSELAGE LAYOUT
    # ================================================================ #

    fuselage_cylinder_start: float = Input(
        10.0,
        validator=_between(5.0, 30.0),
        doc="Nosecone / cylinder junction  [% of fuselage length]  ·  valid: 5 – 30 %\n"
            "Raymer §4.2: nosecone typically 5–30 % of fuselage length.\n"
            "Payload bay begins just aft of this station.",
    )   # [% of fuselage length]

    fuselage_cylinder_end: float = Input(
        70.0,
        validator=_between(50.0, 95.0),
        doc="Cylinder / tail-cone junction  [% of fuselage length]  ·  valid: 50 – 95 %\n"
            "Tail-cone begins at this station. Fuel tank fits between payload and this point.\n"
            "Keep at least 5 % gap above cylinder_start.",
    )   # [% of fuselage length]

    payload_nose_clearance: float = Input(
        0.20,
        validator=_between(0.0, 2.0),
        doc="Gap from nose-cone end to first payload item  [m]  ·  valid: 0 – 2 m\n"
            "Prevents the payload from overlapping the prop nacelle on tractor configs.\n"
            "100 mm default clears most small/medium UAV nacelles.  Increase to 0.3–0.5 m\n"
            "for large turboprop tractor installations.",
    )   # [m]

    # ================================================================ #
    # FUEL SYSTEM
    # ================================================================ #

    fuel_type: str = Input("auto")

    fuel_tank_aspect_ratio: float = Input(
        3.0,
        validator=_between(1.1, 10.0),
        doc="Fuel tank length-to-diameter ratio  [—]  ·  valid: 1.1 – 10.0\n"
            "3 = compact wing-box tank (default).\n"
            "Increase to 5+ for slender HALE fuselages where tank must fit in a narrow bay.",
    )

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
    # WING PLANFORM
    # ================================================================ #

    wing_taper_ratio: float = Input(
        0.40,
        validator=_between(0.20, 1.0),
        doc="Wing taper ratio  λ = c_tip / c_root  [—]  ·  valid: 0.20 – 1.00\n"
            "0.40 — Raymer subsonic endurance UAV default.\n"
            "Lower values increase tip wash-out and reduce induced drag; "
            "values below 0.20 cause structural/manufacturing difficulties.",
    )

    # ================================================================ #
    # ENGINE TYPE
    # ================================================================ #

    @Attribute
    def engine_type(self) -> str:
        """
        Engine type from Mach number and cruise altitude.

        Decision rule (Roskam Vol. I §3.2 / Raymer §10.2 / §13.3):
        ──────────────────────────────────────────────────────────────
        At sea level (h ≤ 9 000 m):
            M ≥ 0.40 → "Jet"
            High Endurance objective → "Turboprop"
            Otherwise → "Piston"

        At mid altitude (9 000 < h ≤ 15 000 m):
            Jet threshold raised to M ≥ 0.50.

        At high altitude (h > 15 000 m):
            Jet threshold raised to M ≥ 0.60.

        Piston ceiling: limited to below 4 500 m (Roskam Vol. I §3.2).
        ──────────────────────────────────────────────────────────────
        """
        mach = self.cruise_speed / self.speed_of_sound
        h    = self.mission_altitude

        if h > 15_000.0:
            jet_mach_min = 0.60
        elif h > 9_000.0:
            jet_mach_min = 0.50
        else:
            jet_mach_min = 0.40

        if mach >= jet_mach_min:
            return "Jet"

        if self.payload_rules.mission_objective == "High Endurance":
            return "Turboprop"
        if h > 4_500.0:
            return "Turboprop"
        return "Piston"

    # ================================================================ #
    # WING ASPECT RATIO
    # ================================================================ #

    @Attribute
    def _wing_ar_roskam(self) -> float:
        """
        Wing AR from Roskam/Raymer empirical formula [—].

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
            return max(6.0, min(ar, 12.0))
        if self.engine_type == "Turboprop":
            return 9.2
        return 7.6

    @Attribute
    def wing_aspect_ratio(self) -> float:
        """
        Wing AR for geometry [—] — Roskam value adjusted upward when
        the root chord would be too large relative to the fuselage.
        """
        ar     = self._wing_ar_roskam
        S      = self.wing_area
        taper  = self.wing_taper_ratio

        b      = math.sqrt(ar * S)
        c_root = 2.0 * S / (b * (1.0 + taper))

        if self.payload is not None:
            cylinder_fraction = (self.fuselage_cylinder_end - self.fuselage_cylinder_start) / 100.0
            payload_fus_length = self.payload.min_fuselage_length / cylinder_fraction
            c_root_max = 0.40 * payload_fus_length
        else:
            c_root_max = 0.40 * self._roskam_fuselage_length_estimate

        if c_root <= c_root_max:
            return ar

        ar_min  = 4.0 * S / (c_root_max * (1.0 + taper)) ** 2
        ar_geom = min(ar_min, 16.0)

        L_fus_est    = self._roskam_fuselage_length_estimate
        span_max     = 4.0 * L_fus_est
        ar_span_cap  = span_max ** 2 / S
        if ar_geom > ar_span_cap and ar_span_cap > ar:
            print(
                f"[Wing AR] span cap: AR {ar_geom:.2f} → {ar_span_cap:.2f}  "
                f"(semi-span would be {math.sqrt(ar_geom * S)/2:.1f} m vs "
                f"fuselage {L_fus_est:.1f} m)"
            )
            ar_geom = max(ar_span_cap, ar)

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
    # ================================================================ #

    @Attribute
    def _roskam_fuselage_length_estimate(self) -> float:
        """
        Quick Roskam length estimate [m].

        Roskam Vol. I Table 3.4: L = 0.23 * MTOW^0.5
        """
        mtow_lbs  = kilograms_to_pounds(self.MTOW)
        length_ft = 0.23 * (mtow_lbs ** 0.50)
        return feet_to_meters(length_ft)

    @Attribute
    def _fuel_tank_length_estimate(self) -> float:
        """
        Fuel tank total length estimate [m] — pure math, no geometry.

        Mirrors Aircraft._fuel_tank_sizing so that Drone.payload_start_x can
        decide whether payload fits before or after the tank without depending
        on the Aircraft Part (which would be circular).
        """
        import math
        from Pythonfiles.Components.Fuel.FuelTank import FUELS, AUTO_SELECTION, _VOLUME_FACTOR
        if math.isnan(self.fuel_weight) or self.fuel_weight <= 0.0:
            return 0.0
        ft       = self.fuel_type if self.fuel_type != "auto" else AUTO_SELECTION.get(self.engine_type, "jet_a")
        density  = FUELS[ft]["density_kg_m3"]
        ar       = max(self.fuel_tank_aspect_ratio, 1.01)
        fuel_vol = self.fuel_weight / density
        tank_vol = fuel_vol * _VOLUME_FACTOR
        r3       = tank_vol / (math.pi * (2.0 * ar - 2.0 / 3.0))
        R        = r3 ** (1.0 / 3.0)
        return 2.0 * R * ar   # total capsule length

    @Attribute
    def payload_start_x(self) -> float:
        """
        X-position where the first payload item begins [m].

        Always placed from the nose: cylinder_start + nose_clearance offset.
        If this puts the payload inside the fuel tank the fuselage sizing in
        Aircraft._min_fuselage_length_for_payload_tank_clearance will force
        the fuselage (and therefore the wing/tank station) to grow until the
        tank clears the payload end.
        """
        L_est            = self._roskam_fuselage_length_estimate
        cylinder_start_x = (self.fuselage_cylinder_start / 100.0) * L_est
        return cylinder_start_x + self.payload_nose_clearance * L_est

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

    @property
    def mission(self) -> Mission:
        m = Mission(
            mission_altitude=self.mission_altitude,
            mission_range=self.mission_range,
            mission_endurance=self.mission_endurance,
            payload_weight=self.payload_weight,
            maximum_mach=self.maximum_mach,
            cruise_speed=self.cruise_speed,
            loiter_speed=self.loiter_speed_seed,
            mission_objective=self.mission_objective,
            maximum_load_factor=self.maximum_load_factor,
            wing_aspect_ratio=self._wing_ar_roskam,
            speed_of_sound=self.speed_of_sound,
            air_density=self.air_density,
            engine_type=self.engine_type,
        )
        m.performance_margins_summary()
        return m

    # ================================================================ #
    # ACTIONS
    # ================================================================ #

    @action(label="Show Design Point")
    def WP_WS_diagram(self):
        self.mission.thrust_and_wing_loading_plot()
        save_dir  = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        png_path  = os.path.join(save_dir, f"Outputfiles/design_point_{timestamp}.png")

        _archive_previous(os.path.join(save_dir, "Outputfiles"), "design_point_*.png")

        try:
            self.mission.save_wp_ws_figure(png_path)
            print(f"✓ Design-point diagram saved: {png_path}")
        except Exception as exc:
            print(f"Design-point PNG save failed: {exc}")

    @action(label="Show V-n Diagram")
    def vn_diagram(self):

        save_dir  = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        png_path  = os.path.join(save_dir, f"Outputfiles/vn-diagram_{timestamp}.png")

        _archive_previous(os.path.join(save_dir, "Outputfiles"), "vn-diagram*.png")

        try:
            plot_vn_diagram(
                MTOW            = self.MTOW,
                wing_area       = self.wing_area,
                cruise_speed    = self.cruise_speed,
                cruise_altitude = self.mission_altitude,
                n_pos           = self.maximum_load_factor,
                output_dir      = os.path.join(save_dir, "Outputfiles"),
            )
            print(f"✓ Design-point diagram saved: {png_path}")
        except Exception as exc:
            print(f"Design-point PNG save failed: {exc}")


    @action(label="Run Wing Airfoil Sweep")
    def run_wing_sweep(self):
        self.aircraft.main_wing.run_sweep()

    @action(label="Plot Wing XFoil polars")
    def plot_wing_cl_alpha(self):
        self.aircraft.main_wing.plot_cl_alpha()
        save_dir  = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        png_path  = os.path.join(save_dir, f"Outputfiles/wing_polars_{timestamp}.png")

        _archive_previous(os.path.join(save_dir, "Outputfiles"), "wing_polars_*.png")

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
        Write a comprehensive design-summary PDF to the Outputfiles folder.
        Any previously generated drone_report_*.pdf is moved to
        Outputfiles/data/ before the new file is written.
        """
        import tempfile
        import math as _math

        from Pythonfiles.Components.Mission.ISA_calculator import ISA_calculator

        save_dir  = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        pdf_path  = os.path.join(save_dir, f"Outputfiles/drone_report_{timestamp}.pdf")

        _archive_previous(os.path.join(save_dir, "Outputfiles"), "drone_report_*.pdf")

        try:
            from reportlab.lib.pagesizes import A4
            from reportlab.lib.units    import cm
            from reportlab.lib.styles  import getSampleStyleSheet, ParagraphStyle
            from reportlab.lib          import colors
            from reportlab.platypus     import (
                SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle,
                HRFlowable, Image as RLImage, PageBreak,
            )

            doc    = SimpleDocTemplate(pdf_path, pagesize=A4,
                                       leftMargin=2*cm, rightMargin=2*cm,
                                       topMargin=2*cm, bottomMargin=2*cm)
            styles = getSampleStyleSheet()

            title_style = ParagraphStyle(
                "Title2", parent=styles["Title"],
                fontSize=18, spaceAfter=6, textColor=colors.HexColor("#003366"),
            )
            h1_style = ParagraphStyle(
                "H1", parent=styles["Heading1"],
                fontSize=13, spaceAfter=4, textColor=colors.HexColor("#003366"),
            )
            body_style = styles["Normal"]
            note_style = ParagraphStyle(
                "Note", parent=styles["Normal"],
                fontSize=8, textColor=colors.grey,
            )

            def section(title):
                return [
                    Spacer(1, 0.3*cm),
                    Paragraph(title, h1_style),
                    HRFlowable(width="100%", thickness=1,
                               color=colors.HexColor("#003366"), spaceAfter=4),
                ]

            def data_table(rows, col_widths=None):
                if col_widths is None:
                    col_widths = [9*cm, 8*cm]
                tbl = Table(rows, colWidths=col_widths)
                tbl_style = TableStyle([
                    ("BACKGROUND",   (0, 0), (-1, 0),  colors.HexColor("#003366")),
                    ("TEXTCOLOR",    (0, 0), (-1, 0),  colors.white),
                    ("FONTNAME",     (0, 0), (-1, 0),  "Helvetica-Bold"),
                    ("FONTSIZE",     (0, 0), (-1, 0),  10),
                    ("ALIGN",        (1, 1), (-1, -1), "RIGHT"),
                    ("FONTSIZE",     (0, 1), (-1, -1), 9),
                    ("ROWBACKGROUNDS", (0, 1), (-1, -1),
                     [colors.HexColor("#EEF3FA"), colors.white]),
                    ("GRID",         (0, 0), (-1, -1), 0.4, colors.grey),
                    ("TOPPADDING",   (0, 0), (-1, -1), 3),
                    ("BOTTOMPADDING",(0, 0), (-1, -1), 3),
                    ("LEFTPADDING",  (0, 0), (-1, -1), 6),
                    ("RIGHTPADDING", (0, 0), (-1, -1), 6),
                ])
                tbl.setStyle(tbl_style)
                return tbl

            # ── Gather all data ────────────────────────────────────────── #

            # Basic weights
            mtow       = self.MTOW
            empty_wt   = self.empty_weight
            fuel_wt    = self.fuel_weight
            payload_wt = self.payload_weight
            eng_type   = self.engine_type

            # Wing
            wing_area  = self.wing_area
            wing_span  = self.wing_semi_span * 2.0
            wing_ar    = self.wing_aspect_ratio
            wing_ld    = self.wing_loading
            ld         = self.ld_cruise

            # Stability
            cg_x  = self.cg_x
            np_x  = self.neutral_point_x
            sm    = self.static_margin
            stab  = self.stability_status
            fus_len = self.aircraft.fuselage.length
            fus_rad = self.aircraft.fuselage.radius

            # ISA atmosphere at cruise altitude
            T_isa, p_isa, rho_isa, a_isa, _ = ISA_calculator(self.mission_altitude)

            # Mach number
            mach_cruise = self.mach

            # Performance margins and Breguet fractions
            pm  = self.performance_margins
            fr  = self.mission._fuel_fracs()
            sfc = self.mission.specific_fuel      # [1/hr]
            eta = self.mission.prop_efficiency

            # Achievable endurance or range (whichever is the non-driver)
            driver = pm['fuel_dominant_leg']
            e_achievable  = None
            r_achievable  = None
            e_margin_pct  = None
            r_margin_pct  = None
            if driver == 'cruise':
                frac    = pm['fuel_frac_cruise']
                w4_eq   = max(1.0 - frac, 1e-6)
                if eng_type == 'Jet':
                    e_achievable = -_math.log(w4_eq) * pm['ld_loiter'] / sfc
                else:
                    e_achievable = -_math.log(w4_eq) * pm['ld_loiter'] * eta / sfc
                e_margin_pct = (e_achievable - self.mission_endurance) / self.mission_endurance * 100.0
            else:
                frac    = pm['fuel_frac_loiter']
                w3_eq   = max(1.0 - frac, 1e-6)
                v_kmh   = self.cruise_speed * 3.6
                if eng_type == 'Jet':
                    r_achievable = -_math.log(w3_eq) * v_kmh * pm['ld_cruise'] / sfc
                else:
                    r_achievable = -_math.log(w3_eq) * eta * v_kmh * pm['ld_cruise'] / sfc
                r_margin_pct = (r_achievable - self.mission_range) / self.mission_range * 100.0

            # Wing detailed geometry
            mac_wing      = self.aircraft.main_wing.mean_aerodynamic_chord
            c_root_wing   = self.aircraft.main_wing.c_root_aero
            c_tip_wing    = self.aircraft.main_wing.c_tip
            sweep_wing    = self.aircraft.wing_sweep_le
            dihedral_wing = self.aircraft.wing_dihedral
            tc_wing       = self.aircraft.main_wing.thickness_to_chord
            x_ac_wing     = self.aircraft.main_wing.x_ac

            # Tail surfaces
            ht_area  = self.aircraft.horizontal_tail._effective_area
            ht_span  = self.aircraft.horizontal_tail._effective_span * 2.0
            ht_ar    = self.aircraft.horizontal_tail.aspect_ratio
            ht_arm   = self.aircraft.horizontal_tail.tail_arm
            ht_mac   = self.aircraft.horizontal_tail.mean_aerodynamic_chord
            x_ac_ht  = self.aircraft.horizontal_tail.x_ac

            vt_area  = self.aircraft.vertical_tail._effective_area
            vt_span  = self.aircraft.vertical_tail._effective_span
            vt_ar    = self.aircraft.vertical_tail.aspect_ratio
            vt_arm   = self.aircraft.vertical_tail.tail_arm

            # Fuel tank
            ft_sizing = self.aircraft._fuel_tank_sizing

            # Component mass and CG breakdown
            mass_bd = self.aircraft.mass_breakdown
            cg_bd   = self.aircraft.cg_breakdown

            # W/P–W/S diagram
            diagram_path = None
            try:
                tmp = tempfile.NamedTemporaryFile(
                    suffix=".png", delete=False,
                    dir=os.path.join(save_dir, "Outputfiles"))
                tmp.close()
                self.mission.save_wp_ws_figure(tmp.name)
                diagram_path = tmp.name
            except Exception as _de:
                print(f"[PDF] diagram embed skipped: {_de}")

            # ── Build story ────────────────────────────────────────────── #
            story = []

            story.append(Paragraph("UAV Initial Sizing — Design Report", title_style))
            story.append(Paragraph(
                f"Generated: {datetime.datetime.now().strftime('%d %b %Y  %H:%M')}",
                body_style,
            ))
            story.append(Spacer(1, 0.4*cm))

            # ── Section 1: Mission Parameters ──────────────────────────── #
            story += section("1 · Mission Parameters")
            story.append(data_table([
                ["Parameter",              "Value"],
                ["Cruise speed",           f"{self.cruise_speed:.1f} m/s   ({self.cruise_speed * 3.6:.0f} km/h)"],
                ["Cruise Mach number",     f"M = {mach_cruise:.4f}"],
                ["Mission altitude",       f"{self.mission_altitude:.0f} m   ({self.mission_altitude * 3.2808:.0f} ft)"],
                ["ISA temperature",        f"{T_isa:.1f} K   ({T_isa - 273.15:.1f} °C)"],
                ["ISA pressure",           f"{p_isa / 1000.0:.3f} kPa"],
                ["ISA air density",        f"{rho_isa:.4f} kg/m³"],
                ["Speed of sound",         f"{a_isa:.2f} m/s"],
                ["Mission range",          f"{self.mission_range:.0f} km"],
                ["Mission endurance",      f"{self.mission_endurance:.2f} hr"],
                ["Payload role",           str(self.payload_role)],
                ["Mission objective",      self.mission_objective],
                ["UAV class",              self.uav_class],
                ["Max. load factor",       f"{self.maximum_load_factor:.2f} g"],
                ["Engine type",            eng_type],
                ["SFC / BSFC",             f"{sfc:.3f} 1/hr"],
                ["Propeller efficiency",   f"{eta:.2f}" if eng_type != "Jet" else "N/A (jet)"],
            ]))

            # ── Section 2: Weight Budget ───────────────────────────────── #
            story += section("2 · Weight Budget")
            story.append(data_table([
                ["Component",            "Mass [kg]",                    "Fraction of MTOW"],
                ["MTOW",                 f"{mtow:.2f}",                  "—"],
                ["Empty weight",         f"{empty_wt:.2f}",              f"{empty_wt / mtow:.4f}"],
                ["Fuel weight",          f"{fuel_wt:.2f}",               f"{fuel_wt / mtow:.4f}"],
                ["Payload weight",       f"{payload_wt:.2f}",            f"{payload_wt / mtow:.4f}"],
            ], col_widths=[6.5*cm, 5.5*cm, 5*cm]))

            story.append(Spacer(1, 0.25*cm))
            story.append(Paragraph("Component mass breakdown:", body_style))
            story.append(Spacer(1, 0.1*cm))
            comp_rows = [["Component", "Mass [kg]", "CG x [m]"]]
            for k in mass_bd:
                comp_rows.append([
                    k.replace("_", " ").title(),
                    f"{mass_bd[k]:.2f}",
                    f"{cg_bd.get(k, 0.0):.3f}",
                ])
            comp_rows.append([
                "TOTAL",
                f"{self.aircraft.total_structural_mass:.2f}",
                f"{cg_x:.3f}",
            ])
            story.append(data_table(comp_rows, col_widths=[7*cm, 4.5*cm, 5.5*cm]))

            # ── Section 3: Performance Margins ────────────────────────── #
            story += section("3 · Performance Margins")
            driver_label = "CRUISE  (range legs consume more fuel)" \
                if driver == 'cruise' else "LOITER  (endurance leg consumes more fuel)"
            perf_rows = [
                ["Parameter",                   "Value"],
                ["L/D at cruise legs",           f"{pm['ld_cruise']:.3f}"],
                ["L/D at loiter leg",            f"{pm['ld_loiter']:.3f}"],
                ["Fuel fraction — cruise legs",  f"{pm['fuel_frac_cruise']:.4f}"],
                ["Fuel fraction — loiter leg",   f"{pm['fuel_frac_loiter']:.4f}"],
                ["Total fuel fraction  Wf/W0",   f"{pm['wf_w0']:.4f}"],
                ["Sizing driver",                driver_label],
            ]
            if driver == 'cruise' and e_achievable is not None:
                perf_rows += [
                    ["Required range (driver)",      f"{self.mission_range:.0f} km"],
                    ["Required endurance",           f"{self.mission_endurance:.2f} hr"],
                    ["Achievable endurance *",       f"{e_achievable:.2f} hr   (+{e_margin_pct:.1f}% margin)"],
                ]
            elif r_achievable is not None:
                perf_rows += [
                    ["Required endurance (driver)",  f"{self.mission_endurance:.2f} hr"],
                    ["Required range",               f"{self.mission_range:.0f} km"],
                    ["Achievable range *",           f"{r_achievable:.0f} km   (+{r_margin_pct:.1f}% margin)"],
                ]
            story.append(data_table(perf_rows))
            story.append(Spacer(1, 0.1*cm))
            story.append(Paragraph(
                "* Achievable value: using the fuel fraction consumed by the "
                "dominant leg, how long the non-dominant segment could be flown "
                "(Breguet inversion at the same L/D). Represents the margin "
                "available beyond the stated requirement.",
                note_style,
            ))

            # ── Section 4: Mission Segment Weight Fractions ────────────── #
            story += section("4 · Mission Segment Weight Fractions")
            story.append(data_table([
                ["Segment",                          "Wi+1 / Wi",             "Fuel consumed (% MTOW)"],
                ["Taxi & warm-up  (w1/w0)",          f"{fr['w1_w0']:.4f}",    f"{(1 - fr['w1_w0']) * 100:.2f} %"],
                ["Climb           (w2/w1)",          f"{fr['w2_w1']:.4f}",    f"{(1 - fr['w2_w1']) * 100:.2f} %"],
                ["Cruise outbound (w3/w2)",          f"{fr['w3_w2']:.4f}",    f"{(1 - fr['w3_w2']) * 100:.2f} %"],
                ["Loiter          (w4/w3)",          f"{fr['w4_w3']:.4f}",    f"{(1 - fr['w4_w3']) * 100:.2f} %"],
                ["Cruise return   (w5/w4)",          f"{fr['w5_w4']:.4f}",    f"{(1 - fr['w5_w4']) * 100:.2f} %"],
                ["Reserve loiter  (w6/w5)",          f"{fr['w6_w5']:.4f}",    f"{(1 - fr['w6_w5']) * 100:.2f} %"],
                ["Descent & land  (w7/w6)",          f"{fr['w7_w6']:.4f}",    f"{(1 - fr['w7_w6']) * 100:.2f} %"],
                ["Taxi out        (w8/w7)",          f"{fr['w8_w7']:.4f}",    f"{(1 - fr['w8_w7']) * 100:.2f} %"],
                ["TOTAL  Wf/W0",                    f"{fr['wf_w0']:.4f}",    f"{(1 - fr['wf_w0']) * 100:.2f} %"],
            ], col_widths=[6.5*cm, 4*cm, 6.5*cm]))

            # ── Section 5: Wing & Aerodynamic Sizing ──────────────────── #
            story += section("5 · Wing & Aerodynamic Sizing")
            if eng_type == "Jet":
                thr_label = "Thrust loading  T/W  [—]"
                thr_value = f"{self.thrust_loading:.4f}" if self.thrust_loading is not None else "N/A"
            else:
                thr_label = "Power loading  W/P  [kg/W]"
                thr_value = (
                    f"{self.power_loading:.5f}   ({1.0 / self.power_loading:.1f} W/kg)"
                    if self.power_loading is not None else "N/A"
                )
            story.append(data_table([
                ["Parameter",              "Value"],
                ["Wing area  S",           f"{wing_area:.3f} m²"],
                ["Wing span  b",           f"{wing_span:.3f} m"],
                ["Aspect ratio  AR (geom)",f"{wing_ar:.3f}"],
                ["Aspect ratio  AR (Roskam sizing)", f"{self._wing_ar_roskam:.3f}"],
                ["Taper ratio  λ",         f"{self.wing_taper_ratio:.3f}"],
                ["Root chord  c_root",     f"{c_root_wing:.3f} m"],
                ["Tip chord   c_tip",      f"{c_tip_wing:.3f} m"],
                ["Mean aero chord  MAC",   f"{mac_wing:.3f} m"],
                ["Sweep LE  Λ",            f"{sweep_wing:.1f} °"],
                ["Dihedral  Γ",            f"{dihedral_wing:.1f} °"],
                ["Thickness-to-chord  t/c",f"{tc_wing:.3f}"],
                ["Wing AC (x from nose)",  f"{x_ac_wing:.3f} m"],
                ["Wing loading  W/S",      f"{wing_ld:.2f} N/m²"],
                ["Cruise L/D",             f"{ld:.3f}"],
                [thr_label,                thr_value],
            ]))

            # ── Section 6: Tail Surfaces ───────────────────────────────── #
            story += section("6 · Tail Surfaces")
            story.append(data_table([
                ["Parameter",          "Horizontal Tail",          "Vertical Tail"],
                ["Area  S",            f"{ht_area:.3f} m²",        f"{vt_area:.3f} m²"],
                ["Span / height",      f"{ht_span:.3f} m",         f"{vt_span:.3f} m"],
                ["Aspect ratio  AR",   f"{ht_ar:.3f}",             f"{vt_ar:.3f}"],
                ["Tail arm  l_t",      f"{ht_arm:.3f} m",          f"{vt_arm:.3f} m"],
                ["MAC",                f"{ht_mac:.3f} m",          "—"],
                ["AC x-position",      f"{x_ac_ht:.3f} m",         "—"],
                ["Volume coeff.",      f"{self.aircraft.tail_volume_coefficient_h:.3f}",
                                       f"{self.aircraft.tail_volume_coefficient_v:.4f}"],
            ], col_widths=[5.5*cm, 5.5*cm, 6*cm]))

            # ── Section 7: Fuselage & Fuel System ─────────────────────── #
            story += section("7 · Fuselage & Fuel System")
            fus_rows = [
                ["Parameter",              "Value"],
                ["Fuselage length",        f"{fus_len:.3f} m"],
                ["Fuselage radius",        f"{fus_rad:.4f} m"],
                ["Fineness ratio  l/d",    f"{fus_len / (2.0 * fus_rad):.2f}"],
                ["Cylinder start",         f"{self.fuselage_cylinder_start:.1f} %"],
                ["Cylinder end",           f"{self.fuselage_cylinder_end:.1f} %"],
            ]
            if ft_sizing is not None:
                fuel_vol_L = fuel_wt / ft_sizing.fuel_density * 1000.0
                fus_rows += [
                    ["Fuel type",              ft_sizing.fuel_label],
                    ["Fuel density",           f"{ft_sizing.fuel_density:.1f} kg/m³"],
                    ["Fuel mass",              f"{fuel_wt:.2f} kg"],
                    ["Fuel volume",            f"{fuel_vol_L:.1f} L   ({fuel_vol_L / 1000.0:.4f} m³)"],
                    ["Tank outer radius",      f"{ft_sizing.outer_radius * 1000.0:.1f} mm"],
                    ["Tank total length",      f"{ft_sizing.total_length * 1000.0:.1f} mm"],
                    ["Tank AR (l/d)",          f"{self.fuel_tank_aspect_ratio:.2f}"],
                ]
            else:
                fus_rows.append(["Fuel system", "No fuel tank (fuel mass = 0)"])
            story.append(data_table(fus_rows))

            # ── Section 8: Longitudinal Stability ─────────────────────── #
            story += section("8 · Longitudinal Stability")
            cg_rows = [["Component", "Mass [kg]", "CG x [m]", "m · x [kg·m]"]]
            for k in mass_bd:
                cg_rows.append([
                    k.replace("_", " ").title(),
                    f"{mass_bd[k]:.2f}",
                    f"{cg_bd.get(k, 0.0):.3f}",
                    f"{mass_bd[k] * cg_bd.get(k, 0.0):.2f}",
                ])
            cg_rows.append([
                "TOTAL",
                f"{self.aircraft.total_structural_mass:.2f}",
                f"{cg_x:.3f}",
                f"{sum(mass_bd[k] * cg_bd.get(k, 0.0) for k in mass_bd):.2f}",
            ])
            story.append(data_table(cg_rows, col_widths=[5.5*cm, 3.5*cm, 3.5*cm, 4.5*cm]))

            story.append(Spacer(1, 0.25*cm))
            story.append(data_table([
                ["Parameter",                  "Value"],
                ["Wing MAC",                   f"{mac_wing:.3f} m"],
                ["Wing AC  x_ac_w",            f"{x_ac_wing:.3f} m"],
                ["Horizontal tail AC  x_ac_h", f"{x_ac_ht:.3f} m"],
                ["Aircraft CG  (from nose)",   f"{cg_x:.3f} m   ({cg_x / fus_len * 100:.1f} % fus. length)"],
                ["Neutral point (from nose)",  f"{np_x:.3f} m   ({np_x / fus_len * 100:.1f} % fus. length)"],
                ["Static margin  SM",          f"{sm * 100.0:.2f} % MAC"],
                ["Stability assessment",       stab],
            ]))

            # # ── Section 9: W/P–W/S Diagram ────────────────────────────── #
            # if diagram_path and os.path.exists(diagram_path):
            #     story += section("9 · W/P – W/S Design-Point Diagram")
            #     try:
            #         img = RLImage(diagram_path, width=15*cm, height=10*cm)
            #         story.append(img)
            #     except Exception as _ie:
            #         story.append(Paragraph(f"[Diagram embed failed: {_ie}]", body_style))

            doc.build(story)
            print(f"✓ PDF report saved: {pdf_path}")

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
        save_dir  = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        stp_path  = os.path.join(save_dir, f"Outputfiles/drone_geometry_{timestamp}.stp")

        _archive_previous(os.path.join(save_dir, "Outputfiles"), "drone_geometry_*.stp")

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
                # Handle lists/sequences (quantified @Part returns these)
                if isinstance(obj, (list, tuple)):
                    for item in obj:
                        _collect(item)
                    return
                if not isinstance(obj, GeomBase):
                    return
                added = _add_shape(obj)
                if not added:
                    try:
                        for child in obj.children:
                            if child is not obj:
                                _collect(child)
                    except Exception:
                        pass

            # ── main aircraft structure (fuselage, wings, tails, engine nacelles) ──
            _collect(self.aircraft)

            # ── payload (lives as a Part on Drone, NOT under Aircraft) ──
            try:
                _collect(self.payload)
            except Exception as _exc:
                print(f"[STP] payload collection skipped: {_exc}")

            # ── propeller blades (quantified @Part — may not surface through
            #    obj.children traversal; collect explicitly by path) ──
            try:
                eng = self.aircraft.engines
                for _prop in (eng.prop_starboard, eng.prop_port):
                    if getattr(_prop, 'suppress', False):
                        continue
                    try:
                        for _blade in _prop.blades:
                            _add_shape(_blade)
                        _add_shape(_prop.spinner)
                    except Exception as _be:
                        print(f"[STP] blade/spinner collection skipped for "
                              f"{_prop.label}: {_be}")
            except Exception as _exc:
                print(f"[STP] propeller blade collection skipped: {_exc}")

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
    def _mission_sizing(self) -> tuple:
        """Single cached call to fuel_weight_sizing."""
        return self.mission.fuel_weight_sizing()

    @Attribute
    def MTOW(self) -> float:
        return self._mission_sizing[0]

    @Attribute
    def empty_weight(self) -> float:
        return self._mission_sizing[1]

    @Attribute
    def fuel_weight(self) -> float:
        return self._mission_sizing[2]

    # ================================================================ #
    # LOADING OUTPUTS
    # ================================================================ #

    @Attribute
    def wing_loading(self) -> float:
        W_S, _ = self.mission.thrust_and_wing_loading()
        return W_S

    @Attribute
    def power_loading(self) -> Optional[float]:
        if self.engine_type in ("Turboprop", "Piston"):
            _, W_P = self.mission.thrust_and_wing_loading()
            return W_P
        return None

    @Attribute
    def thrust_loading(self) -> Optional[float]:
        if self.engine_type == "Jet":
            _, T_W = self.mission.thrust_and_wing_loading()
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
        return self.mission.ld_cruise()

    # ================================================================ #
    # PERFORMANCE MARGINS
    # ================================================================ #

    @Attribute
    def performance_margins(self) -> dict:
        return self.mission.performance_margins()

    @Attribute
    def performance_margins_summary(self) -> str:
        return self.mission.performance_margins_summary()

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
            wing_taper_ratio=self.wing_taper_ratio,
            payload_object=self.payload,
            payload_nose_clearance=self.payload_nose_clearance,
            fuselage_cylinder_start=self.fuselage_cylinder_start,
            fuselage_cylinder_end=self.fuselage_cylinder_end,
            fuel_mass=self.fuel_weight,
            fuel_tank_type=self.fuel_type,
            fuel_tank_aspect_ratio=self.fuel_tank_aspect_ratio,
            engine_type_str=self.engine_type,
        )

    # ================================================================ #
    # STABILITY SHORTCUTS
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