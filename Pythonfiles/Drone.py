"""
drone.py
========
Top-level drone class.
"""

import math
import os
import glob
import shutil
from typing import Optional

from parapy.core import Input, Attribute, Part, action
from parapy.geom import GeomBase


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
        validator=_between(0.0, 20_000.0),
        doc="Cruise / loiter altitude  [m]  ·  valid: 0 – 20 000 m\n"
            "Practical ceilings by engine type:\n"
            "  Piston   ≤ 4 500 m  |  Turboprop ≤ 9 000 m  |  Jet up to 20 000 m\n"
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
    def payload_start_x(self) -> float:
        """X-position where the payload bay begins [m]."""
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
        import datetime

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

    @action(label="Run Wing Airfoil Sweep")
    def run_wing_sweep(self):
        self.aircraft.main_wing.run_sweep()

    @action(label="Plot Wing XFoil polars")
    def plot_wing_cl_alpha(self):
        import datetime

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
        Write a design-summary PDF to the Outputfiles folder.
        Any previously generated drone_report_*.pdf is moved to
        Outputfiles/data/ before the new file is written.
        """
        import datetime
        import tempfile

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
                HRFlowable, Image as RLImage,
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

            story = []

            story.append(Paragraph("UAV Initial Sizing — Design Report", title_style))
            story.append(Paragraph(
                f"Generated: {datetime.datetime.now().strftime('%d %b %Y  %H:%M')}",
                body_style,
            ))
            story.append(Spacer(1, 0.4*cm))

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

            story += section("4 · Fuselage")
            story.append(data_table([
                ["Parameter",              "Value"],
                ["Fuselage length",        f"{fus_len:.2f} m"],
                ["Fuselage radius",        f"{fus_rad:.3f} m"],
                ["Cylinder start",         f"{self.fuselage_cylinder_start:.1f} %"],
                ["Cylinder end",           f"{self.fuselage_cylinder_end:.1f} %"],
            ]))

            story += section("5 · Longitudinal Stability")
            story.append(data_table([
                ["Parameter",              "Value"],
                ["CG position  (from nose)", f"{cg_x:.3f} m"],
                ["Neutral point (from nose)", f"{np_x:.3f} m"],
                ["Static margin  SM",      f"{sm*100:.1f} % MAC"],
                ["Assessment",             stab],
            ]))

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
        import datetime

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