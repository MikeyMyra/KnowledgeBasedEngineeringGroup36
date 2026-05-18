"""
Aircraft.py — Mid-level assembly that combines all physical UAV components.

Instantiates and positions the LiftingSurface (wing + tail), Fuselage,
Engine, and FuelTank from mission-sizing outputs passed down by Drone.
Computes geometric outputs such as aerodynamic centre, CG, and tail volume
coefficients used for stability checking.
"""
import os
import math
import glob
import shutil
from types import SimpleNamespace

from parapy.core import Input, Attribute, Part, action
from parapy.geom import GeomBase

from Pythonfiles.Components.Liftingsurfaces.Liftingsurface import LiftingSurface
from Pythonfiles.Components.Fuselage.Fuselage import Fuselage
from Pythonfiles.Components.Engines.Engine import Engine
from Pythonfiles.Components.Fuel.FuelTank import FuelTank, FUELS, AUTO_SELECTION, _VOLUME_FACTOR




class Aircraft(GeomBase):

    # ============================================================ #
    # MISSION  — always required
    # ============================================================ #

    cruise_speed:    float = Input()   # [m/s]
    cruise_altitude: float = Input()
    aircraft_mass:   float = Input()   # [kg]  MTOW
    ld_required:     float = Input(None)
    maximum_load_factor: float = Input(1)

    # ============================================================ #
    # WING SIZING
    # ============================================================ #

    effective_wing_area:      float = Input()   # [m²]
    effective_wing_semi_span: float = Input()   # [m]

    # ============================================================ #
    # FUSELAGE
    # ============================================================ #

    fuselage_cylinder_start: float = Input(10.0)
    fuselage_cylinder_end:   float = Input(70.0)
    payload_object = Input(None)

    payload_nose_clearance: float = Input(0.12)

    fuel_mass:   float = Input(0.0)   # [kg] — passed in from Drone.fuel_weight
    engine_mass: float = Input(None)  # [kg] — None = 10% MTOW (Roskam default)

    # Fuel tank
    fuel_tank_type: str = Input("auto")
    """Fuel type key ('avgas_100ll', 'jet_a', 'jp8', 'lipo_battery') or 'auto'."""
    fuel_tank_aspect_ratio: float = Input(3.0)
    """Tank length-to-diameter ratio.  3 is a good default for wing-box fits."""
    fuel_tank_color: str = Input("Green")
    """Render colour for the fuel tank capsule."""
    engine_type_str: str = Input("Piston")
    """Engine type string from Drone — used for auto fuel-type selection."""

    # ============================================================ #
    # WING AERODYNAMICS
    # ============================================================ #

    wing_taper_ratio:             float = Input(0.40)
    # Optional NACA 4-series code passed from Drone (e.g. "2412").
    # Forwarded to main_wing.naca_input; has no effect on tails.
    wing_naca_input:              str   = Input(None)
    # Path to the active .dat file that drives wing geometry.
    # Generated and resolved by Drone._active_wing_dat_path.
    wing_active_dat_path:         str   = Input(None)
    
    wing_sweep_le: float = Input(
        5.0,
        doc="Wing leading-edge sweep angle  [°]  ·  recommended: 0 – 30°\n"
            "Values above ~30° push the swept wing tip far aft, risking\n"
            "overlap with the horizontal tail planform at high AR.\n"
            "Subsonic UAVs: 0–10°  |  transonic jets: 15–30°.",
    )
    wing_twist:                   float = Input(0.0)
    wing_dihedral:                float = Input(5.0)
    wing_thickness_to_chord:      float = Input(0.15)
    wing_maximum_camber:          float = Input(0.04)
    wing_maximum_camber_position: float = Input(0.40)
    wing_t_factor_root:           float = Input(1.0)
    wing_t_factor_tip:            float = Input(1.0)

    # ============================================================ #
    # TAIL AERODYNAMICS
    # ============================================================ #

    tail_taper_ratio:             float = Input(0.40)
    tail_sweep_le:                float = Input(10.0)
    tail_twist:                   float = Input(0.0)
    tail_dihedral:                float = Input(0.0)
    tail_thickness_to_chord:      float = Input(0.15)
    tail_maximum_camber:          float = Input(0.0)
    tail_maximum_camber_position: float = Input(0.0)
    tail_t_factor_root:           float = Input(1.0)
    tail_t_factor_tip:            float = Input(1.0)

    # ============================================================ #
    # ENGINE / PROPULSION
    # ============================================================ #

    thrust_to_weight: float = Input()
    rho:              float = Input()   # ISA sea-level [kg/m³]; unused — Engine uses main_wing.density

    # ============================================================ #
    # UNDERCARRIAGE
    # ============================================================ #

    undercarriage_retractible: bool = Input(False)

    # ============================================================ #
    # COLORS
    # ============================================================ #

    fuselage_cones_color:      object = Input("steelblue")
    fuselage_cylinder_color:   object = Input("steelblue")
    undercarriage_color_tyre:  str    = Input("black")
    undercarriage_color_axle:  str    = Input("white")
    undercarriage_color_strut: str    = Input("silver")

    main_wing_color_wingbox:        str = Input("black")
    main_wing_color_liftingsurface: str = Input("yellow")

    tail_h_color_wingbox:        str = Input("black")
    tail_h_color_liftingsurface: str = Input("silver")
    tail_v_color_wingbox:        str = Input("black")
    tail_v_color_liftingsurface: str = Input("white")

    engine_color_nacelle: str = Input("silver")

    # ============================================================ #
    # ROSKAM / RAYMER EMPIRICAL CONSTANTS
    # ============================================================ #

    tail_volume_coefficient_h: float = Input(0.60)
    tail_volume_coefficient_v: float = Input(0.04)
    tail_aspect_ratio_h:       float = Input(4.50)
    tail_aspect_ratio_v:       float = Input(1.80)

    disk_loading_uav:  float = Input(80.0)
    target_solidity:   float = Input(0.15)
    blade_sweep:       float = Input(5.0)

    nacelle_length_override:   float = Input(None)
    nacelle_radius_override:   float = Input(None)
    n_blades_override:         int   = Input(None)
    blade_length_override:     float = Input(None)
    blade_root_chord_override: float = Input(None)

    g: float = Input(9.81)

    wing_front_spar_position: float = Input(0.15)
    wing_rear_spar_position:  float = Input(0.60)
    tail_front_spar_position: float = Input(0.15)
    tail_rear_spar_position:  float = Input(0.60)

    inlet_radius_ratio:  float = Input(0.85)
    nozzle_radius_ratio: float = Input(0.70)

    mesh_deflection: float = Input(1e-4)

    @Attribute
    def _fuel_tank_sizing(self):

        if math.isnan(self.fuel_mass) or self.fuel_mass <= 0.0:
            return None

        # ── resolve fuel type ──────────────────────────────────────── #
        ft = self.fuel_tank_type
        if ft == "auto":
            ft = AUTO_SELECTION.get(self.engine_type_str, "jet_a")
        props = FUELS[ft]

        density = props["density_kg_m3"]
        label   = props["label"]

        # ── capsule geometry (mirrors FuelTank formulas exactly) ────── #
        ar         = max(self.fuel_tank_aspect_ratio, 1.01)
        fuel_vol   = self.fuel_mass / density
        tank_vol   = fuel_vol * _VOLUME_FACTOR
        r3         = tank_vol / (math.pi * (2.0 * ar - 2.0 / 3.0))
        R          = r3 ** (1.0 / 3.0)
        total_len  = 2.0 * R * ar

        return SimpleNamespace(
            # Fuselage interface
            min_fuselage_length = total_len,
            min_fuselage_radius = R * 1.03,
            # CG / report interface
            outer_radius  = R,
            total_length  = total_len,
            cg_local_x    = total_len / 2.0,
            fuel_label    = label,
            fuel_density  = density,
        )

    @Attribute
    def _fm_safe(self):
        return self.fuel_mass if (not math.isnan(self.fuel_mass) and self.fuel_mass > 0.0) else 0.0

    @Part
    def fuel_tank(self):
        """
        Rendered fuel tank capsule — positioned aft of the payload bay.
        """
        return FuelTank(
            suppress=self._fm_safe <= 0.0,
            fuel_mass=max(self._fm_safe, 0.01),   # guard against suppress=False/0
            fuel_type=self.fuel_tank_type,
            engine_type=self.engine_type_str,
            tank_aspect_ratio=self.fuel_tank_aspect_ratio,
            color_tank=self.fuel_tank_color,
            position=self._fuel_tank_position,
        )

    @Attribute
    def _fuel_tank_position(self):
        """
        Nose position of the rendered fuel tank [m from aircraft origin].

        Roskam Vol. I §8.1: fuel CG at wing AC for stability — the rendered
        tank is a fuselage capsule for visualisation; mass/CG analysis uses
        wing AC regardless of the rendered position.
        """
        from parapy.geom import translate as _translate, Vector as _Vector
        payload_len = (self.payload_object.min_fuselage_length
                       if self.payload_object is not None else 0.0)
        nominal_start = (self.fuselage._x_cylinder_start
                         + self.fuselage.length * self.payload_nose_clearance
                         + payload_len
                         + 0.05)
        tank_start_x = max(nominal_start, self._wingbox_end_x + 0.05)
        if tank_start_x > nominal_start + 1e-6:
            print(
                f"[Aircraft] Fuel tank pushed aft by wingbox: "
                f"nominal {nominal_start:.3f} m → {tank_start_x:.3f} m "
                f"(wingbox end {self._wingbox_end_x:.3f} m)"
            )
        return _translate(self.position, _Vector(1, 0, 0), tank_start_x)

    # ============================================================ #
    # PARTS
    # ============================================================ #

    @Part
    def fuselage(self):
        return Fuselage(
            aircraft_mass=self.aircraft_mass,
            cylinder_start=self.fuselage_cylinder_start,
            cylinder_end=self.fuselage_cylinder_end,
            color_taper=self.fuselage_cones_color,
            color_cylinder=self.fuselage_cylinder_color,
            undercarriage_retractible=self.undercarriage_retractible,
            undercarriage_color_tyre=self.undercarriage_color_tyre,
            undercarriage_color_axle=self.undercarriage_color_axle,
            undercarriage_color_strut=self.undercarriage_color_strut,
            prop_clearance_radius=self._approx_prop_clearance_radius,
            payload=self.payload_object,
            payload_nose_clearance=self.payload_nose_clearance,
            fuel_tank=self._fuel_tank_sizing,   # sizing-only Attribute, no position dep
            length_min_override=self._min_fuselage_length_from_wing,
            radius_min_override=self._min_fuselage_radius_from_wing,
        )

    @Part
    def main_wing(self):
        return LiftingSurface(
            label="main_wing",
            effective_area=self.effective_wing_area,
            effective_semi_span=self.effective_wing_semi_span,

            fuselage_length=self.fuselage.length,
            fuselage_radius=self.fuselage.radius,

            is_tail=False,
            is_vertical_tail=False,

            taper_ratio=self.wing_taper_ratio,
            sweep_le=self.wing_sweep_le,
            twist=self.wing_twist,
            dihedral=self.wing_dihedral,

            thickness_to_chord_input=self.wing_thickness_to_chord,
            maximum_camber_input=self.wing_maximum_camber,
            maximum_camber_position_input=self.wing_maximum_camber_position,
            naca_input=self.wing_naca_input,
            active_dat_path=self.wing_active_dat_path,

            t_factor_root=self.wing_t_factor_root,
            t_factor_tip=self.wing_t_factor_tip,

            front_spar_position=self.wing_front_spar_position,
            rear_spar_position=self.wing_rear_spar_position,

            wing_ref=None,

            color_wingbox=self.main_wing_color_wingbox,
            color_liftingsurface=self.main_wing_color_liftingsurface,

            ld_required=self.ld_required,
            weight=self.aircraft_mass,
            velocity=self.cruise_speed,
            altitude=self.cruise_altitude,
            maximum_load_factor=self.maximum_load_factor,
        )

    @Part
    def horizontal_tail(self):
        return LiftingSurface(
            label="horizontal_tail",
            wing_ref=self.main_wing,
            is_tail=True,
            is_vertical_tail=False,

            fuselage_length=self.fuselage.length,
            fuselage_radius=self.fuselage.radius,
            fuselage_cone_radius_fn=self.fuselage.local_radius_at,

            taper_ratio=self.tail_taper_ratio,
            sweep_le=self.tail_sweep_le,
            twist=self.tail_twist,
            dihedral=self.tail_dihedral,

            thickness_to_chord_input=self.tail_thickness_to_chord,
            maximum_camber_input=self.tail_maximum_camber,
            maximum_camber_position_input=self.tail_maximum_camber_position,

            t_factor_root=self.tail_t_factor_root,
            t_factor_tip=self.tail_t_factor_tip,

            front_spar_position=self.tail_front_spar_position,
            rear_spar_position=self.tail_rear_spar_position,

            tail_volume_coefficient_h=self.tail_volume_coefficient_h,
            tail_volume_coefficient_v=self.tail_volume_coefficient_v,
            tail_aspect_ratio_h=self.tail_aspect_ratio_h,
            tail_aspect_ratio_v=self.tail_aspect_ratio_v,

            color_wingbox=self.tail_h_color_wingbox,
            color_liftingsurface=self.tail_h_color_liftingsurface,
        )

    @Part
    def vertical_tail(self):
        return LiftingSurface(
            label="vertical_tail",
            wing_ref=self.main_wing,
            is_tail=True,
            is_vertical_tail=True,

            fuselage_length=self.fuselage.length,
            fuselage_radius=self.fuselage.radius,
            fuselage_cone_radius_fn=self.fuselage.local_radius_at,

            taper_ratio=self.tail_taper_ratio,
            sweep_le=35.0,
            twist=0.0,
            dihedral=0.0,

            thickness_to_chord_input=self.tail_thickness_to_chord,
            maximum_camber_input=0.0,
            maximum_camber_position_input=self.tail_maximum_camber_position,

            t_factor_root=self.tail_t_factor_root,
            t_factor_tip=self.tail_t_factor_tip,

            front_spar_position=self.tail_front_spar_position,
            rear_spar_position=self.tail_rear_spar_position,

            tail_volume_coefficient_h=self.tail_volume_coefficient_h,
            tail_volume_coefficient_v=self.tail_volume_coefficient_v,
            tail_aspect_ratio_h=self.tail_aspect_ratio_h,
            tail_aspect_ratio_v=self.tail_aspect_ratio_v,

            color_wingbox=self.tail_v_color_wingbox,
            color_liftingsurface=self.tail_v_color_liftingsurface,
        )

    @Part
    def engines(self):
        return Engine(
            cruise_speed=self.cruise_speed,
            mtow=self.aircraft_mass,
            thrust_to_weight=self.thrust_to_weight,

            rho=self.main_wing.density,
            g=self.g,
            mission_altitude=self.cruise_altitude,    # ← drives altitude ceiling check

            semi_span=self.main_wing.effective_semi_span,
            sweep_le=self.main_wing.sweep_le,
            dihedral=self.main_wing.dihedral,

            fuselage_length=self.fuselage.length,
            fuselage_radius=self.fuselage.radius,

            wing_root_x=self.main_wing._root_position.x,
            wing_root_z=self.main_wing._root_position.z,

            disk_loading_uav=self.disk_loading_uav,
            target_solidity=self.target_solidity,

            nacelle_length_override=self.nacelle_length_override,
            nacelle_radius_override=self.nacelle_radius_override,

            inlet_radius_ratio=self.inlet_radius_ratio,
            nozzle_radius_ratio=self.nozzle_radius_ratio,

            n_blades_override=self.n_blades_override,
            blade_length_override=self.blade_length_override,
            blade_root_chord_override=self.blade_root_chord_override,

            blade_sweep=self.blade_sweep,

            color_nacelle=self.engine_color_nacelle,
        )
    
    @Attribute
    def _approx_prop_clearance_radius(self) -> float:
        """
        Approximate largest rotating radius for undercarriage strut sizing [m].

        Jet engines  : nacelle outer radius, sea-level rated (lapse-corrected).
                       Roskam Vol. V §4: D_nac = 0.2284 * T_sl_kN^0.4.
        Prop engines : propeller blade-tip radius, altitude-corrected.
                       Roskam Vol. I §3.6 / actuator-disk: D = 0.658 * P_kW^0.25,
                       scaled to altitude by D_alt = D_sl * sqrt(rho_sl / rho).
        """
        from math import sqrt
        n = 1 if self.aircraft_mass < 2700 else 2

        if self.engine_type_str == "Jet":
            # Nacelle radius (sea-level equivalent after lapse correction)
            T_per_N  = self.thrust_to_weight * self.aircraft_mass * self.g / n
            sigma    = max(self.rho, 0.01) / 1.225
            lapse    = min(sigma ** 0.75, 1.0)
            T_sl_kN  = T_per_N / lapse / 1000.0
            return 0.2284 * (T_sl_kN ** 0.4) / 2.0

        else:
            P_total_W  = self.aircraft_mass / max(self.thrust_to_weight, 1e-9)
            P_per_kW   = max(P_total_W / n, 1.0) / 1000.0
            D_sl_per   = 0.658 * (P_per_kW ** 0.25)   # Roskam §3.6 sea-level
            geo_cap = self.effective_wing_semi_span * 0.15
            return min(D_sl_per / 2.0, geo_cap)

    @Attribute
    def _min_fuselage_length_from_wing(self) -> float:
        """
        Fuselage length lower bound that prevents wing–tail geometric overlap [m].

        Raymer §4.2: tail moment arm ≈ 2.5–3.5 × MAC.
        Global Hawk: span 39.9 m, fuselage 14.5 m — span/fuselage ≈ 2.75.
        """
        S      = self.effective_wing_area
        b      = self.effective_wing_semi_span * 2.0
        lamb   = self.wing_taper_ratio
        c_root = 2.0 * S / (b * (1.0 + lamb))
        return 4.5 * c_root

    @Attribute
    def _min_fuselage_radius_from_wing(self) -> float:
        """
        Fuselage radius lower bound from wing root chord.
        Raymer §4.2: r_fus >= 8% root chord to limit interference drag.
        """
        # root chord from area and span using taper ratio
        S    = self.effective_wing_area
        b    = self.effective_wing_semi_span * 2
        lamb = self.wing_taper_ratio
        c_root = 2 * S / (b * (1 + lamb))
        return 0.08 * c_root

    @Attribute
    def _wingbox_start_x(self) -> float:
        """
        X-position of the wing front spar at the fuselage centreline [m].
        """
        return self.main_wing.attach_x + self.wing_front_spar_position * self.main_wing.c_root_geometric

    @Attribute
    def _wingbox_end_x(self) -> float:
        """
        X-position of the wing rear spar at the fuselage centreline [m].
        """
        return self.main_wing.attach_x + self.wing_rear_spar_position * self.main_wing.c_root_geometric

    # ------------------------------------------------------------ #
    # Component masses
    # ------------------------------------------------------------ #

    @Attribute
    def _engine_mass(self) -> float:
        """
        Engine group mass [kg].
        Roskam Vol. I Table 8.1: propulsion group ~9–11% MTOW.
        Default 10% when not supplied.
        """
        return self.engine_mass if self.engine_mass is not None else 0.10 * self.aircraft_mass

    @Attribute
    def mass_breakdown(self) -> dict:
        """
        Dictionary of {component_label: mass [kg]} for all major groups.

        Sources
        -------
        - Fuselage  : Roskam Vol. I §8.3 Eq. (8.5)
        - Wing/tails: Roskam Vol. I §8.4 / §8.5 Table 8.1
        - Engine    : Roskam Vol. I Table 8.1 (10% MTOW default)
        - Fuel      : from mission sizing (passed in as fuel_mass)
        - Payload   : from Payload.total_mass
        """
        d = {
            "fuselage":        self.fuselage.calculate_mass,
            "main_wing":       self.main_wing.calculate_mass,
            "horizontal_tail": self.horizontal_tail.calculate_mass,
            "vertical_tail":   self.vertical_tail.calculate_mass,
            "engine":          self._engine_mass,
            "fuel":            self.fuel_mass,
        }
        if self.payload_object is not None:
            d["payload"] = self.payload_object.total_mass
        # Fuel tank structural mass: Roskam Vol. I §8.3 ~2–3 % of fuel mass for
        # metallic integral tanks.  Use 2.5 % as a UAV composite-tank estimate.
        if self.fuel_mass > 0:
            d["fuel_tank_structure"] = 0.025 * self.fuel_mass
        return d

    @Attribute
    def total_structural_mass(self) -> float:
        """Sum of all component masses [kg] — should be ≈ MTOW as a sanity check."""
        return sum(self.mass_breakdown.values())

    # ------------------------------------------------------------ #
    # Component CG x-positions (from nose)
    # ------------------------------------------------------------ #

    @Attribute
    def cg_breakdown(self) -> dict:
        """
        Dictionary of {component_label: cg_x [m]} for all major groups.
        """
        payload_len   = (self.payload_object.min_fuselage_length
                         if self.payload_object is not None else 0.0)
        # ── payload wingbox overlap check ──────────────────────────── #
        # payload starts at cylinder_start + payload_nose_clearance
        payload_start_x = self.fuselage._x_cylinder_start + self.fuselage.length * self.payload_nose_clearance
        payload_end_x   = payload_start_x + payload_len
        if (payload_len > 0.0
                and payload_end_x > self._wingbox_start_x
                and payload_start_x < self._wingbox_end_x):
            print(
                f"[Aircraft] WARNING: payload bay (x={payload_start_x:.3f}"
                f"–{payload_end_x:.3f} m) overlaps wingbox "
                f"(x={self._wingbox_start_x:.3f}–{self._wingbox_end_x:.3f} m). "
                f"Consider reducing payload length or moving the wing aft."
            )

        fuel_cg_x = self.main_wing.x_ac   # Roskam §8.1

        d = {
            "fuselage":        self.fuselage.cg_x,
            "main_wing":       self.main_wing.cg_x,
            "horizontal_tail": self.horizontal_tail.cg_x,
            "vertical_tail":   self.vertical_tail.cg_x,
            "engine":          self.main_wing.x_ac,   # engine at wing AC
            "fuel":            fuel_cg_x,             # fuel in centre wing box
        }
        if self.payload_object is not None:
            d["payload"] = self.payload_object.cg_x   # item positions already in world frame
        if self.fuel_mass > 0:
            d["fuel_tank_structure"] = fuel_cg_x      # tank structure CG ≈ fuel CG
        return d

    @Attribute
    def cg_x(self) -> float:
        """
        Aircraft CG x-position from nose [m].

        Roskam Vol. I §8.1, Eq. (8.1).
        """
        masses = self.mass_breakdown
        cgs    = self.cg_breakdown
        total  = sum(masses.values())
        if total == 0:
            return 0.0
        return sum(masses[k] * cgs[k] for k in masses) / total

    # ------------------------------------------------------------ #
    # Neutral Point  (Roskam Vol. II §3.2)
    # ------------------------------------------------------------ #

    @Attribute
    def neutral_point_x(self) -> float:
        """
        Aircraft neutral point x-position from nose [m].

        Roskam Vol. II §3.2, Eq. (3.15) — simplified for straight-tapered
        wing + horizontal tail, subsonic incompressible
        """
        cl_alpha   = 2.0 * 3.14159265   # 2π [1/rad] — thin-aerofoil theory
        eta_h      = 0.90               # tail efficiency (Roskam Vol. II Table 3.1)
        S_w        = self.main_wing._effective_area
        S_h        = self.horizontal_tail._effective_area
        x_ac_w     = self.main_wing.x_ac
        x_ac_h     = self.horizontal_tail.x_ac

        tail_factor = eta_h * (S_h / S_w)   # dimensionless tail contribution weight

        return (cl_alpha * x_ac_w + tail_factor * cl_alpha * x_ac_h) / \
               (cl_alpha + tail_factor * cl_alpha)

    # ------------------------------------------------------------ #
    # Static Margin  (Roskam Vol. II §3.3)
    # ------------------------------------------------------------ #

    @Attribute
    def static_margin(self) -> float:
        """
        Longitudinal static margin as fraction of MAC [-].

            SM = (NP - CG) / MAC_wing

        Roskam Vol. II §3.3:
        - SM > 0  → statically stable (NP aft of CG)
        - SM = 0  → neutral (NP coincides with CG)
        - SM < 0  → unstable (NP forward of CG)
        - Target  : 0.05–0.15 (5–15 % MAC) for UAVs without FBW stability
                    augmentation.  FBW aircraft may accept SM down to –0.10.

        Roskam Vol. II §3.3, Eq. (3.1): SM = (x_np - x_cg) / c_bar
        """
        return (self.neutral_point_x - self.cg_x) / self.main_wing.mean_aerodynamic_chord

    @Attribute
    def static_margin_percent(self) -> float:
        """Static margin as a percentage of MAC [%]."""
        return self.static_margin * 100.0

    @Attribute
    def is_stable(self) -> bool:
        """True when static margin > 0 (NP aft of CG)."""
        return self.static_margin > 0.0

    @Attribute
    def stability_status(self) -> str:
        """Human-readable stability assessment."""
        sm = self.static_margin_percent
        if sm >= 5.0:
            return f"STABLE — SM = {sm:.1f}% MAC (target 5-15%)"
        elif sm >= 0.0:
            return f"MARGINAL — SM = {sm:.1f}% MAC (below 5% target)"
        else:
            return f"UNSTABLE — SM = {sm:.1f}% MAC (requires FBW or redesign)"

    # ------------------------------------------------------------ #
    # Output helper
    # ------------------------------------------------------------ #

    def _save_txt_report(self, lines: list, filename_stem: str) -> str:
        """Print *lines* to the terminal and save them to Outputfiles/<stem>_<ts>.txt.

        Returns the saved file path (or an error string if saving failed).
        """
        import datetime
        text = "\n".join(lines)
        print(text)

        save_dir  = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'Outputfiles'))
        archive_dir = os.path.join(save_dir, "data")
        os.makedirs(archive_dir, exist_ok=True)
        for existing in glob.glob(os.path.join(save_dir, f"{filename_stem}_*.txt")):
            shutil.move(existing, os.path.join(archive_dir, os.path.basename(existing)))
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        path      = os.path.join(save_dir, f"{filename_stem}_{timestamp}.txt")
        try:
            with open(path, "w", encoding="utf-8") as f:
                f.write(text + "\n")
            print(f"  → saved to {path}")
        except Exception as e:
            print(f"  [WARNING] could not save report: {e}")
        return path

    # ------------------------------------------------------------ #
    # Summary actions
    # ------------------------------------------------------------ #

    @action(label="Print fuel tank debug")
    def print_fuel_debug(self):
        lines = ["=" * 50, "FUEL TANK DEBUG"]
        lines.append(f"  fuel_mass        : {self.fuel_mass:.3f} kg")
        lines.append(f"  suppress would be: {self.fuel_mass <= 0.0}")
        lines.append(f"  _fuel_tank_sizing: {self._fuel_tank_sizing}")
        if self._fuel_tank_sizing is not None:
            ft = self._fuel_tank_sizing
            lines.append(f"  tank total_length: {ft.total_length:.3f} m")
            lines.append(f"  tank outer_radius: {ft.outer_radius:.3f} m")
        try:
            pos = self._fuel_tank_position
            lines.append(f"  tank position    : {pos}")
        except Exception as e:
            lines.append(f"  tank position ERR: {e}")
        lines.append("=" * 50)
        self._save_txt_report(lines, "fuel_debug")

    @action(label="Print stability report")
    def print_stability_report(self):
        masses = self.mass_breakdown
        cgs    = self.cg_breakdown

        lines = ["=" * 70, "STABILITY REPORT", "=" * 70]

        lines.append("\nCOMPONENT MASSES & CG POSITIONS")
        lines.append(f"  {'Component':<20s}  {'Mass [kg]':>10s}  {'CG x [m]':>10s}  {'m*x [kg.m]':>12s}")
        lines.append("  " + "-" * 58)
        for k in masses:
            m  = masses[k]
            cx = cgs[k]
            lines.append(f"  {k:<20s}  {m:>10.2f}  {cx:>10.3f}  {m*cx:>12.2f}")
        lines.append("  " + "-" * 58)
        lines.append(
            f"  {'TOTAL':<20s}  {self.total_structural_mass:>10.2f}  "
            f"{self.cg_x:>10.3f}  "
            f"{sum(masses[k]*cgs[k] for k in masses):>12.2f}"
        )

        lines.append("\nFUEL TANK")
        if self._fuel_tank_sizing is not None:
            ft = self._fuel_tank_sizing
            lines.append(f"  Fuel type          : {ft.fuel_label}")
            lines.append(f"  Fuel density       : {ft.fuel_density:.0f} kg/m3")
            lines.append(f"  Fuel mass          : {self.fuel_mass:.1f} kg")
            lines.append(f"  Tank outer radius  : {ft.outer_radius*1000:.1f} mm")
            lines.append(f"  Tank total length  : {ft.total_length*1000:.1f} mm")
        else:
            lines.append("  No fuel tank (fuel_mass = 0)")

        lines.append("\nAERODYNAMIC REFERENCE")
        lines.append(f"  Wing MAC              : {self.main_wing.mean_aerodynamic_chord:.3f} m")
        lines.append(f"  Wing AC (x_ac_w)      : {self.main_wing.x_ac:.3f} m")
        lines.append(f"  HT AC   (x_ac_h)      : {self.horizontal_tail.x_ac:.3f} m")
        lines.append(f"  Neutral Point         : {self.neutral_point_x:.3f} m")

        lines.append("\nLONGITUDINAL STABILITY")
        lines.append(f"  Aircraft CG           : {self.cg_x:.3f} m from nose")
        lines.append(f"  Neutral Point         : {self.neutral_point_x:.3f} m from nose")
        lines.append(f"  Static Margin         : {self.static_margin_percent:.1f}% MAC")
        lines.append(f"  Assessment            : {self.stability_status}")
        lines.append("=" * 70)

        self._save_txt_report(lines, "stability_report")


# ================================================================ #
# ENTRY POINT
# ================================================================ #

if __name__ == "__main__":
    from parapy.gui import display

    ac = Aircraft(
        cruise_speed=220.0,
        aircraft_mass=2000,
        cruise_altitude=5000,
        effective_wing_area=20.0,
        effective_wing_semi_span=8.0,
        thrust_to_weight=0.35,
        fuselage_cylinder_start=10.0,
        fuselage_cylinder_end=70.0,
        fuel_mass=300.0,
        fuel_tank_type="jet_a",
        engine_type_str="Jet",
    )
    display(ac)
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       