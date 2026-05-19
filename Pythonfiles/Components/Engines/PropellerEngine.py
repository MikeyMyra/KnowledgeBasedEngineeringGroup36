"""
Piston/turboprop propeller engine geometry.

Sizes engine cowling and propeller disk from MTOW and actuator-disk theory
(Roskam Vol. I §3.6), and lofts the nacelle solid. Supports both tractor
and pusher configurations for 3-D visualisation.
"""

from math import radians, tan, sin, cos, pi, sqrt, isfinite

from parapy.core import Input, Attribute, Part, child
from parapy.geom import GeomBase, LoftedSolid, Circle, translate, rotate, Vector

from Pythonfiles.Components.Frame import Frame


class PropellerEngine(GeomBase):
    """
    Propeller (tractor/pusher) engine with Roskam-based sizing.

    Sizing relations (Roskam Vol. I, §3.6 + actuator-disk theory):
    """

    # ------------------------------------------------------------------ #
    # PRIMARY INPUTS
    # ------------------------------------------------------------------ #

    mtow: float = Input()
    n_engines: int = Input()
    thrust_to_weight: float = Input()
    cruise_speed: float = Input()

    rho: float = Input()
    g: float = Input()

    # ------------------------------------------------------------------ #
    # GEOMETRY CONTEXT
    # ------------------------------------------------------------------ #

    semi_span: float = Input()
    sweep_le: float = Input()
    dihedral: float = Input()
    fuselage_radius: float = Input()

    # ------------------------------------------------------------------ #
    # POSITIONING
    # ------------------------------------------------------------------ #

    attach_spanwise_pct: float = Input()
    attach_x_offset: float = Input()
    attach_z_offset: float = Input()

    # ------------------------------------------------------------------ #
    # ROSKAM PARAMETERS
    # ------------------------------------------------------------------ #

    # Mission altitude [m] — used for the altitude feasibility check below
    mission_altitude: float = Input(0.0)

    disk_loading_uav: float = Input()
    target_solidity: float = Input()

    # ------------------------------------------------------------------ #
    # OVERRIDES
    # ------------------------------------------------------------------ #

    nacelle_length_override: float = Input()
    nacelle_radius_override: float = Input()

    n_blades_override: int = Input()
    blade_length_override: float = Input()
    blade_root_chord_override: float = Input()

    blade_sweep: float = Input()

    # ------------------------------------------------------------------ #
    # DISPLAY CONTROL
    # ------------------------------------------------------------------ #

    taper_sections: int = Input(8)
    color_nacelle: str = Input("Silver")
    color_spinner: str = Input("White")
    color_blade: str = Input("DarkGray")
    mesh_deflection: float = Input(1e-4)

    inlet_radius_ratio: float = Input(0.85)
    nozzle_radius_ratio: float = Input(0.70)

    # ------------------------------------------------------------------ #
    # ROSKAM THRUST
    # ------------------------------------------------------------------ #

    @Attribute
    def total_thrust(self) -> float:
        return self.thrust_to_weight * self.mtow * self.g

    @Attribute
    def thrust_per_engine(self) -> float:
        return self.total_thrust / self.n_engines

    # ------------------------------------------------------------------ #
    # ROSKAM PROPELLER SIZING  (Vol. I §3.6)
    # ------------------------------------------------------------------ #

    @Attribute
    def _disk_area(self) -> float:
        return self.thrust_per_engine / self.disk_loading_uav

    @Attribute
    def _power_required_cruise(self) -> float:
        """Cruise power from thrust × velocity / propeller efficiency."""
        eta_prop = 0.82   # Roskam Vol. I §3.6: typical cruise propeller efficiency
        return self.thrust_per_engine * self.cruise_speed / eta_prop

    @Attribute
    def _power_required_climb(self) -> float:
        """
        Climb power requirement.
        Roskam Vol. I §4.5: P_climb = W * ROC / eta_prop
        ROC target: ~3 m/s for large UAV (Raymer Table 3.6)
        """
        eta_prop = 0.75   # lower efficiency in climb
        roc      = 3.0    # [m/s] — rate of climb target
        return (self.mtow * self.g * roc) / (self.n_engines * eta_prop)

    @Attribute
    def _power_required_takeoff(self) -> float:
        """
        Static thrust at takeoff — propeller momentum theory at V=0.
        P = T^1.5 / sqrt(2 * rho_sl * A)
        Uses sea-level density (takeoff condition).
        """
        rho_sl = 1.225
        return (self.thrust_per_engine ** 1.5) / sqrt(2.0 * rho_sl * self._disk_area)

    @Attribute
    def shaft_power(self) -> float:
        """Design shaft power — maximum across all sizing conditions."""
        return max(
            self._power_required_cruise,
            self._power_required_climb,
            self._power_required_takeoff,
        )

    @Attribute
    def altitude_feasibility(self) -> str:
        """
        Altitude note for propeller propulsion.

        Engine type is determined solely by Mach number (M < 0.40 → propeller).

        Roskam Vol. I §3.2 practical altitude bands (for reference):
        Piston ceiling      : ~4 500 m  (15 000 ft)
        Turboprop ceiling   : ~9 000 m  (30 000 ft)
        Above 9 000 m: turboprop 
        """
        h          = self.mission_altitude
        rho_sl     = 1.225
        rho        = max(self.rho, 0.01)
        alt_scale  = sqrt(rho_sl / rho)
        P_kW       = self.shaft_power / 1000.0
        D_sl       = 0.658 * (P_kW ** 0.25)
        D_alt      = D_sl * alt_scale

        if h > 9_000.0:
            return (f"HIGH-ALT PROP — {h:.0f} m  |  "
                    f"density scale ×{alt_scale:.2f}  |  "
                    f"D_sl={D_sl:.2f} m → D_alt={D_alt:.2f} m  "
                    f"(capped at {self._max_blade_length*2:.2f} m if > wing limit)")
        if h > 4_500.0:
            return (f"MID-ALT PROP — {h:.0f} m  |  "
                    f"density scale ×{alt_scale:.2f}  |  "
                    f"D_sl={D_sl:.2f} m → D_alt={D_alt:.2f} m  "
                    f"(turboprop / turbo-normalised engine recommended)")
        return (f"OK — {h:.0f} m  |  "
                f"density scale ×{alt_scale:.2f}  |  "
                f"D_sl={D_sl:.2f} m → D_alt={D_alt:.2f} m")

    @Attribute
    def _max_blade_length(self) -> float:
        wing_limit = self.semi_span * 0.15
        fus_limit  = self.fuselage_radius * 1.5 if self.fuselage_radius > 0 else wing_limit
        if self.attach_spanwise_pct * self.semi_span < 0.10 * self.semi_span:
            return max(fus_limit, 0.10)
        return wing_limit

    @Attribute
    def blade_length(self) -> float:
        """
        Propeller blade length [m] with altitude density scaling.

        Step 1 — Roskam sea-level diameter (Vol. I §3.6):
            D_sl = 0.658 · P_kW^0.25

        Step 2 — Actuator-disk altitude correction:
            At altitude ρ the disk must sweep more area to generate the
            same thrust T, because T = 2ρAv_i² → A ∝ 1/ρ.
            Diameter scales as D ∝ √A ∝ 1/√ρ, so:
                D_alt = D_sl · √(ρ_sl / ρ)

            At 20 km (ρ ≈ 0.089 kg/m³) the correction factor is
            √(1.225 / 0.089) ≈ 3.7 — props are roughly 3.7× larger
            in diameter than at sea level.

        Step 3 — Geometric cap (_max_blade_length):
            Blades cannot exceed 15% semi-span (avoid tip vortex
            interference) or 1.5× fuselage radius at the nose.
            For HALE missions where the altitude correction is large,
            the correct response is more engines or accepting the cap —
            the n_blades Roskam formula naturally adds blades to maintain
            target disk solidity when the disk grows.
        """
        if self.blade_length_override is not None:
            return self.blade_length_override

        P_kW     = self.shaft_power / 1000.0
        D_sl     = 0.658 * (P_kW ** 0.25)     # Roskam sea-level diameter [m]

        # Altitude density scaling — actuator disk theory
        rho_sl = 1.225                          # ISA sea-level density [kg/m³]
        rho    = max(self.rho, 0.01)            # guard against near-zero density
        alt_scale = sqrt(rho_sl / rho)          # D_alt / D_sl = sqrt(rho_sl/rho)
        D_alt  = D_sl * alt_scale

        return min(D_alt / 2.0, self._max_blade_length)

    @Attribute
    def blade_root_chord(self) -> float:
        if self.blade_root_chord_override is not None:
            return self.blade_root_chord_override
        D_prop = 2.0 * self.blade_length
        c_base = 0.065 * D_prop
        return c_base * (self.n_blades / 2.0)

    @Attribute
    def blade_tip_chord(self) -> float:
        return self.blade_root_chord * 0.30

    @Attribute
    def n_blades(self) -> int:
        if self.n_blades_override is not None:
            return self.n_blades_override
        R = self.blade_length
        if not isfinite(R) or R <= 0:
            return 2
        D_prop = 2.0 * R
        c_base = 0.065 * D_prop
        if c_base <= 0:
            return 2
        n_raw = self.target_solidity * pi * R / c_base
        if not isfinite(n_raw):
            return 2
        return max(2, min(6, round(n_raw)))

    # ------------------------------------------------------------------ #
    # SPINNER SIZING
    # ------------------------------------------------------------------ #

    @Attribute
    def spinner_radius(self) -> float:
        return 0.15 * (2.0 * self.blade_length)

    @Attribute
    def spinner_length(self) -> float:
        return 1.5 * self.spinner_radius

    # ------------------------------------------------------------------ #
    # NACELLE SIZING  (motor cowling, not thrust-sized)
    # ------------------------------------------------------------------ #

    @Attribute
    def nacelle_radius(self) -> float:
        if self.nacelle_radius_override is not None:
            return self.nacelle_radius_override
        return self.spinner_radius

    @Attribute
    def nacelle_length(self) -> float:
        if self.nacelle_length_override is not None:
            return self.nacelle_length_override
        return 3.5 * self.nacelle_radius

    # ------------------------------------------------------------------ #
    # NACELLE PROFILE GEOMETRY
    # ------------------------------------------------------------------ #

    @Attribute
    def _nacelle_x_positions(self) -> list:
        n = self.taper_sections
        return [i / (n - 1) * self.nacelle_length for i in range(n)]

    @Attribute
    def _nacelle_radii(self) -> list:
        n   = self.taper_sections
        r   = self.nacelle_radius
        r_i = self.inlet_radius_ratio  * r
        r_n = self.nozzle_radius_ratio * r
        result = []
        for i in range(n):
            t = i / (n - 1)
            if t < 0.15:
                s = t / 0.15
                result.append(r_i + (r - r_i) * s)
            elif t > 0.80:
                s = (t - 0.80) / 0.20
                result.append(r + (r_n - r) * s)
            else:
                result.append(r)
        return result

    # ------------------------------------------------------------------ #
    # ATTACH POSITION
    # ------------------------------------------------------------------ #

    @Attribute
    def _attach_y(self) -> float:
        return 0.0 if self.n_engines == 1 else self.semi_span * self.attach_spanwise_pct

    @Attribute
    def _attach_x(self) -> float:
        if self.n_engines == 1:
            return 0.0
        return self._attach_y * tan(radians(self.sweep_le)) + self.attach_x_offset

    @Attribute
    def _attach_z(self) -> float:
        if self.n_engines == 1:
            return 0.0
        return self._attach_y * tan(radians(self.dihedral)) + self.attach_z_offset

    @Attribute
    def _engine_base(self):
        """Translated-only position (no rotation)."""
        return translate(
            self.position,
            Vector(1, 0, 0), self._attach_x,
            Vector(0, 1, 0), self._attach_y,
            Vector(0, 0, 1), self._attach_z,
        )

    @Attribute
    def _engine_position(self):
        """Fully oriented engine frame (rotated)."""
        return self._engine_base.rotate90('y')

    # ------------------------------------------------------------------ #
    # PARTS — NACELLE
    # ------------------------------------------------------------------ #

    @Part
    def nacelle_profiles(self):
        return Circle(
            quantify=self.taper_sections,
            radius=self._nacelle_radii[child.index],
            color=self.color_nacelle,
            position=translate(
                self._engine_base,
                Vector(1, 0, 0), self._nacelle_x_positions[child.index],
            ).rotate90('y'),
        )

    @Part
    def nacelle(self):
        return LoftedSolid(
            profiles=self.nacelle_profiles,
            color=self.color_nacelle,
            transparency=0.3,
            mesh_deflection=self.mesh_deflection,
        )

    # ------------------------------------------------------------------ #
    # PARTS — SPINNER
    # ------------------------------------------------------------------ #

    @Attribute
    def _spinner_radii(self) -> list:
        n = 6
        r = self.spinner_radius
        return [max(0.005, r * (1.0 - (1.0 - i / (n - 1)) ** 2) ** 0.5)
                for i in range(n)]

    @Attribute
    def _spinner_x_positions(self) -> list:
        n = 6
        return [i / (n - 1) * self.spinner_length for i in range(n)]

    @Part
    def spinner_profiles(self):
        return Circle(
            quantify=6,
            radius=self._spinner_radii[child.index],
            color=self.color_spinner,
            position=translate(
                self._engine_base,
                Vector(1, 0, 0),
                -self.spinner_length + self._spinner_x_positions[child.index],
            ).rotate90('y'),
        )

    @Part
    def spinner(self):
        return LoftedSolid(
            profiles=self.spinner_profiles,
            color=self.color_spinner,
            mesh_deflection=self.mesh_deflection,
        )

    # ------------------------------------------------------------------ #
    # PARTS — BLADES
    # ------------------------------------------------------------------ #

    @Attribute
    def _blade_angles_deg(self) -> list:
        return [360.0 / self.n_blades * i for i in range(self.n_blades)]

    @Attribute
    def _blade_root_positions(self) -> list:
        return [
            translate(
                rotate(self._engine_base, 'x', -radians(a)),
                Vector(0, sin(radians(a)), cos(radians(a))), self.spinner_radius,
            )#.rotate90('y')
            for a in self._blade_angles_deg
        ]

    @Attribute
    def _blade_tip_positions(self) -> list:
        x_sweep = self.blade_length * tan(radians(self.blade_sweep))
        return [
            translate(
                rotate(self._engine_base, 'x', -radians(a)),
                Vector(1, 0, 0), x_sweep,
                Vector(0, sin(radians(a)), cos(radians(a))),
                self.spinner_radius + self.blade_length,
            )#.rotate90('y')
            for a in self._blade_angles_deg
        ]

    @Part
    def blade_root_profiles(self):
        return Circle(
            quantify=self.n_blades,
            radius=self.blade_root_chord * 0.5,
            color=self.color_blade,
            position=self._blade_root_positions[child.index],
        )

    @Part
    def blade_tip_profiles(self):
        return Circle(
            quantify=self.n_blades,
            radius=self.blade_tip_chord * 0.5,
            color=self.color_blade,
            position=self._blade_tip_positions[child.index],
        )

    @Part
    def blades(self):
        return LoftedSolid(
            quantify=self.n_blades,
            profiles=[
                self.blade_root_profiles[child.index],
                self.blade_tip_profiles[child.index],
            ],
            color=self.color_blade,
            mesh_deflection=self.mesh_deflection,
        )

    # ------------------------------------------------------------------ #
    # FRAME
    # ------------------------------------------------------------------ #

    @Part
    def frame(self):
        return Frame(pos=self._engine_position, hidden=False)


# ---------------------------------------------------------------------- #
# TEST
# ---------------------------------------------------------------------- #

if __name__ == "__main__":
    from parapy.gui import display

    # Single centreline tractor prop on a medium UAV (~100 kg)
    prop = PropellerEngine(
        label="prop_engine",

        cruise_speed=50,
        mtow=100.0,
        n_engines=1,
        thrust_to_weight=0.35,
        rho=1.225,
        g=9.81,

        semi_span=4.0,
        sweep_le=5.0,
        dihedral=3.0,
        fuselage_radius=0.20,

        attach_spanwise_pct=0.0,
        attach_x_offset=-0.30,
        attach_z_offset=0.0,

        disk_loading_uav=80.0,
        target_solidity=0.15,
        
        inlet_radius_ratio=0.85,
        nozzle_radius_ratio=0.7,

        nacelle_length_override=None,
        nacelle_radius_override=None,

        n_blades_override=None,
        blade_length_override=None,
        blade_root_chord_override=None,

        blade_sweep=5.0,

        taper_sections=8,
        color_nacelle="Silver",
        color_spinner="White",
        color_blade="DarkGray",
    )

    display(prop)