from math import radians, tan, sin, cos, pi, sqrt

from parapy.core import Input, Attribute, Part, child
from parapy.geom import (
    GeomBase, LoftedSolid,
    Circle, translate, rotate, Vector,
)

from Pythonfiles.Components.Frame import Frame


class PropellerEngine(GeomBase):
    """
    Propeller (tractor/pusher) engine with Roskam-based sizing.

    Sizing relations (Roskam Vol. I, §3.6 + actuator-disk theory):
    ─────────────────────────────────────────────────────────────────────
    Thrust:         T_total = T/W * MTOW * g
                    T_eng   = T_total / n_engines

    Disk area:      A = T_eng / DL,  DL = disk_loading_uav  [N/m²]
    Shaft power:    P = T * sqrt(T / (2 * rho * A))          [momentum theory]
    Prop diameter:  D_prop = 0.658 * P_kW^0.25              [Roskam §3.6]
                    blade_length = D_prop / 2, capped at _max_blade_length

    Blade chord:    c_base = 0.065 * D_prop                  [Roskam §3.6, 2-blade ref]
                    c_root = c_base * n_blades / 2           [scaled to keep σ ≈ target]
                    c_tip  = 0.30 * c_root

    Blade count:    σ = n * c / (π * R)  → n = σ * π * R / c_base
                    clamped to [2, 6]

    Spinner:        r_spinner = 0.15 * D_prop
                    L_spinner = 1.5 * r_spinner

    Nacelle:        nacelle_radius = spinner_radius  (motor cowl, not thrust-sized)
                    nacelle_length = 3.5 * nacelle_radius
    ─────────────────────────────────────────────────────────────────────
    """

    # ------------------------------------------------------------------ #
    # PRIMARY INPUTS
    # ------------------------------------------------------------------ #

    mtow: float = Input()
    n_engines: int = Input()
    thrust_to_weight: float = Input()

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
    def shaft_power(self) -> float:
        T = self.thrust_per_engine
        return T * sqrt(T / (2.0 * self.rho * self._disk_area))

    @Attribute
    def _max_blade_length(self) -> float:
        wing_limit = self.semi_span * 0.15
        fus_limit  = self.fuselage_radius * 1.5 if self.fuselage_radius > 0 else wing_limit
        if self.attach_spanwise_pct * self.semi_span < 0.10 * self.semi_span:
            return max(fus_limit, 0.10)
        return wing_limit

    @Attribute
    def blade_length(self) -> float:
        if self.blade_length_override is not None:
            return self.blade_length_override
        P_kW = self.shaft_power / 1000.0
        D_roskam = 0.658 * (P_kW ** 0.25)
        return min(D_roskam / 2.0, self._max_blade_length)

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
        D_prop = 2.0 * R
        c_base = 0.065 * D_prop
        if c_base <= 0:
            return 2
        n_raw = self.target_solidity * pi * R / c_base
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
        """Translated-only position (no rotation) — used to place profiles
        so that after rotate90('y') their Z-normal aligns with global X."""
        return translate(
            self.position,
            Vector(1, 0, 0), self._attach_x,
            Vector(0, 1, 0), self._attach_y,
            Vector(0, 0, 1), self._attach_z,
        )

    @Attribute
    def _engine_position(self):
        """Fully oriented engine frame (rotated) — used for the Frame part."""
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
                rotate(self._engine_base, 'x', radians(a)),
                Vector(0, sin(radians(a)), cos(radians(a))), self.spinner_radius,
            ).rotate90('y')
            for a in self._blade_angles_deg
        ]

    @Attribute
    def _blade_tip_positions(self) -> list:
        x_sweep = self.blade_length * tan(radians(self.blade_sweep))
        return [
            translate(
                rotate(self._engine_base, 'x', radians(a)),
                Vector(1, 0, 0), x_sweep,
                Vector(0, sin(radians(a)), cos(radians(a))),
                self.spinner_radius + self.blade_length,
            ).rotate90('y')
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