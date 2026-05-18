from math import radians, tan

from parapy.core import Input, Attribute, Part, child
from parapy.geom import GeomBase, LoftedSolid, Circle, translate, rotate, Vector

from Pythonfiles.Components.Frame import Frame


class JetEngine(GeomBase):
    """
    Turbofan/turbojet nacelle with Roskam-based sizing.

    Sizing relations (Roskam Vol. V, §4 / Vol. I §3.2):
    """

    # ------------------------------------------------------------------ #
    # PRIMARY INPUTS
    # ------------------------------------------------------------------ #

    engine_type: str = Input("jet")

    mtow: float = Input()
    n_engines: int = Input()
    thrust_to_weight: float = Input()

    rho: float = Input()
    g: float = Input()

    cruise_speed: float = Input()   # [m/s]

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
    # OVERRIDES
    # ------------------------------------------------------------------ #

    nacelle_length_override: float = Input()
    nacelle_radius_override: float = Input()

    # ------------------------------------------------------------------ #
    # DISPLAY CONTROL
    # ------------------------------------------------------------------ #

    taper_sections: int = Input(8)
    color_nacelle: str = Input("Silver")
    color_fan: str = Input("DarkGray")
    color_nozzle: str = Input("Gray")
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
        """Required installed thrust per engine at cruise altitude [N]."""
        return self.total_thrust / self.n_engines

    # ------------------------------------------------------------------ #
    # ALTITUDE THRUST LAPSE
    # ------------------------------------------------------------------ #

    @Attribute
    def thrust_lapse_factor(self) -> float:
        """
        Fraction of sea-level thrust available at cruise altitude [-].

        For a turbofan/turbojet, thrust falls with air density as:
            T_alt / T_sl ≈ (ρ_alt / ρ_sl)^0.75

        (Raymer §13.3; Roskam Vol. V §5.2).  Turbojets use ~0.9 but 0.75
        is the conservative design assumption for mixed fleets.
        """
        sigma = max(self.rho, 0.01) / 1.225   # density ratio (clamp away from zero)
        return min(sigma ** 0.75, 1.0)         # cap at 1.0 (no lapse below sea level)

    @Attribute
    def sl_thrust_per_engine(self) -> float:
        """
        Sea-level rated thrust per engine [N].
        """
        return self.thrust_per_engine / self.thrust_lapse_factor

    # ------------------------------------------------------------------ #
    # ROSKAM NACELLE SIZING  (Vol. V §4)
    # ------------------------------------------------------------------ #

    @Attribute
    def nacelle_radius(self) -> float:
        """
        Nacelle outer radius [m] — Roskam Vol. V §4 correlation.
        """
        if self.nacelle_radius_override is not None:
            return self.nacelle_radius_override
        T_kN = self.sl_thrust_per_engine / 1000.0   # ← sea-level equivalent
        return 0.2284 * (T_kN ** 0.4) / 2.0

    @Attribute
    def nacelle_length(self) -> float:
        if self.nacelle_length_override is not None:
            return self.nacelle_length_override
        return 2.5 * (2.0 * self.nacelle_radius)

    # ------------------------------------------------------------------ #
    # ROSKAM INTERNAL SIZING  (Vol. I §3.2)
    # ------------------------------------------------------------------ #

    @Attribute
    def bypass_ratio(self) -> float:
        bpr = 15.0 * (1.0 - self.thrust_to_weight) ** 2
        return max(1.0, min(12.0, bpr))

    @Attribute
    def fan_radius(self) -> float:
        return 0.90 * self.nacelle_radius

    @Attribute
    def core_radius(self) -> float:
        return self.fan_radius * 0.40

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
    # PARTS — FAN
    # ------------------------------------------------------------------ #

    @Attribute
    def _fan_x(self) -> float:
        return self.nacelle_length * 0.15

    @Part
    def fan_profiles(self):
        return Circle(
            quantify=2,
            radius=self.fan_radius,
            color=self.color_fan,
            position=translate(
                self._engine_base,
                Vector(1, 0, 0), self._fan_x + child.index * 0.05,
            ).rotate90('y'),
        )

    @Part
    def fan_disk(self):
        return LoftedSolid(
            profiles=self.fan_profiles,
            color=self.color_fan,
            mesh_deflection=self.mesh_deflection,
        )

    # ------------------------------------------------------------------ #
    # PARTS — EXHAUST
    # ------------------------------------------------------------------ #

    @Attribute
    def _nozzle_x_positions(self) -> list:
        return [self.nacelle_length * 0.78, self.nacelle_length]

    @Part
    def nozzle_profiles(self):
        return Circle(
            quantify=2,
            radius=[self.core_radius, 0.01][child.index],
            color=self.color_nozzle,
            position=translate(
                self._engine_base,
                Vector(1, 0, 0), self._nozzle_x_positions[child.index],
            ).rotate90('y'),
        )

    @Part
    def exhaust_plug(self):
        return LoftedSolid(
            profiles=self.nozzle_profiles,
            color=self.color_nozzle,
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

    # Twin underwing turbofan on a medium UAV
    jet = JetEngine(
        label="jet_engine",

        mtow=5000.0,
        n_engines=2,
        thrust_to_weight=0.40,
        rho=1.225,
        g=9.81,

        semi_span=8.0,
        sweep_le=15.0,
        dihedral=3.0,
        fuselage_radius=0.70,

        attach_spanwise_pct=0.35,
        attach_x_offset=0.20,
        attach_z_offset=-0.10,
        
        inlet_radius_ratio=0.85,
        nozzle_radius_ratio=0.7,

        nacelle_length_override=None,
        nacelle_radius_override=None,

        taper_sections=10,
        color_nacelle="Silver",
        color_fan="DarkGray",
        color_nozzle="Gray",
    )

    display(jet)