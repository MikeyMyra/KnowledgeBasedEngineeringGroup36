import sys
import os
from math import radians, tan, sin, cos, pi, sqrt

from parapy.core import Input, Attribute, Part, child
from parapy.geom import (
    GeomBase, LoftedSolid,
    Circle, translate, rotate, Vector, Point
)

from Pythonfiles.Components.Frame import Frame


# ======================================================================
# INTERNAL SUBCLASSES  (prefixed _ : not instantiated directly)
# ======================================================================

class _EngineGeometry(GeomBase):
    """Shared nacelle geometry used by both jet and propeller subclasses.

    Receives pre-computed sizing values from the Engine parent — no
    Roskam calculations live here, just geometry.
    """

    nacelle_length: float = Input()
    nacelle_radius: float = Input()
    inlet_radius_ratio: float = Input(0.85)
    nozzle_radius_ratio: float = Input(0.70)
    taper_sections: int = Input(8)
    color_nacelle: str = Input("Silver")
    mesh_deflection: float = Input(1e-4)

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

    @Part
    def nacelle_profiles(self):
        return Circle(
            quantify=self.taper_sections,
            radius=self._nacelle_radii[child.index],
            color=self.color_nacelle,
            position=translate(
                self.position,
                Vector(1, 0, 0), self._nacelle_x_positions[child.index],
            ),
        )

    @Part
    def nacelle(self):
        return LoftedSolid(
            profiles=self.nacelle_profiles,
            color=self.color_nacelle,
            transparency=0.3,
            mesh_deflection=self.mesh_deflection,
        )


class _JetEngine(_EngineGeometry):
    """Turbofan/turbojet geometry: intake ring + fan disk + exhaust plug."""

    fan_radius: float = Input()
    core_radius: float = Input()
    color_fan: str = Input("DarkGray")
    color_nozzle: str = Input("Gray")

    @Part
    def intake_ring(self):
        return Circle(
            radius=self.nacelle_radius * self.inlet_radius_ratio,
            color="White",
            position=self.position,
        )

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
                self.position,
                Vector(1, 0, 0), self._fan_x + child.index * 0.05,
            ),
        )

    @Part
    def fan_disk(self):
        return LoftedSolid(
            profiles=self.fan_profiles,
            color=self.color_fan,
            mesh_deflection=self.mesh_deflection,
        )

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
                self.position,
                Vector(1, 0, 0), self._nozzle_x_positions[child.index],
            ),
        )

    @Part
    def exhaust_plug(self):
        return LoftedSolid(
            profiles=self.nozzle_profiles,
            color=self.color_nozzle,
            mesh_deflection=self.mesh_deflection,
        )


class _PropellerEngine(_EngineGeometry):
    """Propeller engine geometry: spinner + blades."""

    n_blades: int = Input(3)
    blade_length: float = Input()
    blade_root_chord: float = Input()
    blade_tip_chord: float = Input()
    blade_sweep: float = Input(5.0)
    spinner_radius: float = Input()
    spinner_length: float = Input()
    color_spinner: str = Input("White")
    color_blade: str = Input("DarkGray")

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
                self.position,
                Vector(1, 0, 0),
                -self.spinner_length + self._spinner_x_positions[child.index],
            ),
        )

    @Part
    def spinner(self):
        return LoftedSolid(
            profiles=self.spinner_profiles,
            color=self.color_spinner,
            mesh_deflection=self.mesh_deflection,
        )

    @Attribute
    def _blade_angles_deg(self) -> list:
        return [360.0 / self.n_blades * i for i in range(self.n_blades)]

    @Attribute
    def _blade_root_positions(self) -> list:
        return [
            translate(
                rotate(self.position, 'x', radians(a)),
                Vector(0, sin(radians(a)), cos(radians(a))), self.spinner_radius,
            )
            for a in self._blade_angles_deg
        ]

    @Attribute
    def _blade_tip_positions(self) -> list:
        x_sweep = self.blade_length * tan(radians(self.blade_sweep))
        return [
            translate(
                rotate(self.position, 'x', radians(a)),
                Vector(1, 0, 0), x_sweep,
                Vector(0, sin(radians(a)), cos(radians(a))),
                self.spinner_radius + self.blade_length,
            )
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


# ======================================================================
# ENGINE  —  public class, instantiated by Aircraft
# ======================================================================

class Engine(GeomBase):
    """Engine assembly with Roskam-based sizing and internal jet/prop selection.

    Sizing relations (Roskam Vol. I / Vol. V §4):
    ─────────────────────────────────────────────────────────────────────
    Thrust:        T_total = T/W * MTOW * g
                   T_eng   = T_total / n_engines

    Jet nacelle    Roskam Vol. V, §4 — diameter scales with thrust:
                   D_nac = k_D * (T_eng_kN)^0.4,  k_D=0.2284
                   L_nac = 2.5 * D_nac

    Propeller      Roskam Vol. I §3.6 + actuator disk theory:
                   A_disk = T / DL,  DL=80 N/m² (UAV fixed-wing, Table 3.6)
                   P_shaft = T * sqrt(T / (2*rho*A_disk))   [momentum theory]
                   D_prop  = 0.658 * P_kW^0.25              [Roskam §3.6]
                   D_prop capped so blade_length <= max_blade_length

    Prop nacelle   Sized from spinner, NOT from thrust (a prop nacelle is just
                   a motor cowling; the Roskam jet-nacelle formula is not valid):
                   spinner_radius = 0.15 * D_prop   (15% of prop diameter)
                   nacelle_radius = spinner_radius   (nacelle wraps the spinner)
                   nacelle_length = 3.5 * nacelle_radius  (slender motor cowl)

    Blade count    Roskam Vol. I §3.6: disk solidity σ = n*c/(π*R)
                   Target σ = 0.15 (statistical midpoint for UAV fixed-wing props)
                   n = round(σ * π * R / c_blade), clamped to [2, 6]
                   NOTE: override with n_blades_override if needed.

    Blade chord    Roskam Vol. I §3.6: c = 0.065 * D_prop for σ ≈ 0.10
                   Scaled up proportionally when n_blades > 2 to hold σ ≈ 0.15.

    Max blade      For a centreline tractor (n_engines=1, attach_spanwise_pct=0),
    length         clearance is limited by the fuselage radius, not semi_span.
                   max_blade_length = max(fuselage_radius * 1.5, semi_span * 0.15)
                   This prevents the old bug where a centreline prop got D_max=0.
    ─────────────────────────────────────────────────────────────────────
    """

    # ============================================================ #
    # ENGINE CORE
    # ============================================================ #

    engine_type: str = Input("propeller")

    mtow: float = Input()
    n_engines: int = Input(1)
    thrust_to_weight: float = Input(0.35)

    rho: float = Input(1.225)
    g: float = Input(9.81)

    # ============================================================ #
    # GEOMETRY CONTEXT (FROM AIRCRAFT, NOT WING OBJECTS)
    # ============================================================ #

    semi_span: float = Input()
    sweep_le: float = Input(5.0)
    dihedral: float = Input(5.0)

    fuselage_radius: float = Input(0.0)

    # ============================================================ #
    # POSITIONING
    # ============================================================ #

    attach_spanwise_pct: float = Input(0.35)
    attach_x_offset: float = Input(0.0)
    attach_z_offset: float = Input(0.0)

    # ============================================================ #
    # DESIGN CONTROL
    # ============================================================ #

    disk_loading_uav: float = Input(80.0)
    target_solidity: float = Input(0.15)

    # ============================================================ #
    # OVERRIDES (GEOMETRY ONLY)
    # ============================================================ #

    nacelle_length_override: float = Input(None)
    nacelle_radius_override: float = Input(None)

    n_blades_override: int = Input(None)
    blade_length_override: float = Input(None)
    blade_root_chord_override: float = Input(None)

    blade_sweep: float = Input(5.0)

    taper_sections: int = Input(8)
    color_nacelle: str = Input("Silver")

    mesh_deflection: float = Input(1e-4)

    # ------------------------------------------------------------------ #
    # ROSKAM THRUST SIZING
    # ------------------------------------------------------------------ #

    @Attribute
    def total_thrust(self) -> float:
        """Total installed thrust [N]. Roskam Vol. I, §3.2."""
        return self.thrust_to_weight * self.mtow * self.g

    @Attribute
    def thrust_per_engine(self) -> float:
        return self.total_thrust / self.n_engines

    # ------------------------------------------------------------------ #
    # ROSKAM PROPELLER SIZING  (Vol. I, §3.6)
    # ------------------------------------------------------------------ #

    @Attribute
    def _disk_area(self) -> float:
        """Actuator disk area from UAV disk loading [m²].

        Roskam Vol. I, Table 3.6: A = T / DL, DL = 80 N/m² for UAV fixed-wing.
        This directly seeds the momentum-theory shaft power calculation and is
        independent of nacelle geometry — avoids the previous circular dependency.
        """
        return self.thrust_per_engine / self.disk_loading_uav

    @Attribute
    def shaft_power(self) -> float:
        """Shaft power from actuator-disk momentum theory [W].

        P = T * sqrt(T / (2 * rho * A_disk))  — static thrust, ideal disk.
        """
        T = self.thrust_per_engine
        return T * sqrt(T / (2.0 * self.rho * self._disk_area))

    @Attribute
    def _max_blade_length(self) -> float:
        """Maximum allowable blade (prop radius) for geometric clearance [m].

        For a centreline tractor (single engine, attach_spanwise_pct ≈ 0) the
        limiting dimension is fuselage clearance, not wing semi_span.
        For a wing-mounted engine the limiting dimension is 15% of semi_span.

        The previous implementation used only semi_span * 0.15, which collapsed
        to zero for a centreline prop (attach_y = 0 → D_max = 0).
        """
        wing_limit = self.semi_span * 0.15
        # For centreline or near-centreline props, fall back to fuselage clearance.
        # fuselage_radius * 1.5 gives a reasonable 50% tip-clearance margin.
        fus_limit  = self.fuselage_radius * 1.5 if self.fuselage_radius > 0 else wing_limit
        # Use fuselage limit when spanwise attachment is less than 10% semi_span
        if self.attach_spanwise_pct * self.semi_span < 0.10 * self.semi_span:
            return max(fus_limit, 0.10)     # hard floor: at least 10 cm radius
        return wing_limit

    @Attribute
    def blade_length(self) -> float:
        """Propeller blade length (= prop radius) [m].

        Roskam Vol. I, §3.6: D_prop = 0.658 * P_kW^0.25 for GA propellers.
        Capped at _max_blade_length to maintain wing / fuselage clearance.
        NOTE (Roskam): select nearest standard prop diameter from catalogue.
        """
        if self.blade_length_override is not None:
            return self.blade_length_override
        P_kW = self.shaft_power / 1000.0
        D_roskam = 0.658 * (P_kW ** 0.25)
        return min(D_roskam / 2.0, self._max_blade_length)

    @Attribute
    def blade_root_chord(self) -> float:
        """Blade root chord [m].

        Roskam Vol. I, §3.6 baseline: c = 0.065 * D_prop  (σ ≈ 0.10 for 2 blades).
        Scaled by n_blades / 2 so that disk solidity stays near target_solidity
        regardless of blade count.
        NOTE (Roskam): verify chord against manufacturer data for final design.
        """
        if self.blade_root_chord_override is not None:
            return self.blade_root_chord_override
        D_prop = 2.0 * self.blade_length
        c_base = 0.065 * D_prop                 # Roskam §3.6 baseline (2 blades)
        return c_base * (self.n_blades / 2.0)   # scale to keep solidity consistent

    @Attribute
    def blade_tip_chord(self) -> float:
        """Blade tip chord = 30% of root chord (standard taper for UAV props)."""
        return self.blade_root_chord * 0.30

    @Attribute
    def n_blades(self) -> int:
        """Blade count from Roskam Vol. I §3.6 disk solidity relation.

        σ = n * c_blade / (π * R)
        → n = σ * π * R / c_blade

        where c_blade is the Roskam baseline chord (2-blade reference),
        R = blade_length, σ = target_solidity (default 0.15).
        Clamped to [2, 6] — practical range for fixed-wing UAV propellers.
        NOTE (Roskam): 2 blades for low-drag cruise, 4-6 for compact / high-power.
        Override with n_blades_override if a specific blade count is required.
        """
        if self.n_blades_override is not None:
            return self.n_blades_override

        R = self.blade_length
        D_prop = 2.0 * R
        c_base = 0.065 * D_prop         # Roskam §3.6 baseline chord (2-blade ref)
        if c_base <= 0:
            return 2
        n_raw = self.target_solidity * pi * R / c_base
        return max(2, min(6, round(n_raw)))

    # ------------------------------------------------------------------ #
    # PROPELLER NACELLE SIZING
    # A prop nacelle is a motor cowling — it wraps the spinner, not the disk.
    # The jet-nacelle Roskam thrust formula gives a wildly oversized cowl here.
    # ------------------------------------------------------------------ #

    @Attribute
    def spinner_radius(self) -> float:
        """Spinner radius = 15% of prop diameter [m].

        Roskam Vol. I §3.6: spinner diameter is typically 12–18% of prop diameter
        for UAV/GA tractor propellers. 15% is the statistical midpoint.
        """
        return 0.15 * (2.0 * self.blade_length)

    @Attribute
    def spinner_length(self) -> float:
        """Spinner length = 1.5 * spinner_radius (ellipsoidal nose profile)."""
        return 1.5 * self.spinner_radius

    # ------------------------------------------------------------------ #
    # ROSKAM JET NACELLE SIZING  (Vol. V, §4)
    # Only used for jet engines.
    # ------------------------------------------------------------------ #

    @Attribute
    def _jet_nacelle_radius(self) -> float:
        """Jet nacelle outer radius [m]. Roskam Vol. V, §4: D = 0.2284 * T_kN^0.4."""
        T_kN = self.thrust_per_engine / 1000.0
        return 0.2284 * (T_kN ** 0.4) / 2.0

    @Attribute
    def _jet_nacelle_length(self) -> float:
        """Jet nacelle length [m]. Roskam Vol. V, §4: L = 2.5 * D_nac."""
        return 2.5 * (2.0 * self._jet_nacelle_radius)

    # ------------------------------------------------------------------ #
    # RESOLVED NACELLE DIMENSIONS (type-aware)
    # ------------------------------------------------------------------ #

    @Attribute
    def nacelle_radius(self) -> float:
        """Nacelle outer radius [m].

        Jet:  Roskam Vol. V §4 thrust-based formula.
        Prop: equals spinner_radius (motor cowl, not thrust-sized).
        Override with nacelle_radius_override if needed.
        """
        if self.nacelle_radius_override is not None:
            return self.nacelle_radius_override
        if self.engine_type == "jet":
            return self._jet_nacelle_radius
        # Propeller: nacelle is just a motor cowling around the spinner
        return self.spinner_radius

    @Attribute
    def nacelle_length(self) -> float:
        """Nacelle length [m].

        Jet:  Roskam Vol. V §4: L = 2.5 * D_nac.
        Prop: L = 3.5 * nacelle_radius (slender motor cowl).
        Override with nacelle_length_override if needed.
        """
        if self.nacelle_length_override is not None:
            return self.nacelle_length_override
        if self.engine_type == "jet":
            return self._jet_nacelle_length
        return 3.5 * self.nacelle_radius

    # ------------------------------------------------------------------ #
    # ROSKAM JET INTERNAL SIZING
    # ------------------------------------------------------------------ #

    @Attribute
    def bypass_ratio(self) -> float:
        """BPR estimate. Roskam Vol. I §3.2: BPR = 15*(1-T/W)^2, clamped [1,12]."""
        bpr = 15.0 * (1.0 - self.thrust_to_weight) ** 2
        return max(1.0, min(12.0, bpr))

    @Attribute
    def fan_radius(self) -> float:
        """Fan tip radius = 90% of jet nacelle radius."""
        return 0.90 * self._jet_nacelle_radius

    @Attribute
    def core_radius(self) -> float:
        """Core radius = 40% of fan radius."""
        return self.fan_radius * 0.40

    # ------------------------------------------------------------------ #
    # ATTACH POSITION
    # ------------------------------------------------------------------ #

    @Attribute
    def _attach_y(self) -> float:
        """Spanwise attach position [m]."""
        return 0.0 if self.n_engines == 1 else self.semi_span * self.attach_spanwise_pct

    @Attribute
    def _attach_x(self) -> float:
        if self.n_engines == 1:
            return 0
        return (self.wing_root_x
                + self._attach_y * tan(radians(self.sweep_le))
                + self.attach_x_offset)

    @Attribute
    def _attach_z(self) -> float:
        if self.n_engines == 1:
            return 0
        return (self.wing_root_z
                + self._attach_y * sin(radians(self.dihedral))
                + self.attach_z_offset)

    @Attribute
    def _engine_position(self):
        """Engine position: translate first in fuselage frame, then orient."""
        return translate(
            self.position,
            Vector(1, 0, 0), self._attach_x,
            Vector(0, 1, 0), self._attach_y,
            Vector(0, 0, 1), self._attach_z,
        ).rotate90('y')

    # ------------------------------------------------------------------ #
    # ENGINE INSTANCE
    # ------------------------------------------------------------------ #

    @Part(parse=False)
    def engine_instance(self):
        shared = dict(
            nacelle_length=self.nacelle_length,
            nacelle_radius=self.nacelle_radius,
            taper_sections=self.taper_sections,
            color_nacelle=self.color_nacelle,
            mesh_deflection=self.mesh_deflection,
            position=self._engine_position,
        )
        if self.engine_type == "jet":
            return _JetEngine(
                **shared,
                fan_radius=self.fan_radius,
                core_radius=self.core_radius,
            )
        return _PropellerEngine(
            **shared,
            n_blades=self.n_blades,
            blade_length=self.blade_length,
            blade_root_chord=self.blade_root_chord,
            blade_tip_chord=self.blade_tip_chord,
            blade_sweep=self.blade_sweep,
            spinner_radius=self.spinner_radius,
            spinner_length=self.spinner_length,
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

    # ============================================================ #
    # PROPELLER ENGINE TEST CASE (FULL INPUT TRACEABILITY)
    # ============================================================ #

    prop_engine = Engine(

        # ---------------------- TYPE ------------------------------ #
        engine_type="propeller",

        # ---------------------- GLOBAL ---------------------------- #
        mtow=1000.0,
        n_engines=1,
        thrust_to_weight=0.35,
        rho=1.225,
        g=9.81,

        # ---------------------- WING INTERFACE ------------------- #
        semi_span=8.0,
        sweep_le=5.0,
        dihedral=5.0,
        fuselage_radius=0.625,

        attach_spanwise_pct=0.0,
        attach_x_offset=-0.5,
        attach_z_offset=0.0,

        # ---------------------- NACELLE OVERRIDES --------------- #
        nacelle_length_override=None,
        nacelle_radius_override=None,

        # ---------------------- PROPELLER ------------------------ #
        n_blades_override=None,
        blade_length_override=None,
        blade_root_chord_override=None,

        blade_sweep=5.0,

        # ---------------------- DISK / ROSKAM ------------------- #
        disk_loading_uav=80.0,

        # ---------------------- GEOMETRY CONTROL ----------------- #
        taper_sections=8,
        color_nacelle="Silver",

        mesh_deflection=1e-4,
    )

    # ============================================================ #
    # JET ENGINE TEST CASE (FULL INPUT TRACEABILITY)
    # ============================================================ #

    jet_engine = Engine(

        # ---------------------- TYPE ------------------------------ #
        engine_type="jet",

        # ---------------------- GLOBAL ---------------------------- #
        mtow=1000.0,
        n_engines=2,
        thrust_to_weight=0.40,
        rho=1.225,
        g=9.81,

        # ---------------------- WING INTERFACE ------------------- #
        semi_span=8.0,
        sweep_le=15.0,
        dihedral=3.0,
        fuselage_radius=0.7,

        attach_spanwise_pct=0.35,
        attach_x_offset=0.2,
        attach_z_offset=0.1,

        # ---------------------- NACELLE OVERRIDES --------------- #
        nacelle_length_override=None,
        nacelle_radius_override=None,

        # ---------------------- JET-SPECIFIC INPUTS ------------- #
        n_blades_override=None,
        blade_length_override=None,
        blade_root_chord_override=None,

        blade_sweep=0.0,

        # ---------------------- DISK / ROSKAM ------------------- #
        disk_loading_uav=80.0,

        # ---------------------- GEOMETRY CONTROL ----------------- #
        taper_sections=10,
        color_nacelle="Silver",

        mesh_deflection=1e-4,
    )

    display([prop_engine, jet_engine])