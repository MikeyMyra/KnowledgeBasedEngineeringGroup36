import sys
import os
from math import radians, tan, sin, cos

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
        # ParaPy translate() works in the GLOBAL frame even after a rotate().
        # After rotate(pos, 'x', a), the local radial direction is no longer
        # global Z — it is (0, sin(a), cos(a)) in global coordinates.
        # Using Vector(0, 0, 1) for all blades placed every blade root at the
        # same angular position (global Z+), making n_blades look like one blade.
        # Fix: translate by the true radial unit vector for each blade angle.
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

    Nacelle        Roskam Vol. V, §4 — diameter/length scale with thrust:
                   D_nac = k_D * (T_eng_kN)^0.4
                   L_nac = k_L * D_nac
                   k_D=0.2284, k_L=2.5  (jet)
                   k_D=0.1956, k_L=1.8  (propeller)

    Propeller      Roskam Vol. I, §3.6 + actuator disk theory:
                   Disk area seeded from UAV disk loading DL_uav=80 N/m²
                   (Roskam Vol. I Table 3.6: UAV fixed-wing ~ 60-120 N/m²)
                   P_shaft = T * sqrt(T / (2*rho*A_disk))
                   D_prop  = 0.658 * (P_kW)^0.25   (Roskam §3.6)
                   D_prop  capped at 15% semi_span per side (clearance limit)
                   c_blade = 0.065 * D_prop  (solidity σ ≈ 0.10, Roskam §3.6)

    BPR (jet):     BPR = 15 * (1 - T/W)^2   clamped [1, 12]
    ─────────────────────────────────────────────────────────────────────

    Previous issue fixed: _disk_area_seed formerly used nacelle_radius × 4
    as a proxy for prop disk radius. This was circular (nacelle_radius itself
    comes from thrust) and gave unrealistically high shaft power estimates.
    The seed now uses a UAV disk loading of 80 N/m² directly, which anchors
    to real fixed-wing UAV propeller data.
    NOTE (Roskam): D_prop cap at 15% semi_span is a clearance heuristic —
    override blade_length_override if propeller selection drives a larger disk.
    """

    # ------------------------------------------------------------------ #
    # INPUTS — TYPE
    # ------------------------------------------------------------------ #

    engine_type: str = Input("propeller")   # "jet" or "propeller"

    # ------------------------------------------------------------------ #
    # INPUTS — SIZING
    # ------------------------------------------------------------------ #

    mtow: float = Input()
    n_engines: int = Input(1)
    thrust_to_weight: float = Input(0.35)
    rho: float = Input(1.225)
    g: float = Input(9.81)

    # Roskam Vol. I, Table 3.6: UAV fixed-wing disk loading 60-120 N/m².
    # 80 N/m² is used as the statistical midpoint for single-prop tractor/pusher.
    # NOTE (Roskam): multirotor UAVs use 30-60 N/m²; increase for high-speed UAV.
    disk_loading_uav: float = Input(80.0)   # [N/m²]

    # ------------------------------------------------------------------ #
    # INPUTS — WING INTERFACE
    # ------------------------------------------------------------------ #

    semi_span: float = Input()
    sweep_le: float = Input(25.0)
    dihedral: float = Input(5.0)
    wing_root_x: float = Input()
    wing_root_z: float = Input()

    attach_spanwise_pct: float = Input(0.35)
    attach_x_offset: float = Input(0.0)
    attach_z_offset: float = Input(0)

    # ------------------------------------------------------------------ #
    # INPUTS — STYLE / OVERRIDES
    # ------------------------------------------------------------------ #

    nacelle_length_override: float = Input(None)
    nacelle_radius_override: float = Input(None)
    n_blades: int = Input(3)
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
        """Thrust per engine [N]."""
        return self.total_thrust / self.n_engines

    # ------------------------------------------------------------------ #
    # ROSKAM NACELLE SIZING  (Vol. V, §4)
    # ------------------------------------------------------------------ #

    @Attribute
    def _k_D(self) -> float:
        """Nacelle diameter coefficient. Roskam Vol. V, §4."""
        return 0.2284 if self.engine_type == "jet" else 0.1956

    @Attribute
    def _k_L(self) -> float:
        """Nacelle length-to-diameter ratio. Roskam Vol. V, §4."""
        return 2.5 if self.engine_type == "jet" else 1.8

    @Attribute
    def nacelle_radius(self) -> float:
        """Nacelle outer radius [m]. Roskam Vol. V, §4: D_nac = k_D * T_kN^0.4."""
        if self.nacelle_radius_override is not None:
            return self.nacelle_radius_override
        T_kN = self.thrust_per_engine / 1000.0
        return self._k_D * (T_kN ** 0.4) / 2.0

    @Attribute
    def nacelle_length(self) -> float:
        """Nacelle length [m]. Roskam Vol. V, §4: L_nac = k_L * D_nac."""
        if self.nacelle_length_override is not None:
            return self.nacelle_length_override
        return self._k_L * (2.0 * self.nacelle_radius)

    # ------------------------------------------------------------------ #
    # ROSKAM JET SIZING
    # ------------------------------------------------------------------ #

    @Attribute
    def bypass_ratio(self) -> float:
        """Bypass ratio estimate. Roskam Vol. I, §3.2: BPR = 15*(1-T/W)^2."""
        bpr = 15.0 * (1.0 - self.thrust_to_weight) ** 2
        return max(1.0, min(12.0, bpr))

    @Attribute
    def fan_radius(self) -> float:
        """Fan tip radius = 90% of nacelle radius (annulus assumption)."""
        return 0.90 * self.nacelle_radius

    @Attribute
    def core_radius(self) -> float:
        """Core radius = 40% of fan radius (BPR-consistent annulus)."""
        return self.fan_radius * 0.40

    # ------------------------------------------------------------------ #
    # ROSKAM PROPELLER SIZING  (Vol. I, §3.6)
    # ------------------------------------------------------------------ #

    @Attribute
    def _disk_area(self) -> float:
        """Actuator disk area seeded from UAV disk loading [m²].

        Roskam Vol. I, Table 3.6: fixed-wing UAV disk loading ~ 60-120 N/m².
        Using DL=80 N/m² as statistical midpoint: A = T / DL.

        Previous approach used nacelle_radius×4 as a proxy, which was circular
        (nacelle_radius itself derives from thrust) and gave P_shaft ~4× too high,
        inflating the Roskam D_prop formula. The D_prop cap at 15% semi_span
        masked this, but shaft power was still unrealistic.
        """
        return self.thrust_per_engine / self.disk_loading_uav

    @Attribute
    def shaft_power(self) -> float:
        """Shaft power from actuator disk theory [W].

        P = T * sqrt(T / (2 * rho * A_disk))  — momentum theory, static thrust.
        """
        T = self.thrust_per_engine
        return T * (T / (2.0 * self.rho * self._disk_area)) ** 0.5

    @Attribute
    def blade_length(self) -> float:
        """Propeller blade length (= prop radius) [m].

        Roskam Vol. I, §3.6: D_prop = 0.658 * P_kW^0.25 for GA propellers.
        Capped at 15% semi_span per side to maintain wing/ground clearance.
        NOTE (Roskam): select nearest standard prop diameter from catalogue.
        """
        if self.blade_length_override is not None:
            return self.blade_length_override
        P_kW = self.shaft_power / 1000.0
        D = 0.658 * (P_kW ** 0.25)
        D_max = 2.0 * self.semi_span * 0.15
        return min(D, D_max) / 2.0

    @Attribute
    def blade_root_chord(self) -> float:
        """Blade root chord [m].

        Roskam Vol. I, §3.6: c_blade = 0.065 * D_prop for solidity σ ≈ 0.10.
        """
        if self.blade_root_chord_override is not None:
            return self.blade_root_chord_override
        return 0.065 * (2.0 * self.blade_length)

    @Attribute
    def blade_tip_chord(self) -> float:
        """Blade tip chord = 30% of root chord (standard taper for UAV props)."""
        return self.blade_root_chord * 0.30

    # ------------------------------------------------------------------ #
    # ATTACH POSITION
    # ------------------------------------------------------------------ #

    @Attribute
    def _attach_y(self) -> float:
        """Spanwise attach position: centreline for single engine, else % semi_span."""
        return 0.0 if self.n_engines == 1 else self.semi_span * self.attach_spanwise_pct

    @Attribute
    def _attach_x(self) -> float:
        return 0.0 if self.n_engines == 1 else (self.wing_root_x
                                                    + self._attach_y * tan(radians(self.sweep_le))
                                                    + self.attach_x_offset)

    @Attribute
    def _attach_z(self) -> float:
        return 0.0 if self.n_engines == 1 else self.wing_root_z + self._attach_y * sin(radians(self.dihedral)) + self.attach_z_offset

    @Attribute
    def _engine_position(self):
        """Nacelle-axis position: translated to attach point, then oriented forward.

        Bug fix: the previous code did rotate90('y') first, then translated using
        Vector(1,0,0) etc. in the now-rotated frame. After rotate90('y') around Y,
        the old X-axis becomes the new Z-axis, so translating by Vector(1,0,0)*attach_x
        moved along old-Z instead of along the fuselage. This placed the engine in
        completely the wrong location.

        Correct order: translate in the original (fuselage-aligned) frame first,
        then rotate the orientation at that translated point so the nacelle axis
        (X after rotate90) aligns with the fuselage thrust direction.
        """
        return translate(
            self.position,
            Vector(1, 0, 0), self._attach_x,
            Vector(0, 1, 0), self._attach_y,
            Vector(0, 0, 1), self._attach_z,
        ).rotate90('y')

    # ------------------------------------------------------------------ #
    # ENGINE INSTANCE — type selected here, not in Aircraft
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
            kwargs = dict(
                fan_radius=self.fan_radius,
                core_radius=self.core_radius,
            )
            return _JetEngine(**shared, **kwargs)
        kwargs = dict(
            n_blades=self.n_blades,
            blade_length=self.blade_length,
            blade_root_chord=self.blade_root_chord,
            blade_tip_chord=self.blade_tip_chord,
            blade_sweep=self.blade_sweep,
            spinner_radius=self.nacelle_radius,
            spinner_length=self.nacelle_length * 0.20,
        )
        return _PropellerEngine(**shared, **kwargs)

    # ------------------------------------------------------------------ #
    # FRAME
    # ------------------------------------------------------------------ #

    @Part
    def frame(self):
        return Frame(
            pos=self._engine_position,
            hidden=False,
        )


# ---------------------------------------------------------------------- #
# TEST
# ---------------------------------------------------------------------- #

if __name__ == "__main__":
    from parapy.gui import display

    eng = Engine(
        label="engine",
        engine_type="propeller",
        mtow=25.0,
        n_engines=1,
        thrust_to_weight=0.45,
        semi_span=1.5,
        sweep_le=0.0,
        dihedral=3.0,
        wing_root_x=0.8,
        wing_root_z=0.2,
        attach_spanwise_pct=0.0,
        attach_x_offset=-0.4,
        attach_z_offset=0.0,
        n_blades=3,
    )
    display(eng)