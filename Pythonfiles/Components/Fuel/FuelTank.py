"""
Capsule-shaped fuel tank sized from fuel mass and type.

Loads fuel properties (density, energy content) from fuel_properties.json,
applies ullage and structure fractions, and lofts a capsule geometry
(cylinder + hemispherical caps) placed in the fuselage centre section.
"""

import json
import math
import os

from parapy.core  import Input, Attribute, Part, child, action
from parapy.geom  import GeomBase, LoftedSolid, Circle, translate, Vector


_DB_PATH = os.path.join(os.path.dirname(__file__), "..", "..", ".." , "Inputfiles", "fuel_properties.json")

with open(_DB_PATH, encoding="utf-8") as _f:
    _FUEL_DB = json.load(_f)

FUELS          = _FUEL_DB["fuels"]
AUTO_SELECTION = _FUEL_DB["auto_selection"]
_ULLAGE        = _FUEL_DB["ullage_fraction"]        # 0.05
_STRUCTURE     = _FUEL_DB["structure_fraction"]     # 0.05
_VOLUME_FACTOR = 1.0 / (1.0 - _ULLAGE - _STRUCTURE)  # ≈ 1.111


class FuelTank(GeomBase):
    """
    Capsule-shaped fuel tank sized from fuel mass and fuel type.

    The tank is placed in the fuselage centre wing box immediately aft of
    the payload bay.  Its ``position`` (nose) is set by Aircraft.
    
    Geometry
    --------
    The tank is a *capsule*: a cylinder of length L_cyl capped at each end
    by a hemisphere of radius R.

        |<-- R -->|<-------- L_cyl -------->|<-- R -->|
        └─ nose ─┘         cylinder         └─ tail ─┘
        Total length  L_total = L_cyl + 2·R
        Total volume  V       = π·R²·L_cyl + (4/3)·π·R³

    References
    ----------
    - Roskam Vol. I §3.5  — fuel system mass fractions
    - MIL-T-5578          — ullage and pressurisation requirements
    - fuel_properties.json — density and compatibility data
    """

    # ------------------------------------------------------------------ #
    # PRIMARY INPUTS
    # ------------------------------------------------------------------ #

    fuel_mass: float = Input()
    """Fuel mass [kg] — from mission sizing (Drone.fuel_weight)."""

    fuel_type: str = Input("auto")
    """
    Fuel type key from fuel_properties.json, or ``'auto'`` to select
    based on ``engine_type``.

    Valid keys: ``'avgas_100ll'``, ``'jet_a'``, ``'jp8'``, ``'lipo_battery'``.
    """

    engine_type: str = Input("Piston")
    """
    Engine type string from Drone — used only when ``fuel_type == 'auto'``.
    One of: ``'Piston'``, ``'Turboprop'``, ``'Jet'``, ``'Electric'``.
    """

    tank_aspect_ratio: float = Input(3.0)
    """
    Tank length-to-diameter ratio  AR = L_total / (2·R).

    AR = 1   → pure sphere (no cylindrical section).
    AR = 3   → moderately elongated capsule (default; good for wing-box fit).
    AR > 5   → slender tank (useful when fuselage radius is constrained).

    Must be ≥ 1.01 (enforced internally).
    """

    # ------------------------------------------------------------------ #
    # DISPLAY CONTROL
    # ------------------------------------------------------------------ #

    taper_sections: int   = Input(12)
    color_tank:     str   = Input("CornflowerBlue")
    transparency:   float = Input(0.35)
    mesh_deflection: float = Input(1e-4)

    # ------------------------------------------------------------------ #
    # FUEL DATABASE LOOKUP
    # ------------------------------------------------------------------ #

    @Attribute
    def resolved_fuel_type(self) -> str:
        """
        Resolved fuel type key [str].

        If ``fuel_type == 'auto'``, selects from ``auto_selection`` map
        using ``engine_type``.  Falls back to ``'jet_a'`` if key missing.
        """
        if self.fuel_type != "auto":
            if self.fuel_type not in FUELS:
                raise ValueError(
                    f"Unknown fuel_type '{self.fuel_type}'. "
                    f"Valid options: {list(FUELS.keys())} or 'auto'."
                )
            return self.fuel_type
        return AUTO_SELECTION.get(self.engine_type, "jet_a")

    @Attribute
    def fuel_properties(self) -> dict:
        """Full properties dict for the resolved fuel type."""
        return FUELS[self.resolved_fuel_type]

    @Attribute
    def fuel_label(self) -> str:
        """Human-readable fuel name."""
        return self.fuel_properties["label"]

    @Attribute
    def fuel_density(self) -> float:
        """Fuel density [kg/m³]."""
        return self.fuel_properties["density_kg_m3"]

    @Attribute
    def fuel_lhv(self) -> float:
        """Lower heating value [MJ/kg]."""
        return self.fuel_properties["lhv_mj_kg"]

    # ------------------------------------------------------------------ #
    # VOLUME SIZING
    # ------------------------------------------------------------------ #

    @Attribute
    def fuel_volume(self) -> float:
        """Net fuel volume [m³]  =  fuel_mass / density."""
        return self.fuel_mass / self.fuel_density

    @Attribute
    def tank_volume(self) -> float:
        """
        Required tank internal volume [m³], including ullage and structure.
        """
        return self.fuel_volume * _VOLUME_FACTOR

    # ------------------------------------------------------------------ #
    # CAPSULE GEOMETRY
    # ------------------------------------------------------------------ #

    @Attribute
    def _ar(self) -> float:
        """Aspect ratio, enforced ≥ 1.01 to guarantee a non-zero cylinder."""
        return max(self.tank_aspect_ratio, 1.01)

    @Attribute
    def outer_radius(self) -> float:
        """
        Tank outer radius R [m].

        Derived from  V_tank = π·R³·(2·AR − 2/3):
            R = (V_tank / (π·(2·AR − 2/3)))^(1/3)
        """
        ar  = self._ar
        r3  = self.tank_volume / (math.pi * (2.0 * ar - 2.0 / 3.0))
        return r3 ** (1.0 / 3.0)

    @Attribute
    def cylinder_length(self) -> float:
        """Length of the cylindrical mid-section [m]."""
        return 2.0 * self.outer_radius * (self._ar - 1.0)

    @Attribute
    def total_length(self) -> float:
        """Total tank length (cylinder + 2 hemispherical caps) [m]."""
        return 2.0 * self.outer_radius * self._ar

    # ------------------------------------------------------------------ #
    # MASS & CG
    # ------------------------------------------------------------------ #

    @Attribute
    def cg_local_x(self) -> float:
        """
        Fuel CG x-offset from tank nose [m].
        """
        return self.total_length / 2.0

    # ------------------------------------------------------------------ #
    # FUSELAGE SIZING OUTPUTS
    # ------------------------------------------------------------------ #

    @Attribute
    def min_fuselage_length(self) -> float:
        """
        Minimum fuselage cylinder contribution required for this tank [m].
        """
        return self.total_length

    @Attribute
    def min_fuselage_radius(self) -> float:
        """
        Minimum fuselage inner radius to accommodate the tank [m].
        """
        return self.outer_radius * 1.03

    # ------------------------------------------------------------------ #
    # PROFILE GEOMETRY FOR LOFTED SOLID
    # ------------------------------------------------------------------ #

    @Attribute
    def _profiles_data(self) -> list:
        """
        List of (x_local, radius) pairs for all cross-section circles.

        Layout:
        ① Nose hemisphere  — taper_sections circles,  x: 0 → R,  r: 0 → R
        ② Cylinder end     — 1 circle at x = R + L_cyl,  r = R
           (only added when L_cyl > 1 mm to avoid duplicate profiles)
        ③ Tail hemisphere  — taper_sections−1 circles, x: R+L_cyl → L_total,
                             r: R → 0  (first point shared with cylinder)

        A minimum radius of 0.2 % of R prevents degenerate (zero-radius)
        profiles that cause LoftedSolid to fail.
        """
        n      = max(self.taper_sections, 4)
        R      = self.outer_radius
        L_cyl  = self.cylinder_length
        r_min  = max(R * 0.002, 1e-4)   # degenerate-profile guard

        data = []

        # ① nose hemisphere
        for i in range(n):
            theta = (math.pi / 2.0) * i / (n - 1)
            x_loc = R * (1.0 - math.cos(theta))   # 0 → R
            r_loc = R * math.sin(theta)             # 0 → R
            data.append((x_loc, max(r_loc, r_min)))

        # ② cylinder far end
        if L_cyl > 1e-3:
            data.append((R + L_cyl, R))

        # ③ tail hemisphere
        for i in range(1, n):
            theta = (math.pi / 2.0) * i / (n - 1)
            x_loc = R + L_cyl + R * math.sin(theta)   # R+L_cyl → total_length
            r_loc = R * math.cos(theta)                 # R → 0
            data.append((x_loc, max(r_loc, r_min)))

        return data

    @Attribute
    def _profile_xs(self) -> list:
        return [d[0] for d in self._profiles_data]

    @Attribute
    def _profile_rs(self) -> list:
        return [d[1] for d in self._profiles_data]

    # ------------------------------------------------------------------ #
    # PARTS
    # ------------------------------------------------------------------ #

    @Part
    def profiles(self):
        return Circle(
            quantify=len(self._profiles_data),
            radius=self._profile_rs[child.index],
            color=self.color_tank,
            position=translate(
                self.position.rotate90('y'),
                Vector(1, 0, 0), self._profile_xs[child.index],
            ),
        )

    @Part
    def tank(self):
        return LoftedSolid(
            profiles=self.profiles,
            color=self.color_tank,
            transparency=self.transparency,
            mesh_deflection=self.mesh_deflection,
        )

    # ------------------------------------------------------------------ #
    # SUMMARY ACTION
    # ------------------------------------------------------------------ #

    @action(label="Print fuel tank summary")
    def print_summary(self):
        print("=" * 60)
        print("FUEL TANK SUMMARY")
        print("=" * 60)
        print(f"  Fuel type          : {self.fuel_label}")
        print(f"  Fuel density       : {self.fuel_density:.0f} kg/m³")
        print(f"  Fuel LHV           : {self.fuel_lhv:.1f} MJ/kg")
        print(f"  Fuel mass          : {self.fuel_mass:.1f} kg")
        print(f"  Fuel volume        : {self.fuel_volume*1000:.1f} L  "
              f"({self.fuel_volume:.4f} m³)")
        print(f"  Tank volume (×{_VOLUME_FACTOR:.3f}): {self.tank_volume*1000:.1f} L  "
              f"(ullage + structure included)")
        print()
        print(f"  Aspect ratio AR    : {self._ar:.2f}  (L_total / D_outer)")
        print(f"  Outer radius R     : {self.outer_radius*1000:.1f} mm")
        print(f"  Cylinder length    : {self.cylinder_length*1000:.1f} mm")
        print(f"  Total length       : {self.total_length*1000:.1f} mm")
        print()
        print(f"  min_fuselage_length: {self.min_fuselage_length*1000:.1f} mm")
        print(f"  min_fuselage_radius: {self.min_fuselage_radius*1000:.1f} mm  "
              f"(+3 % clearance)")
        print(f"  CG offset from nose: {self.cg_local_x*1000:.1f} mm")
        print("=" * 60)


# ------------------------------------------------------------------ #
# TEST
# ------------------------------------------------------------------ #

if __name__ == "__main__":
    from parapy.gui import display

    # Medium ISR drone — ~120 kg fuel (Jet-A), AR 3
    t1 = FuelTank(
        label="jet_a_tank",
        fuel_mass=120.0,
        fuel_type="jet_a",
        engine_type="Turboprop",
        tank_aspect_ratio=3.0,
        color_tank="CornflowerBlue",
    )
    t1.print_summary()

    # Small piston UAV — 15 kg Avgas, AR 4 (slender to fit narrow fuselage)
    t2 = FuelTank(
        label="avgas_tank",
        fuel_mass=15.0,
        fuel_type="avgas_100ll",
        engine_type="Piston",
        tank_aspect_ratio=4.0,
        color_tank="Gold",
    )
    t2.print_summary()

    # HALE — 800 kg Jet-A, AR 5 (large, slender to fit narrow HALE fuselage)
    t3 = FuelTank(
        label="hale_tank",
        fuel_mass=800.0,
        fuel_type="jet_a",
        engine_type="Turboprop",
        tank_aspect_ratio=5.0,
        color_tank="SteelBlue",
    )
    t3.print_summary()

    display([t1, t2, t3])
