import math

from parapy.core import Input, Attribute, Part, child
from parapy.geom import GeomBase, LoftedSolid, Circle, Vector, translate

from Pythonfiles.Components.Frame import Frame
from Pythonfiles.Components.Fuselage.Undercarriage import Undercarriage

from Pythonfiles.metric_imperial_conversions import (
    kilograms_to_pounds,
    feet_to_meters,
    meters_to_feet,
    pounds_to_kilograms,
)


class Fuselage(GeomBase):
    """Fuselage: nosecone (tapered) + cylinder + tailcone (tapered).

    Roskam defaults (Vol. I):
    - length            : §3.3 / Table 3.4 — estimated from MTOW via power-law regression
    - radius            : §3.3 — fuselage fineness ratio l/d ~ 9
    - cylinder_start    : §3.3 Fig. 3.7 — nosecone typically 10% of fuselage length
    - cylinder_end      : §3.3 Fig. 3.7 — tailcone typically starts at 70% fuselage length

    Structural mass (Roskam Vol. I, §8.3):
    - W_fus = C_f * MTOW^0.5 * L_fus^0.25
      C_f = 0.328 (UAV re-fit from Roskam Table 8.1 "homebuilt" row scaled for UAV).
    - CG assumed at 45% fuselage length (Roskam Vol. I §8.3 statistical midpoint).
    """

    # ------------------------------------------------------------------ #
    # PRIMARY SIZING
    # ------------------------------------------------------------------ #

    aircraft_mass:   float = Input()        # MTOW [kg]
    length_override: float = Input(None)    # user override [m]
    radius_override: float = Input(None)    # user override [m]
    length_min_override: float = Input(0.0)
    radius_min_override: float = Input(0.0)

    cylinder_start: float = Input()         # nosecone end   [% of length]
    cylinder_end:   float = Input()         # tailcone start [% of length]

    # ------------------------------------------------------------------ #
    # RENDERING QUALITY
    # ------------------------------------------------------------------ #

    taper_sections:    int   = Input(10)
    cylinder_sections: int   = Input(4)
    min_radius_pct:    float = Input(0.0001)
    mesh_deflection:   float = Input(1e-4)

    color_taper:    str = Input()
    color_cylinder: str = Input()

    undercarriage_color_tyre:  str  = Input()
    undercarriage_color_axle:  str  = Input()
    undercarriage_color_strut: str  = Input()
    undercarriage_retractible: bool = Input()
    # Largest rotating radius that must clear the ground (prop tip or nacelle).
    # Computed by Aircraft and forwarded here so Undercarriage can size struts.
    prop_clearance_radius: float = Input(0.0)

    # Payload object — drives min length/radius, None = no constraint
    payload = Input(None)

    # Gap between the nose-cone/cylinder junction and the first payload item [m].
    # Ensures the payload does not overlap the prop nacelle (tractor configs).
    # Forwarded from Aircraft.payload_nose_clearance.
    payload_nose_clearance: float = Input(0.0)

    # FuelTank object — placed in centre wing box, drives additional
    # cylinder length and possibly radius.  None = no tank constraint.
    fuel_tank = Input(None)

    # ------------------------------------------------------------------ #
    # ROSKAM FUSELAGE GEOMETRY
    # ------------------------------------------------------------------ #

    @Attribute
    def _roskam_length(self) -> float:
        """Fuselage length from Roskam Vol. I Table 3.4 UAV re-fit [m].
        Formula is L = 0.23 * MTOW^0.5 with MTOW in lbs, result in ft.
        Converted to metric via metric_imperial_conversions.
        """
        mtow_lbs  = kilograms_to_pounds(self.aircraft_mass)
        length_ft = 0.23 * (mtow_lbs ** 0.50)
        return feet_to_meters(length_ft)

    @Attribute
    def _roskam_radius(self) -> float:
        """
        Fuselage radius from fineness ratio l/d = 14 [m].

        Roskam Vol. I §3.3 gives l/d ≈ 9 for manned aircraft, but UAVs are
        notably more slender.  Real-world UAV data:
          Predator A  : 8.2 m / 0.86 m  → l/d ≈ 19
          Global Hawk : 14.5 m / 1.0 m  → l/d ≈ 29
          Heron       : ~8 m  / 0.70 m  → l/d ≈ 23
        l/d = 14 is a conservative lower bound that still keeps the fuselage
        reasonably slender for a compact UAV without payload / tank constraints.
        """
        return self._roskam_length / (14.0 * 2.0)

    @Attribute
    def _tank_gap(self) -> float:
        """Structural gap between payload bay and fuel tank [m] (50 mm)."""
        return 0.05

    @Attribute
    def length(self) -> float:
        """
        Fuselage total length [m] — payload + fuel-tank led sizing.

        Priority (highest to lowest):
        ① Manual override  (length_override)
        ② Combined bay length = payload bay + gap + fuel tank length,
           scaled by 1/cylinder_fraction to get total fuselage length.
           Both the payload bay and the fuel tank are altitude-independent
           (payload geometry is fixed; tank depends only on fuel_mass /
           density, not on fuselage dimensions), so the fuselage stays
           compact even when the wing grows large at high altitude.
        ③ Roskam power-law estimate — only when neither payload nor fuel
           tank is defined  (L = 0.23 · MTOW^0.5, Roskam Vol. I Table 3.4).
        ④ length_min_override  (from wing-chord constraint in Aircraft).

        Fuselage cylinder layout (longitudinal):
        ┌─────────────┬──────────────┬─────────────────┐
        │ payload bay │  50 mm gap  │  fuel tank      │
        └─────────────┴──────────────┴─────────────────┘
        ← cylinder_start %           cylinder_end % →
        """
        if self.length_override is not None:
            return max(self.length_override, self.length_min_override)

        cylinder_fraction = (self.cylinder_end - self.cylinder_start) / 100.0

        # payload_nose_clearance is the gap reserved between the cylinder start
        # and the first payload item (to clear the prop nacelle).  It must be
        # counted as part of the cylinder occupancy so the fuselage stretches
        # to fit payload_clearance + payload_bay + gap + tank.
        payload_len = (self.payload.min_fuselage_length + self.payload_nose_clearance
                       if self.payload is not None else 0.0)
        tank_len    = (self.fuel_tank.min_fuselage_length + self._tank_gap
                       if self.fuel_tank is not None else 0.0)

        combined = payload_len + tank_len

        if combined > 0.0:
            base = combined / cylinder_fraction
        else:
            # Neither payload nor fuel tank — fall back to Roskam estimate.
            base = self._roskam_length

        return max(base, self.length_min_override)

    @Attribute
    def _structural_min_radius(self) -> float:
        """
        Hard lower bound on fuselage radius [m] driven by structural integrity.

        The old value of 6 % of length gave l/d ≈ 8.3 — far too fat for a
        slender UAV fuselage.  Real UAV fineness ratios are l/d = 15–29.
        Using 3 % of length gives l/d ≈ 16.7, which is a conservative but
        realistic lower bound for a semi-monocoque composite UAV fuselage.

        Absolute floor of 60 mm ensures longerons and skin can physically fit
        even for very small (< 2 m) fuselages.
        """
        return max(self.length * 0.03, 0.06)

    @Attribute
    def radius(self) -> float:
        """
        Fuselage outer radius [m] — payload + fuel-tank led sizing.

        Priority (highest to lowest):
        ① Manual override  (radius_override)
        ② Largest of payload cross-section and fuel tank cross-section.
           The fuselage must accommodate both: the payload bay and the
           fuel tank share the same cylindrical section, so the fuselage
           radius must satisfy the tighter of the two radial constraints.
           • Payload: Payload.min_fuselage_radius  (+5 % clearance baked in)
           • Tank:    FuelTank.min_fuselage_radius  (+3 % clearance baked in)
        ③ Roskam fineness-ratio estimate — only when no payload OR tank defined.
        ④ Structural minimum  (_structural_min_radius  ≈ 6 % of length)
        ⑤ Wing-interference minimum  (radius_min_override = 8 % root chord)
        """
        if self.radius_override is not None:
            return self.radius_override

        candidates = [self._structural_min_radius, self.radius_min_override]

        if self.payload is not None:
            candidates.append(self.payload.min_fuselage_radius)

        if self.fuel_tank is not None:
            candidates.append(self.fuel_tank.min_fuselage_radius)

        if self.payload is None and self.fuel_tank is None:
            # No geometry driver — fall back to Roskam fineness-ratio estimate.
            candidates.append(self._roskam_radius)

        return max(candidates)

    # ------------------------------------------------------------------ #
    # STRUCTURAL MASS  (Roskam Vol. I §8.3)
    # ------------------------------------------------------------------ #

    @Attribute
    def calculate_mass(self) -> float:
        """
        Fuselage structural mass [kg].

        Roskam Vol. I §8.3, Eq. (8.5) — UAV re-fit:
            W_fus = 0.328 * MTOW^0.5 * L_fus^0.25

        MTOW in lbs, L_fus in ft, result in lbs — converted to kg.
        The coefficient 0.328 is the Roskam Table 8.1 "homebuilt" value
        scaled down by 0.70 for composite/light UAV construction.
        Reference: Roskam Table 8.1; UAV scaling per Raymer §15.3.
        """
        from Pythonfiles.metric_imperial_conversions import meters_to_feet, pounds_to_kilograms
        mtow_lbs   = kilograms_to_pounds(self.aircraft_mass)
        length_ft  = meters_to_feet(self.length)
        weight_lbs = 0.328 * (mtow_lbs ** 0.50) * (length_ft ** 0.25)
        return pounds_to_kilograms(weight_lbs)

    @Attribute
    def cg_x(self) -> float:
        """
        Fuselage structural CG x-position from nose [m].

        Roskam Vol. I §8.3: fuselage mass centroid at ~45% of fuselage length
        for conventional tapered fuselages (nosecone + cylinder + tailcone).
        """
        return 0.45 * self.length

    # ------------------------------------------------------------------ #
    # LOCAL RADIUS INTERPOLATION
    # ------------------------------------------------------------------ #

    def local_radius_at(self, x: float) -> float:
        """Fuselage cross-section radius [m] at longitudinal station x [m]."""
        import numpy as np
        xs = list(self._nose_positions) + list(self._cyl_positions) + list(self._tail_positions)
        rs = list(self._nose_radii)     + list(self._cyl_radii)     + list(self._tail_radii)
        return float(np.interp(x, xs, rs))

    # ------------------------------------------------------------------ #
    # POSITION HELPER
    # ------------------------------------------------------------------ #

    def _pos_x(self, x: float):
        return translate(
            self.position.rotate90('y'),
            Vector(1, 0, 0), x,
        )

    # ------------------------------------------------------------------ #
    # JUNCTION POSITIONS
    # ------------------------------------------------------------------ #

    @Attribute
    def _x_nose_tip(self) -> float:
        return 0.0

    @Attribute
    def _x_cylinder_start(self) -> float:
        return (self.cylinder_start / 100.0) * self.length

    @Attribute
    def _x_cylinder_end(self) -> float:
        return (self.cylinder_end / 100.0) * self.length

    @Attribute
    def _x_tail_tip(self) -> float:
        return self.length

    # ------------------------------------------------------------------ #
    # REFERENCE FRAMES
    # ------------------------------------------------------------------ #

    @Part
    def frame_nose(self):
        return Frame(pos=self._pos_x(self._x_nose_tip), hidden=False)

    @Part
    def frame_cylinder_start(self):
        return Frame(pos=self._pos_x(self._x_cylinder_start), hidden=False)

    @Part
    def frame_cylinder_end(self):
        return Frame(pos=self._pos_x(self._x_cylinder_end), hidden=False)

    @Part
    def frame_tail(self):
        return Frame(pos=self._pos_x(self._x_tail_tip), hidden=False)

    # ------------------------------------------------------------------ #
    # CHILD COMPONENTS
    # ------------------------------------------------------------------ #

    @Part
    def undercarriage(self):
        return Undercarriage(
            retractible=self.undercarriage_retractible,
            aircraft_mass=self.aircraft_mass,
            fuselage_length=self.length,
            fuselage_radius=self.radius,
            prop_clearance_radius=self.prop_clearance_radius,
            label="undercarriage",
            color_tyre=self.undercarriage_color_tyre,
            color_axle=self.undercarriage_color_axle,
            color_strut=self.undercarriage_color_strut,
        )

    # ------------------------------------------------------------------ #
    # NOSE CONE GEOMETRY
    # ------------------------------------------------------------------ #

    @Attribute
    def _nose_positions(self) -> list:
        cs = self.cylinder_start / 100.0
        n  = self.taper_sections
        return [(i / (n - 1)) * cs * self.length for i in range(n)]

    @Attribute
    def _nose_radii(self) -> list:
        r_min = self.min_radius_pct / 100.0
        n     = self.taper_sections
        def ellipse_blend(t):
            return r_min + (1.0 - r_min) * math.sqrt(max(0.0, 1.0 - t ** 2))
        return [ellipse_blend(1.0 - i / (n - 1)) * self.radius for i in range(n)]

    @Part
    def nose_profiles(self):
        return Circle(
            quantify=self.taper_sections,
            color=self.color_taper,
            radius=self._nose_radii[child.index],
            position=self._pos_x(self._nose_positions[child.index]),
        )

    @Part
    def nose(self):
        return LoftedSolid(
            profiles=self.nose_profiles,
            color=self.color_taper,
            transparency=0.3,
            mesh_deflection=self.mesh_deflection,
        )

    # ------------------------------------------------------------------ #
    # CYLINDRICAL PART GEOMETRY
    # ------------------------------------------------------------------ #

    @Attribute
    def _cyl_positions(self) -> list:
        cs = self.cylinder_start / 100.0
        ce = self.cylinder_end   / 100.0
        nc = self.cylinder_sections
        return [(cs + (i / nc) * (ce - cs)) * self.length for i in range(nc + 1)]

    @Attribute
    def _cyl_radii(self) -> list:
        return [self.radius] * (self.cylinder_sections + 1)

    @Part
    def cyl_profiles(self):
        return Circle(
            quantify=self.cylinder_sections + 1,
            color=self.color_cylinder,
            radius=self._cyl_radii[child.index],
            position=self._pos_x(self._cyl_positions[child.index]),
        )

    @Part
    def cylinder(self):
        return LoftedSolid(
            profiles=self.cyl_profiles,
            color=self.color_cylinder,
            transparency=0.3,
            mesh_deflection=self.mesh_deflection,
        )

    # ------------------------------------------------------------------ #
    # TAIL CONE GEOMETRY
    # ------------------------------------------------------------------ #

    @Attribute
    def _tail_positions(self) -> list:
        ce = self.cylinder_end / 100.0
        n  = self.taper_sections
        return [(ce + (i / (n - 1)) * (1.0 - ce)) * self.length for i in range(n)]

    @Attribute
    def _tail_radii(self) -> list:
        r_min = self.min_radius_pct / 100.0
        n     = self.taper_sections
        def ellipse_blend(t):
            return r_min + (1.0 - r_min) * math.sqrt(max(0.0, 1.0 - t ** 2))
        return [ellipse_blend(i / (n - 1)) * self.radius for i in range(n)]

    @Part
    def tail_profiles(self):
        return Circle(
            quantify=self.taper_sections,
            color=self.color_taper,
            radius=self._tail_radii[child.index],
            position=self._pos_x(self._tail_positions[child.index]),
        )

    @Part
    def tail(self):
        return LoftedSolid(
            profiles=self.tail_profiles,
            color=self.color_taper,
            transparency=0.3,
            mesh_deflection=self.mesh_deflection,
        )

    # ------------------------------------------------------------------ #
    # REMAINING TODOS
    # ------------------------------------------------------------------ #

    @Attribute
    def calculate_volume(self):
        return 1

    @Attribute
    def calculate_skin_friction(self):
        return 1


# ---------------------------------------------------------------------- #
# TEST
# ---------------------------------------------------------------------- #

if __name__ == '__main__':
    from parapy.gui import display

    obj = Fuselage(
        undercarriage_retractible=False,
        aircraft_mass=2500,
        length_override=20,
        radius_override=1,
        cylinder_start=10,
        cylinder_end=70,
        label="test_fuselage",
        color_taper="SteelBlue",
        color_cylinder="LightBlue",
        undercarriage_color_tyre="pink",
        undercarriage_color_axle="silver",
        undercarriage_color_strut="gray",
    )
    print(f"Fuselage mass : {obj.calculate_mass:.1f} kg")
    print(f"Fuselage CG x : {obj.cg_x:.2f} m")
    display(obj)