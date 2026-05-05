import math

from parapy.core import Input, Attribute, Part, child
from parapy.geom import GeomBase, LoftedSolid, Circle, Vector, translate

from Pythonfiles.Components.Frame import Frame
from Pythonfiles.Components.Fuselage.Undercarriage import Undercarriage


class Fuselage(GeomBase):
    """Fuselage: nosecone (tapered) + cylinder + tailcone (tapered).

    Roskam defaults (Vol. I):
    - length            : §3.3 / Table 3.4 — estimated from MTOW via power-law regression
                          Re-fit for medium/large UAV category (25–500 kg MTOW).
                          Reference points: Predator ~8.2 m / 1020 kg,
                          Shadow 200 ~3.4 m / 170 kg, ScanEagle ~1.2 m / 18 kg.
    - radius            : §3.3 — fuselage fineness ratio l/d ~ 6 for UAVs
                          (lower than GA aircraft; UAVs carry bulkier payload bays)
    - cylinder_start    : §3.3 Fig. 3.7 — nosecone typically 10% of fuselage length
    - cylinder_end      : §3.3 Fig. 3.7 — tailcone typically starts at 70% fuselage length
    """

    # ------------------------------------------------------------------ #
    # PRIMARY SIZING — always required from user
    # ------------------------------------------------------------------ #

    aircraft_mass: float = Input()  # MTOW [kg]
    
    length_override: float = Input(None)   # user override [m]
    radius_override: float = Input(None)   # user override [m]  
    
    # Roskam Vol. I, §3.3, Fig. 3.7: nosecone/forebody typically 8–12% of length.
    # NOTE (Roskam): 10% is the statistical midpoint for fixed-wing subsonic aircraft.
    cylinder_start: float = Input()     # nosecone end [% of length]

    # Roskam Vol. I, §3.3, Fig. 3.7: tailcone starts at 65–75% of fuselage length.
    # NOTE (Roskam): 70% leaves adequate room for empennage attachment volume.
    cylinder_end: float = Input()       # tailcone start [% of length]

    # ------------------------------------------------------------------ #
    # RENDERING QUALITY — fixed defaults, no Roskam relevance
    # ------------------------------------------------------------------ #

    taper_sections: int = Input(10)
    cylinder_sections: int = Input(4)
    min_radius_pct: float = Input(0.0001)
    mesh_deflection: float = Input(1e-4)
    
    color_taper: str = Input()
    color_cylinder: str = Input()
    
    undercarriage_color_tyre: str = Input()
    undercarriage_color_axle: str = Input()
    undercarriage_color_strut: str = Input()

    undercarriage_retractible: bool = Input()

    

    # ------------------------------------------------------------------ #
    # FUSELAGE GEOMETRY — Roskam Vol. I statistical defaults (UAV re-fit)
    # ------------------------------------------------------------------ #

    # Roskam Vol. I, §3.3 / Table 3.4: fuselage length for fixed-wing UAV/UAS.
    # scales with MTOW via: L_fus ≈ a * MTOW^b.
    # For medium/large UAV category (25–500 kg): a ≈ 0.23, b ≈ 0.50 (SI, kg → m).
    # Derivation: Roskam Table 3.4 "homebuilt/UAV" row re-fit to UAV databases:
    #   ScanEagle  18 kg  → ~1.2 m   (0.23 * 18^0.50  ≈ 0.98 m)
    #   Shadow 200 170 kg → ~3.4 m   (0.23 * 170^0.50 ≈ 3.0 m)
    #   Predator   1020 kg→ ~8.2 m   (0.23 * 1020^0.50≈ 7.3 m)
    # NOTE (Roskam): statistical fit; verify against actual payload/avionics volume.
    @Attribute
    def _roskam_length(self) -> float:
        """Fuselage length estimate from Roskam Vol. I, Table 3.4 (UAV re-fit) [m]."""
        return 0.23 * (self.aircraft_mass ** 0.50)

    # Roskam Vol. I, §3.3: fuselage fineness ratio l/d ~ 5–8 for subsonic UAVs.
    # Lower value of 6 used vs GA aircraft (8); UAVs carry bulkier payloads/avionics bays
    # and do not optimise for passenger comfort volume.
    # NOTE (Roskam): fineness ratio < 5 increases form drag noticeably; > 8 wastes volume.
    @Attribute
    def _roskam_radius(self) -> float:
        """Fuselage radius from Roskam Vol. I fineness ratio l/d = 9."""
        fineness_ratio = 9.0
        diameter = self._roskam_length / fineness_ratio
        return diameter / 2.0

    @Attribute
    def length(self) -> float:
        if self.length_override is not None:
            return self.length_override
        return self._roskam_length

    @Attribute
    def radius(self) -> float:
        if self.radius_override is not None:
            return self.radius_override
        return self._roskam_radius

    # ------------------------------------------------------------------ #
    # POSITION HELPER
    # ------------------------------------------------------------------ #

    def _pos_x(self, x: float):
        """Return a position along the fuselage X-axis, starting at the nose (0,0,0)."""
        return translate(
            self.position.rotate90('y'),
            Vector(1, 0, 0), x,
        )

    # ------------------------------------------------------------------ #
    # JUNCTION POSITIONS (reused by frames and profiles)
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
        """Frame at aircraft nose — origin of the fuselage (0, 0, 0)."""
        return Frame(
            pos=self._pos_x(self._x_nose_tip),
            hidden=False,
        )

    @Part
    def frame_cylinder_start(self):
        """Frame at the nosecone / cylinder junction."""
        return Frame(
            pos=self._pos_x(self._x_cylinder_start),
            hidden=False,
        )

    @Part
    def frame_cylinder_end(self):
        """Frame at the cylinder / tailcone junction."""
        return Frame(
            pos=self._pos_x(self._x_cylinder_end),
            hidden=False,
        )

    @Part
    def frame_tail(self):
        """Frame at the tail tip."""
        return Frame(
            pos=self._pos_x(self._x_tail_tip),
            hidden=False,
        )

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
            label="undercarriage",
            color_tyre=self.undercarriage_color_tyre,
            color_axle=self.undercarriage_color_axle,
            color_strut=self.undercarriage_color_strut,
        )

    # ------------------------------------------------------------------ #
    # NOSE CONE GEOMETRY
    # ------------------------------------------------------------------ #

    @Attribute
    def _nose_positions(self) -> list[float]:
        cs = self.cylinder_start / 100.0
        n  = self.taper_sections
        return [(i / (n - 1)) * cs * self.length for i in range(n)]

    @Attribute
    def _nose_radii(self) -> list[float]:
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
            mesh_deflection=self.mesh_deflection,
        )

    # ------------------------------------------------------------------ #
    # CYLINDRICAL PART GEOMETRY
    # ------------------------------------------------------------------ #

    @Attribute
    def _cyl_positions(self) -> list[float]:
        cs = self.cylinder_start / 100.0
        ce = self.cylinder_end   / 100.0
        nc = self.cylinder_sections
        return [(cs + (i / nc) * (ce - cs)) * self.length for i in range(nc + 1)]

    @Attribute
    def _cyl_radii(self) -> list[float]:
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
    def _tail_positions(self) -> list[float]:
        ce = self.cylinder_end / 100.0
        n  = self.taper_sections
        return [(ce + (i / (n - 1)) * (1.0 - ce)) * self.length for i in range(n)]

    @Attribute
    def _tail_radii(self) -> list[float]:
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
            mesh_deflection=self.mesh_deflection,
        )

    # ------------------------------------------------------------------ #
    # CALCULATIONS  # TODO: implement properly
    # ------------------------------------------------------------------ #

    @Attribute
    def calculate_mass(self):
        return 1

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
    display(obj)