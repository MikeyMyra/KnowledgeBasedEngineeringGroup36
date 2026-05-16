import math

from parapy.core import Input, Attribute, Part, child
from parapy.geom import GeomBase, LoftedSolid, RevolvedSolid, Circle, Vector, translate, Point

from Pythonfiles.Components.Frame import Frame


class Undercarriage(GeomBase):
    """Undercarriage geometry: struts + axles + tyres + wheels.

    Roskam defaults (Vol. I):
    - n_main_struts      : §8.5 / Fig. 8.4 — tricycle config standard for fixed-wing UAV/GA
    - wheels_per_strut   : §8.5 Table 8.4 — single wheel up to ~5 700 kg MTOW
    - nose_gear_x        : §8.6 — nose gear at 8–15% fuselage length; 10% midpoint used
    - main_gear_x        : §8.6 — main gear at 55–65% fuselage length; 60% used
    - strut_height       : §8.6 Fig. 8.8 — ground clearance > max prop radius + 18 cm margin
    - wheel_major_radius : §8.5 Table 8.4 — tyre diameter correlation with MTOW
    - wheel_minor_radius : §8.5 — tyre width ≈ 35% of tyre diameter (fixed-wing norm)
    - strut_radius       : NOTE (Roskam default not explicit) — scaled from strut load
    - main_gear_y        : §8.6 Fig. 8.5 — lateral track ≥ 3× wheel radius for tip-over stability
    """

    retractible: bool = Input()
    aircraft_mass: float = Input()          # MTOW [kg]
    fuselage_length: float = Input()
    fuselage_radius: float = Input()

    mesh_deflection: float = Input(1e-4)
    
    color_tyre: str = Input()
    color_axle: str = Input()
    color_strut: str = Input()

    # Largest rotating radius that must clear the ground (propeller tip or
    # nacelle radius depending on engine type).  Supplied by Aircraft via
    # Fuselage; defaults to 0 (no external clearance constraint).
    prop_clearance_radius: float = Input(0.0)

    # ------------------------------------------------------------------ #
    # SIZING MODEL — Roskam Vol. I statistical relationships
    # ------------------------------------------------------------------ #

    @Attribute
    def n_main_struts(self) -> int:
        """Number of main gear struts.

        Roskam Vol. I, §8.5 / Fig. 8.4: tricycle gear is standard for fixed-wing UAV.
        Two main struts sufficient up to ~50 000 kg MTOW; four used above that.
        """
        return 2 if self.aircraft_mass <= 50_000 else 4

    @Attribute
    def wheels_per_strut(self) -> int:
        """Wheels per strut.

        Roskam Vol. I, §8.5, Table 8.4: single wheel per strut up to ~5 700 kg MTOW.
        Dual wheels used above that threshold for ground bearing pressure limits.
        """
        return 1 if self.aircraft_mass <= 5_700 else 2

    @Attribute
    def wheel_major_radius(self) -> float:
        """Tyre outer radius [m].

        Roskam Vol. I, §8.5, Table 8.4 covers GA/transport aircraft (MTOW > ~500 kg).
        The standard fit d_tire = 0.10 * m^0.29 extrapolates poorly to UAV masses,
        producing wheels larger than the fuselage below ~100 kg.

        The formula below is re-fitted to match:
          - Roskam Table 8.4 at the high end (5 700 kg → ~0.21 m radius)
          - Real UAV tyre data at the low end (25 kg → ~0.033 m radius, e.g. 66 mm tyre)
            d_tire [m] ≈ 0.034 * MTOW[kg]^0.21
        NOTE (Roskam): select the nearest standard tyre from a supplier catalogue;
        this formula is a sizing starting point only.
        """
        d_tire = 0.034 * (max(1.0, self.aircraft_mass) ** 0.21)
        return d_tire / 2.0

    @Attribute
    def wheel_minor_radius(self) -> float:
        """Tyre cross-section radius (half-width) [m].

        Roskam Vol. I, §8.5: tyre width ≈ 35% of tyre outer diameter.
        NOTE (Roskam): round up to nearest standard section width.
        """
        return self.wheel_major_radius * 0.35

    @Attribute
    def strut_height(self) -> float:
        """Strut height (extended length from fuselage attach point to axle centre) [m].

        Roskam Vol. I, §8.6, Fig. 8.8: minimum ground clearance ≥ max rotating
        radius (propeller tip or nacelle) + 18 cm safety margin.

        Ground-to-fuselage-centreline distance = fuselage_radius + strut_height
        + wheel_major_radius.  Solving for strut_height:

            strut_height ≥ prop_clearance_radius + 0.18
                           − fuselage_radius − wheel_major_radius

        Three competing floors are taken:
          1. Clearance requirement  (propeller / nacelle, above)
          2. Tyre-stroke floor      : 2.0 × wheel_major_radius  (axle above ground
                                      with tyre and ~50 % stroke travel margin)
          3. Mass-based floor       : 0.05 + 0.015 × log10(m)  — prevents
                                      near-zero struts on very light aircraft
        """
        mass_floor    = 0.05 + 0.015 * math.log10(max(1.0, self.aircraft_mass))
        tyre_floor    = self.wheel_major_radius * 2.0
        if self.prop_clearance_radius > 0.0:
            # Roskam §8.6: strut must place fuselage-CL + clearance above ground
            clearance_req = max(0.0,
                                self.prop_clearance_radius + 0.18
                                - self.fuselage_radius
                                - self.wheel_major_radius)
        else:
            clearance_req = 0.0
        return max(tyre_floor, clearance_req, mass_floor)

    @Attribute
    def _cg_height(self) -> float:
        """Approximate CG height above ground [m].

        Roskam Vol. I, §8.6: used for tip-over stability criterion.
        Conservatively taken as the fuselage centreline height, which equals
        fuselage_radius + strut_height + wheel outer radius.
        """
        return self.fuselage_radius + self.strut_height + self.wheel_major_radius

    @Attribute
    def strut_radius(self) -> float:
        """Structural strut tube radius [m].

        NOTE (Roskam default not explicit): sized as 25% of wheel_major_radius,
        which gives visually and structurally proportionate struts across the UAV
        mass range.  The previous log formula had a 0.04 m floor that was
        oversized for small UAVs.  Flag for structural column-buckling check.
        """
        return self.wheel_major_radius * 0.25

    @Attribute
    def axle_radius(self) -> float:
        """Axle radius [m] — 60% of strut radius (standard proportion)."""
        return self.strut_radius * 0.6

    @Attribute
    def axle_half_width(self) -> float:
        """Half-width of dual-wheel axle [m] — 1.2× tyre major radius."""
        return self.wheel_major_radius * 1.2

    # ------------------------------------------------------------------ #
    # POSITION HELPER
    # ------------------------------------------------------------------ #

    def pos(self, x=0.0, y=0.0, z=0.0, base=None, rotate=None):
        p = base or self.position

        if rotate:
            p = p.rotate90(rotate)

        return translate(
            p,
            Vector(1, 0, 0), x,
            Vector(0, 1, 0), y,
            Vector(0, 0, 1), z - self.fuselage_radius - self.strut_height,
        )

    # ------------------------------------------------------------------ #
    # LONGITUDINAL & LATERAL POSITIONS — Roskam Vol. I §8.6
    # ------------------------------------------------------------------ #

    @Attribute
    def _nose_gear_position_x(self) -> float:
        """Nose gear longitudinal station [m].

        Roskam Vol. I, §8.6: nose gear located at 8–15% of fuselage length.
        10% is the statistical midpoint; ensures adequate steering moment arm.
        NOTE (Roskam): verify nose gear load fraction (typically 8–15% of MTOW).
        """
        return self.fuselage_length * 0.10

    @Attribute
    def _main_gear_position_x(self) -> float:
        """Main gear longitudinal station [m].

        Roskam Vol. I, §8.6: main gear at 55–65% fuselage length for typical CG range.
        60% used as statistical midpoint.
        NOTE (Roskam): move aft if CG travel pushes aft of 30% MAC.
        """
        return self.fuselage_length * 0.60

    @Attribute
    def _main_gear_positions_y(self) -> list:
        """Lateral positions of each main strut [m].

        Roskam Vol. I, §8.6, Fig. 8.5 — roll-over (tip-over) criterion:
        The angle ψ from vertical to the line connecting the main gear contact
        point to the CG must satisfy ψ ≥ 35°  (equivalently the aircraft can
        lean 55° from the ground before tipping):

            ψ = atan(half_track / cg_height) ≥ 35°
            → half_track ≥ cg_height × tan(35°) ≈ cg_height × 0.700

        A practical lower floor of 3.5 × wheel_major_radius is also kept to
        prevent over-narrow gear on very low-CG configurations.
        """
        min_tipover = self._cg_height * math.tan(math.radians(35.0))
        spacing     = max(self.wheel_major_radius * 3.5, min_tipover)

        if self.n_main_struts == 2:
            return [-spacing, spacing]

        return [
            -spacing * 1.6, -spacing * 0.5,
             spacing * 0.5,  spacing * 1.6,
        ]

    # ------------------------------------------------------------------ #
    # NOSE STRUT
    # ------------------------------------------------------------------ #

    @Attribute
    def _nose_strut_positions_z(self):
        return [0.0, self.strut_height]

    @Attribute
    def _nose_strut_radii(self):
        return [self.strut_radius, self.strut_radius]

    @Part
    def nose_strut_profiles(self):
        return Circle(
            quantify=2,
            radius=self._nose_strut_radii[child.index],
            color="Gray",
            position=self.pos(
                x=self._nose_gear_position_x,
                z=self._nose_strut_positions_z[child.index],
            ),
        )

    @Part
    def nose_strut(self):
        return LoftedSolid(
            profiles=self.nose_strut_profiles,
            color=self.color_strut,
            mesh_deflection=self.mesh_deflection,
        )

    @Part
    def nose_strut_frame(self):
        return Frame(
            pos=self.pos(x=self._nose_gear_position_x),
            hidden=False,
        )

    # ------------------------------------------------------------------ #
    # NOSE AXLE
    # ------------------------------------------------------------------ #

    @Attribute
    def _nose_axle_positions_y(self):
        if self.wheels_per_strut == 1:
            return [-self.wheel_minor_radius, self.wheel_minor_radius]
        return [-self.axle_half_width, self.axle_half_width]

    @Attribute
    def _nose_axle_radii(self):
        return [self.axle_radius, self.axle_radius]

    @Part
    def nose_axle_profiles(self):
        return Circle(
            quantify=2,
            radius=self._nose_axle_radii[child.index],
            color="DarkGray",
            position=self.pos(
                x=self._nose_gear_position_x,
                y=self._nose_axle_positions_y[child.index],
                rotate='x',
            ),
        )

    @Part
    def nose_axle(self):
        return LoftedSolid(
            profiles=self.nose_axle_profiles,
            color=self.color_axle,
            mesh_deflection=self.mesh_deflection,
        )

    @Part
    def nose_axle_frame(self):
        return Frame(
            pos=self.pos(x=self._nose_gear_position_x, rotate='x'),
            hidden=False,
        )

    # ------------------------------------------------------------------ #
    # NOSE TYRES
    # ------------------------------------------------------------------ #

    @Attribute
    def _nose_tyre_positions_y(self):
        if self.wheels_per_strut == 1:
            return [0.0]
        return [-self.axle_half_width, self.axle_half_width]

    @Attribute
    def _nose_tyre_count(self):
        return self.wheels_per_strut

    @Part
    def nose_tyres_profiles(self):
        return Circle(
            quantify=self._nose_tyre_count,
            radius=self.wheel_minor_radius,
            color="Black",
            position=self.pos(
                x=self._nose_gear_position_x,
                y=self._nose_tyre_positions_y[child.index],
                z=self.wheel_major_radius,
                rotate='y',
            ),
        )

    @Part
    def nose_tyres(self):
        return RevolvedSolid(
            quantify=self._nose_tyre_count,
            built_from=self.nose_tyres_profiles[child.index],
            center=Point(
                self._nose_gear_position_x,
                self._nose_tyre_positions_y[child.index],
                -self.fuselage_radius - self.strut_height,
            ),
            direction=Vector(0, 1, 0),
            color=self.color_tyre,
            mesh_deflection=self.mesh_deflection,
        )

    @Part
    def nose_tyre_frame(self):
        return Frame(
            pos=self.pos(
                x=self._nose_gear_position_x,
                z=self.wheel_major_radius,
                rotate='y',
            ),
            hidden=False,
        )

    # ------------------------------------------------------------------ #
    # MAIN STRUTS
    # ------------------------------------------------------------------ #

    @Attribute
    def _main_strut_positions_z(self):
        return [0.0, self.strut_height*2]

    @Attribute
    def _main_strut_radii(self):
        return [self.strut_radius, self.strut_radius]

    @Part
    def main_strut_profiles(self):
        return Circle(
            quantify=self.n_main_struts * 2,
            radius=self._main_strut_radii[child.index % 2],
            color="Gray",
            position=self.pos(
                x=self._main_gear_position_x,
                y=self._main_gear_positions_y[child.index // 2],
                z=self._main_strut_positions_z[child.index % 2],
            ),
        )

    @Part
    def main_struts(self):
        return LoftedSolid(
            quantify=self.n_main_struts,
            profiles=[
                self.main_strut_profiles[child.index * 2],
                self.main_strut_profiles[child.index * 2 + 1],
            ],
            color=self.color_strut,
            mesh_deflection=self.mesh_deflection,
        )

    @Part
    def main_strut_frames(self):
        return Frame(
            quantify=self.n_main_struts,
            pos=self.pos(
                x=self._main_gear_position_x,
                y=self._main_gear_positions_y[child.index],
            ),
            hidden=False,
        )

    # ------------------------------------------------------------------ #
    # MAIN AXLES
    # ------------------------------------------------------------------ #

    @Attribute
    def _main_axle_offsets(self):
        if self.wheels_per_strut == 1:
            return [0, 0]
        return [-self.axle_half_width, self.axle_half_width]

    @Part
    def main_axle_profiles(self):
        return Circle(
            quantify=self.n_main_struts * 2,
            radius=self.axle_radius,
            color="DarkGray",
            position=self.pos(
                x=self._main_gear_position_x,
                y=self._main_gear_positions_y[child.index // 2]
                    + self._main_axle_offsets[child.index % 2],
                rotate='x',
            ),
        )

    @Part
    def main_axles(self):
        return LoftedSolid(
            quantify=self.n_main_struts,
            profiles=[
                self.main_axle_profiles[child.index * 2],
                self.main_axle_profiles[child.index * 2 + 1],
            ],
            color=self.color_axle,
            mesh_deflection=self.mesh_deflection,
        )

    @Part
    def main_axle_frames(self):
        return Frame(
            quantify=self.n_main_struts,
            pos=self.pos(
                x=self._main_gear_position_x,
                y=self._main_gear_positions_y[child.index],
                rotate='x',
            ),
            hidden=False,
        )

    # ------------------------------------------------------------------ #
    # MAIN TYRES
    # ------------------------------------------------------------------ #

    @Attribute
    def _main_tyre_count(self):
        return self.n_main_struts * self.wheels_per_strut

    @Attribute
    def _main_tyre_positions_y(self):
        return [
            self._main_gear_positions_y[i // self.wheels_per_strut]
            + self._main_axle_offsets[i % self.wheels_per_strut]
            for i in range(self._main_tyre_count)
        ]

    @Part
    def main_tyres_profiles(self):
        return Circle(
            quantify=self._main_tyre_count,
            radius=self.wheel_minor_radius,
            color="Black",
            position=self.pos(
                x=self._main_gear_position_x,
                y=self._main_tyre_positions_y[child.index],
                z=self.wheel_major_radius,
                rotate='y',
            ),
        )

    @Part
    def main_tyres(self):
        return RevolvedSolid(
            quantify=self._main_tyre_count,
            built_from=self.main_tyres_profiles[child.index],
            center=Point(
                self._main_gear_position_x,
                self._main_tyre_positions_y[child.index],
                -self.fuselage_radius - self.strut_height,
            ),
            direction=Vector(0, 1, 0),
            color=self.color_tyre,
            mesh_deflection=self.mesh_deflection,
        )

    @Part
    def main_tyre_frames(self):
        return Frame(
            quantify=self._main_tyre_count,
            pos=self.pos(
                x=self._main_gear_position_x,
                y=self._main_tyre_positions_y[child.index],
                z=self.wheel_major_radius,
                rotate='y',
            ),
            hidden=False,
        )


# ---------------------------------------------------------------------- #
# TEST
# ---------------------------------------------------------------------- #

if __name__ == '__main__':
    from parapy.gui import display

    obj = Undercarriage(
        retractible=False,
        # Required: sizing fully driven by MTOW
        aircraft_mass=25,       # 25 kg UAV — all geometry auto-derived
        fuselage_length=3,
        fuselage_radius=0.5,
        label="test_undercarriage",
        color_tyre="black",
        color_axle="silver",
        color_strut="gray",
    )
    display(obj)