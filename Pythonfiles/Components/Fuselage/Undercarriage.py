import math
import sys
import os

from parapy.core import Input, Attribute, Part, child
from parapy.geom import (
    GeomBase, LoftedSolid, RevolvedSolid,
    Circle, Vector, translate, Point
)

if __name__ != '__main__':
    from Frame import Frame


class Undercarriage(GeomBase):
    """Undercarriage geometry: struts + axles + tyres + wheels."""

    retractible: bool = Input()
    aircraft_mass: float = Input()
    fuselage_length: float = Input(20)
    fuselage_radius: float = Input(2)

    mesh_deflection: float = Input(1e-4)

    # ------------------------------------------------------------------ #
    # SIZING MODEL
    # ------------------------------------------------------------------ #

    @Attribute
    def n_main_struts(self) -> int:
        return 2 if self.aircraft_mass <= 50_000 else 4

    @Attribute
    def wheels_per_strut(self) -> int:
        return 1 if self.aircraft_mass <= 20_000 else 2

    @Attribute
    def strut_height(self):
        return 0.5 + 0.3 * math.log10(max(1.0, self.aircraft_mass / 1000.0))

    @Attribute
    def strut_radius(self):
        return 0.04 + 0.01 * math.log10(max(1.0, self.aircraft_mass / 1000.0))

    @Attribute
    def wheel_major_radius(self):
        return 0.15 + 0.05 * math.log10(max(1.0, self.aircraft_mass / 1000.0))

    @Attribute
    def wheel_minor_radius(self):
        return self.wheel_major_radius * 0.35

    @Attribute
    def axle_radius(self):
        return self.strut_radius * 0.6

    @Attribute
    def axle_half_width(self):
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
    # POSITIONS
    # ------------------------------------------------------------------ #

    @Attribute
    def _nose_gear_position_x(self):
        return self.fuselage_length * 0.1

    @Attribute
    def _main_gear_position_x(self):
        return self.fuselage_length * 0.6

    @Attribute
    def _main_gear_positions_y(self):
        spacing = self.wheel_major_radius * 3.5

        if self.n_main_struts == 2:
            return [-spacing, spacing]

        return [
            -spacing * 1.6, -spacing * 0.5,
             spacing * 0.5,  spacing * 1.6
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
            color="Gray",
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
            color="DarkGray",
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
            color="Black",
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
        return [0.0, self.strut_height]

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
            color="Gray",
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
            color="DarkGray",
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
        """Pre-compute all tyre y-positions as a plain list so child.index
        lookups never call _main_tyre_y with a non-integer argument."""
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
            color="Black",
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
    
    sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
    from Components.Frame import Frame

    obj = Undercarriage(
        retractible=True,
        aircraft_mass=50000,
        label="base_undercarriage",
    )
    display(obj)