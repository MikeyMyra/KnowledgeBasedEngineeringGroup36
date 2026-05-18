import math

from parapy.core import Attribute, Input, Part
from parapy.geom import Box, Cylinder, GeomBase, rotate

from Pythonfiles.Components.Payload.PayloadDatabase import PAYLOAD_LIBRARY, PAYLOAD_TYPE_COLORS, _DEFAULT_PAYLOAD_COLOR, resolve_model, weapon_grid

from Pythonfiles.Components.Payload.WeaponSolid import WeaponSolid


class PayloadItem(GeomBase):
    """A single payload item — geometry, mass, and cross-section envelope."""

    payload_type: str   = Input()
    model:        str   = Input()
    uav_class:    str   = Input(None)

    # Optional dimension / mass overrides
    mass_override:  float = Input(None)
    length:         float = Input(None)
    width:          float = Input(None)
    height_box:     float = Input(None)
    diameter:       float = Input(None)
    height_cyl:     float = Input(None)
    weapon_count:   int   = Input(1)

    # -------------------------------------------------------------------------
    # Colour
    # -------------------------------------------------------------------------

    @Attribute
    def color(self) -> str:
        """Distinct colour for this payload type from PAYLOAD_TYPE_COLORS."""
        return PAYLOAD_TYPE_COLORS.get(self.payload_type, _DEFAULT_PAYLOAD_COLOR)

    # -------------------------------------------------------------------------
    # Database lookup
    # -------------------------------------------------------------------------

    @Attribute
    def resolved_model(self):
        return resolve_model(self.payload_type, self.model, self.uav_class)

    @Attribute
    def db(self):
        return PAYLOAD_LIBRARY[self.payload_type][self.resolved_model]

    @Attribute
    def label(self):
        return self.db["label"]

    @Attribute
    def source(self):
        return self.db.get("source", "No source recorded")

    @Attribute
    def geometry_type(self):
        return self.db["geometry_type"]

    @Attribute
    def density(self):
        return self.db["density"]

    # -------------------------------------------------------------------------
    # Dimensions  (input overrides take priority over database values)
    # -------------------------------------------------------------------------

    @Attribute
    def final_length(self):
        return self.length if self.length else self.db.get("length", 0.0)

    @Attribute
    def final_width(self):
        return self.width if self.width else self.db.get("width", 0.0)

    @Attribute
    def final_height_box(self):
        return self.height_box if self.height_box else self.db.get("height_box", 0.0)

    @Attribute
    def final_diameter(self):
        return self.diameter if self.diameter else self.db.get("diameter", 0.0)

    @Attribute
    def final_height_cyl(self):
        return self.height_cyl if self.height_cyl else self.db.get("height_cyl", 0.0)

    @Attribute
    def cg_x(self) -> float:
        return self.position.x

    # -------------------------------------------------------------------------
    # Weapon grid geometry
    # -------------------------------------------------------------------------

    @Attribute
    def _weapon_n_cols(self) -> int:
        """Number of columns in the weapon y-z grid."""
        return weapon_grid(self.weapon_count)[0]

    @Attribute
    def _weapon_n_rows(self) -> int:
        """Number of rows in the weapon y-z grid."""
        return weapon_grid(self.weapon_count)[1]

    @Attribute
    def _weapon_grid_positions(self) -> list:
        d   = self.final_diameter
        nc  = self._weapon_n_cols
        nr  = self._weapon_n_rows
        positions = []
        for row in range(nr):
            for col in range(nc):
                if len(positions) >= self.weapon_count:
                    break
                dy = (col - (nc - 1) / 2.0) * d
                dz = (row - (nr - 1) / 2.0) * d
                positions.append((dy, dz))
        return positions

    @Attribute
    def _weapon_offset_list(self) -> list:
        """Flat list of (dy, dz) tuples — avoids child.index on nested lists."""
        if not (self.payload_type == "weapon" and self.weapon_count > 1):
            return []
        return self._weapon_grid_positions

    @Part(parse=False)
    def weapon_solids(self):
        return [
            WeaponSolid(
                diameter=self.final_diameter,
                height_cyl=self.final_height_cyl,
                dy=dy,
                dz=dz,
                color=self.color,
                position=self.position,
            )
            for dy, dz in self._weapon_offset_list
        ]

    # -------------------------------------------------------------------------
    # Envelope in the y-z plane (used by Payload for fuselage sizing)
    # -------------------------------------------------------------------------

    @Attribute
    def cross_section_envelope(self) -> tuple:
        """(half_width_y, half_height_z) of the item's cross-section [m]."""
        if self.payload_type == "weapon" and self.weapon_count > 1:
            d  = self.final_diameter
            nc = self._weapon_n_cols
            nr = self._weapon_n_rows
            return (nc * d / 2.0, nr * d / 2.0)
        if self.geometry_type == "box":
            return (self.final_width / 2.0, self.final_height_box / 2.0)
        r = self.final_diameter / 2.0
        return (r, r)

    # -------------------------------------------------------------------------
    # Extent along the fuselage x-axis
    # -------------------------------------------------------------------------

    @Attribute
    def x_extent(self) -> float:
        """Length of this item along the fuselage x-axis [m]."""
        if self.geometry_type == "box":
            return self.final_length
        return self.final_height_cyl   # cylinder height lies along x

    # -------------------------------------------------------------------------
    # Volume & mass
    # -------------------------------------------------------------------------

    @Attribute
    def single_volume(self):
        if self.geometry_type == "box":
            return self.final_length * self.final_width * self.final_height_box
        return math.pi / 4.0 * self.final_diameter ** 2 * self.final_height_cyl

    @Attribute
    def volume(self):
        if self.payload_type == "weapon":
            return self.single_volume * self.weapon_count
        return self.single_volume

    @Attribute
    def mass(self):
        if self.mass_override is not None:
            return self.mass_override
        return self.volume * self.density

    # -------------------------------------------------------------------------
    # Bounding box
    # -------------------------------------------------------------------------

    @Attribute
    def bounding_box_dims(self):
        """(x_extent, y_width, z_height) of the full item envelope [m]."""
        hy, hz = self.cross_section_envelope
        if self.payload_type == "weapon" and self.weapon_count > 1:
            return (self.x_extent, hy * 2, hz * 2)
        if self.geometry_type == "box":
            return (self.final_length, self.final_width, self.final_height_box)
        return (self.final_height_cyl, self.final_diameter, self.final_diameter)

    # -------------------------------------------------------------------------
    # Geometry
    # -------------------------------------------------------------------------

    @Part(parse=False)
    def solid(self):
        """Single solid for non-weapon items (or a single weapon)."""
        if self.geometry_type == "box":
            return Box(
                length=self.final_width,
                width=self.final_length,
                height=self.final_height_box,
                centered=True,
                color=self.color,
                suppress=(self.payload_type == "weapon" and self.weapon_count > 1),
            )
        return Cylinder(
            radius=self.final_diameter / 2.0,
            height=self.final_height_cyl,
            centered=True,
            color=self.color,
            position=rotate(self.position, "y", math.pi / 2),
            suppress=(self.payload_type == "weapon" and self.weapon_count > 1),
        )
