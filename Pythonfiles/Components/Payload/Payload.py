"""
Payload.py
==========
Payload assembly for the UAV KBE tool.

Raw component data (geometry, density, sources) lives in payload_library.json
alongside this file.  The module-level variables PAYLOAD_LIBRARY, WEAPON_DIMS,
DENSITY, and MANDATORY_PAYLOADS are loaded from that file at import time, so
all other modules (Payloadrules.py, etc.) that import them will work unchanged.

Positional layout
-----------------
Payload items are stacked sequentially along the fuselage x-axis starting at
``payload_start_x`` (measured from the Payload object's own origin).  Each item
is centred at the x-midpoint of its allocated slot; a configurable ``payload_gap``
separates consecutive items.

    payload_start_x
          |
          |<-- extent_0 -->|gap|<-- extent_1 -->|gap|...
          |       c0       |   |       c1       |

  c_i  = payload_start_x + sum(extents[0..i-1]) + i*gap + extent_i/2

For boxes    : extent = ``length``      (the x-aligned dimension)
For cylinders: extent = ``height_cyl``  (the long axis, rotated to lie along x)

Weapon stacking
---------------
When ``weapon_count`` > 1, weapons are arranged side-by-side in the y-z plane
(perpendicular to the fuselage x-axis) rather than repeated along x.  They are
packed into the most square grid possible:

    n_cols = ceil(sqrt(weapon_count))
    n_rows = ceil(weapon_count / n_cols)

Each weapon cylinder keeps its own diameter; the grid pitch equals the diameter
(cylinders touch).  The composite bounding envelope in y and z is:

    envelope_y = n_cols * diameter
    envelope_z = n_rows * diameter

This envelope drives the ``min_fuselage_radius`` requirement.
"""

import json
import math
import os

from parapy.core import Input, Attribute, Part, action, validate, child
from parapy.geom import GeomBase, Box, Cylinder, translate, rotate, Vector


# =============================================================================
# PAYLOAD TYPE COLOUR MAP
# =============================================================================
# Each payload category gets a visually distinct colour so items are
# immediately identifiable in the ParaPy 3-D view.

PAYLOAD_TYPE_COLORS = {
    "flight_computer": "SlateBlue",    # electronics — calm blue-violet
    "battery":         "LimeGreen",    # power storage — bright green
    "eo_ir":           "DeepSkyBlue",  # optical sensor — sky blue
    "radar":           "Orange",       # radar — warning orange
    "lidar":           "Cyan",         # laser rangefinder — cyan
    "comms":           "Gold",         # communication — gold
    "datalink":        "Yellow",       # data-link radio — yellow
    "weapon":          "Firebrick",    # weapon — dark red
}
# Fallback for any category not listed above
_DEFAULT_PAYLOAD_COLOR = "LightGray"


# =============================================================================
# LOAD DATABASE FROM JSON
# =============================================================================

_DATA_FILE = os.path.join(os.path.dirname(__file__), "..", "..", ".." , "Inputfiles", "payload_library.json")

with open(_DATA_FILE, encoding="utf-8") as _f:
    _DB = json.load(_f)

UAV_CLASSES        = _DB["uav_classes"]
CLASS_SCALE        = _DB["class_scale"]
DENSITY            = _DB["density"]
WEAPON_DIMS        = _DB["weapon_dims"]
MANDATORY_PAYLOADS = _DB["mandatory_payloads"]
PAYLOAD_LIBRARY    = _DB["payload_library"]


# =============================================================================
# MODEL RESOLVER
# =============================================================================

def resolve_model(category: str, model: str, uav_class: str = None) -> str:
    """Return the canonical model key for *category* closest to *model*."""
    sub = PAYLOAD_LIBRARY.get(category, {})
    if not sub:
        raise KeyError(f"Unknown payload category: '{category}'")

    if model in sub:
        return model

    model_lower = model.lower()
    for key, entry in sub.items():
        if model_lower in key.lower() or model_lower in entry["label"].lower():
            return key

    if uav_class:
        for key, entry in sub.items():
            if uav_class in entry.get("default_for", []):
                import warnings
                warnings.warn(
                    f"Model '{model}' not found in category '{category}'. "
                    f"Falling back to default for '{uav_class}': '{key}' "
                    f"({entry['label']}).",
                    UserWarning, stacklevel=3,
                )
                return key

    first_key = next(iter(sub))
    import warnings
    warnings.warn(
        f"Model '{model}' not found in category '{category}' and no "
        f"uav_class default available. Falling back to first entry: "
        f"'{first_key}' ({sub[first_key]['label']}).",
        UserWarning, stacklevel=3,
    )
    return first_key


# =============================================================================
# WEAPON GRID HELPERS
# =============================================================================

def weapon_grid(weapon_count: int):
    """
    Return (n_cols, n_rows) for the most-square grid fitting *weapon_count*
    cylinders packed side-by-side in the y-z plane.

    Examples
    --------
    1  → (1, 1)
    2  → (2, 1)
    3  → (2, 2)   [4 slots, 1 empty]
    4  → (2, 2)
    6  → (3, 2)
    8  → (3, 3)   [9 slots, 1 empty]
    """
    n_cols = math.ceil(math.sqrt(weapon_count))
    n_rows = math.ceil(weapon_count / n_cols)
    return n_cols, n_rows


class WeaponSolid(GeomBase):
    """Single weapon cylinder, placed at a pre-computed (dy, dz) offset."""

    diameter:    float = Input()
    height_cyl:  float = Input()
    dy:          float = Input()
    dz:          float = Input()
    color:       str   = Input("Firebrick")

    @Part(parse=False)
    def solid(self):
        return Cylinder(
            radius=self.diameter / 2.0,
            height=self.height_cyl,
            centered=True,
            color=self.color,
            position=rotate(
                translate(self.position, "y", self.dy, "z", self.dz),
                "y", math.pi / 2,
            ),
        )


# =============================================================================
# PAYLOAD ITEM
# =============================================================================

class PayloadItem(GeomBase):
    """
    A single payload component.

    For weapons with ``weapon_count`` > 1 the solid parts are arranged in a
    grid in the y-z plane (perpendicular to the fuselage axis).  All other
    payload types ignore ``weapon_count``.
    """

    payload_type: str = Input()
    model:        str = Input()
    uav_class:    str = Input(None)

    # -------------------------------------------------------------------------
    # Colour
    # -------------------------------------------------------------------------

    @Attribute
    def color(self) -> str:
        """Distinct colour for this payload type, from PAYLOAD_TYPE_COLORS."""
        return PAYLOAD_TYPE_COLORS.get(self.payload_type, _DEFAULT_PAYLOAD_COLOR)

    # Optional overrides
    mass_override:  float = Input(None)
    length:         float = Input(None)
    width:          float = Input(None)
    height_box:     float = Input(None)
    diameter:       float = Input(None)
    height_cyl:     float = Input(None)
    weapon_count:   int   = Input(1)

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
        """CG x-position of this item in the Payload's coordinate frame [m].
        For a symmetrically placed item the CG is at its own x-position."""
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
        """
        List of (dy, dz) offsets [m] for each weapon in the y-z grid,
        centred on the item's local origin.

        The grid is filled row-by-row (y fast, z slow).  The full grid
        envelope is:
            width_y  = n_cols * diameter
            height_z = n_rows * diameter

        Weapons are centred within this envelope, so the offsets run from
        -(n_cols-1)/2 * d  to  +(n_cols-1)/2 * d  in y, and similarly in z.
        """
        d = self.final_diameter
        nc = self._weapon_n_cols
        nr = self._weapon_n_rows
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
        """Flat list of alternating dy, dz values — avoids child.index on nested lists."""
        if not (self.payload_type == "weapon" and self.weapon_count > 1):
            return []
        return self._weapon_grid_positions   # list of (dy, dz) tuples

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
        """
        (half_width_y, half_height_z) of the item's cross-section envelope [m].

        For a single weapon or any non-weapon:
          - box      → (width/2, height_box/2)
          - cylinder → (diameter/2, diameter/2)

        For stacked weapons:
          - (n_cols * diameter / 2,  n_rows * diameter / 2)

        This is the *half*-size because the fuselage radius is measured from
        the centreline; the fuselage must accommodate the largest half-envelope
        across all items.
        """
        if self.payload_type == "weapon" and self.weapon_count > 1:
            d  = self.final_diameter
            nc = self._weapon_n_cols
            nr = self._weapon_n_rows
            return (nc * d / 2.0, nr * d / 2.0)

        if self.geometry_type == "box":
            return (self.final_width / 2.0, self.final_height_box / 2.0)

        # single cylinder (weapon or otherwise)
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
    # Bounding box (outer envelope for display / collision purposes)
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
    #
    # Weapons with weapon_count > 1 are placed at each grid offset in y-z.
    # All other items remain a single solid at the item's own position.
    # -------------------------------------------------------------------------

    @Part(parse=False)
    def solid(self):
        """
        Single solid for non-weapon items (or a single weapon).

        For multi-weapon items use ``weapon_solids`` instead; this part is
        suppressed (but still defined) so that the ParaPy tree is consistent.
        """
        if self.geometry_type == "box":
            return Box(
                length=self.final_length,
                width=self.final_width,
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


# =============================================================================
# PAYLOAD ASSEMBLY
# =============================================================================

class Payload(GeomBase):
    """
    Ordered collection of payload items laid out along the fuselage x-axis.

    Items are placed sequentially starting at ``payload_start_x`` (offset from
    this object's own origin).  Each item occupies a slot equal to its x-extent
    (``length`` for boxes, ``height_cyl`` for cylinders), with ``payload_gap``
    metres of clearance between consecutive items.

    Fuselage sizing outputs
    -----------------------
    ``min_fuselage_length``  — total payload bay length; the fuselage cylindrical
                               section must be at least this long.
    ``min_fuselage_radius``  — the largest cross-section half-envelope across all
                               items; the fuselage radius must be at least this
                               value to guarantee every item fits inside.
    """

    payload_config: list = Input(
        validator=validate.IsInstance(list),
        doc="""
        List of (category, model_key_or_partial_name) tuples.
        Example:
            [
                ("flight_computer", "flight_computer_pixhawk_6c"),
                ("battery",         "battery_small_lipo"),
                ("eo_ir",           "eo_ir_flir_tau2"),
            ]
        Partial / fuzzy model names are resolved via resolve_model().
        """
    )

    weapon_count: int = Input(1)

    uav_class: str = Input(
        None,
        doc="'small', 'medium', or 'large' — drives default fallback in resolve_model().",
    )

    payload_start_x: float = Input(
        0.0,
        doc="X-offset from this Payload's origin where the first item begins [m].",
    )

    payload_gap: float = Input(
        0.005,
        doc="Gap between consecutive payload items along x [m]. Default 5 mm.",
    )

    # -------------------------------------------------------------------------
    # Validation
    # -------------------------------------------------------------------------

    @Attribute
    def payload_types(self):
        return [cfg[0] for cfg in self.payload_config]

    @Attribute
    def has_mandatory_payloads(self):
        return all(p in self.payload_types for p in MANDATORY_PAYLOADS)

    # -------------------------------------------------------------------------
    # Positional layout
    # -------------------------------------------------------------------------

    @Attribute
    def item_x_positions(self):
        """
        Centre-x position of each item, in the Payload's coordinate frame [m].
        """
        x = self.payload_start_x
        positions = []
        for category, model in self.payload_config:
            resolved = resolve_model(category, model, self.uav_class)
            entry    = PAYLOAD_LIBRARY[category][resolved]
            extent   = (entry.get("length",     0.0) if entry["geometry_type"] == "box"
                        else entry.get("height_cyl", 0.0))
            x += extent / 2.0
            positions.append(x)
            x += extent / 2.0 + self.payload_gap
        return positions

    @Attribute
    def total_payload_length(self) -> float:
        """Total length of the payload bay along x [m]."""
        if not self.item_x_positions:
            return 0.0
        last_cat, last_mdl = self.payload_config[-1]
        last_entry = PAYLOAD_LIBRARY[last_cat][resolve_model(last_cat, last_mdl, self.uav_class)]
        last_half  = ((last_entry.get("length", 0.0) if last_entry["geometry_type"] == "box"
                       else last_entry.get("height_cyl", 0.0)) / 2.0)
        return self.item_x_positions[-1] + last_half - self.payload_start_x
    
    @Attribute
    def cg_x(self) -> float:
        """Mass-weighted CG x-position [m] in the Payload's coordinate frame."""
        if self.total_mass == 0:
            return self.payload_start_x
        return sum(item.mass * item.cg_x for item in self.items) / self.total_mass

    # -------------------------------------------------------------------------
    # Fuselage sizing outputs
    # -------------------------------------------------------------------------

    @Attribute
    def min_fuselage_length(self) -> float:
        """
        Minimum fuselage cylindrical-section length needed to contain the full
        payload bay [m].

        Equal to ``total_payload_length``; provided as an explicit named output
        so that the Fuselage class can consume it directly.
        """
        return self.total_payload_length

    @Attribute
    def min_fuselage_radius(self) -> float:
        """
        Minimum fuselage radius needed so that every payload item fits inside
        the fuselage cross-section [m].

        Computed as the largest *half-envelope* dimension across all items and
        both y and z directions, with a small clearance margin added.

        For non-weapon items this is half their width or diameter.
        For stacked weapons this accounts for the full n_cols × n_rows grid.

        A 5 % clearance margin is applied on top of the geometric minimum so
        that items are not flush against the fuselage skin.
        """
        clearance_factor = 1.05
        max_half = 0.0
        for item in self.items:
            hy, hz = item.cross_section_envelope
            max_half = max(max_half, hy, hz)
        return max_half * clearance_factor

    # -------------------------------------------------------------------------
    # Payload items
    # -------------------------------------------------------------------------

    @Part
    def items(self):
        return PayloadItem(
            quantify=len(self.payload_config),
            payload_type=self.payload_config[child.index][0],
            model=self.payload_config[child.index][1],
            uav_class=self.uav_class,
            weapon_count=self.weapon_count,
            position=translate(self.position, "x", self.item_x_positions[child.index]),
        )

    # -------------------------------------------------------------------------
    # Totals
    # -------------------------------------------------------------------------

    @Attribute
    def total_mass(self):
        return sum(item.mass for item in self.items)

    @Attribute
    def total_volume(self):
        return sum(item.volume for item in self.items)

    @Attribute
    def mass_breakdown(self):
        return {item.label: round(item.mass, 3) for item in self.items}

    @Attribute
    def volume_breakdown(self):
        return {item.label: round(item.volume, 6) for item in self.items}

    # -------------------------------------------------------------------------
    # Summary action
    # -------------------------------------------------------------------------

    @action(label="Print payload summary")
    def print_summary(self):
        print("=" * 80)
        print("PAYLOAD SUMMARY")
        print("=" * 80)
        for item, x_pos in zip(self.items, self.item_x_positions):
            bb = item.bounding_box_dims
            if item.payload_type == "weapon" and self.weapon_count > 1:
                nc, nr = item._weapon_n_cols, item._weapon_n_rows
                wc = f" x{item.weapon_count} ({nc}×{nr} grid)"
            elif item.payload_type == "weapon":
                wc = f" x{item.weapon_count}"
            else:
                wc = ""
            print(
                f"{item.label:<35s}"
                f"{wc:<20s}"
                f"x_centre = {x_pos:6.3f} m   "
                f"extent = {item.x_extent*1000:6.0f} mm   "
                f"mass = {item.mass:8.2f} kg   "
                f"BB = ({bb[0]*1000:.0f} x {bb[1]*1000:.0f} x {bb[2]*1000:.0f}) mm"
            )
            print(f"  └─ source: {item.source}")
        print("-" * 80)
        print(f"TOTAL PAYLOAD MASS          : {self.total_mass:.2f} kg")
        print(f"TOTAL PAYLOAD VOLUME        : {self.total_volume:.5f} m³")
        print(f"TOTAL PAYLOAD LENGTH        : {self.total_payload_length:.3f} m  "
              f"(start x = {self.payload_start_x:.3f} m, gap = {self.payload_gap*1000:.1f} mm)")
        print(f"MIN FUSELAGE LENGTH NEEDED  : {self.min_fuselage_length:.3f} m")
        print(f"MIN FUSELAGE RADIUS NEEDED  : {self.min_fuselage_radius:.4f} m  "
              f"(incl. 5 % clearance)")
        print("=" * 80)


# =============================================================================
# EXAMPLES
# =============================================================================

if __name__ == "__main__":

    from parapy.gui import display

    # Small ISR drone
    p1 = Payload(
        uav_class="small",
        payload_start_x=0.2,
        payload_config=[
            ("flight_computer", "flight_computer_pixhawk_6c"),
            ("battery",         "battery_small_lipo"),
            ("eo_ir",           "eo_ir_flir_tau2"),
            ("comms",           "comms_sik_radio"),
        ],
    )
    p1.print_summary()

    # Medium ISR + SAR UAV  (partial / fuzzy names)
    p2 = Payload(
        uav_class="medium",
        payload_start_x=0.5,
        payload_config=[
            ("flight_computer", "cube"),
            ("battery",         "large"),
            ("eo_ir",           "gimbal"),
            ("radar",           "sar"),
            ("datalink",        "small"),
        ],
    )
    p2.print_summary()

    # Strike UAV — 4 weapons stacked in a 2×2 grid
    p3 = Payload(
        uav_class="large",
        payload_start_x=1.0,
        weapon_count=4,
        payload_config=[
            ("flight_computer", "flight_computer_cube_orange"),
            ("battery",         "battery_large_lipo"),
            ("eo_ir",           "eo_ir_gimbal_hd"),
            ("radar",           "unknown_radar_xyz"),
            ("weapon",          "weapon_gbu12"),
        ],
    )
    p3.print_summary()

    # Strike UAV — 6 weapons in a 3×2 grid
    p4 = Payload(
        uav_class="large",
        payload_start_x=1.0,
        weapon_count=6,
        payload_config=[
            ("flight_computer", "flight_computer_cube_orange"),
            ("battery",         "battery_large_lipo"),
            ("weapon",          "weapon_gbu12"),
        ],
    )
    p4.print_summary()

    display([p1, p2, p3, p4])