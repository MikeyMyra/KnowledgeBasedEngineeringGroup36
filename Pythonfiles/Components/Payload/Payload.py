import math

from parapy.core import Attribute, Input, Part, action, child, validate
from parapy.geom import GeomBase, translate

from Pythonfiles.Components.Payload.PayloadDatabase import MANDATORY_PAYLOADS, PAYLOAD_LIBRARY, resolve_model
from Pythonfiles.Components.Payload.PayloadItem import PayloadItem

# Re-export so Drone.py imports keep working without modification
__all__ = ["Payload", "PAYLOAD_LIBRARY", "resolve_model"]


class Payload(GeomBase):
    """
    Ordered collection of payload items laid out along the fuselage x-axis.
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
        """Centre-x position of each item in the Payload's coordinate frame [m]."""
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
        last_entry = PAYLOAD_LIBRARY[last_cat][
            resolve_model(last_cat, last_mdl, self.uav_class)
        ]
        last_half = (
            last_entry.get("length", 0.0) if last_entry["geometry_type"] == "box"
            else last_entry.get("height_cyl", 0.0)
        ) / 2.0
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
        """Minimum fuselage cylindrical-section length to contain the payload [m]."""
        return self.total_payload_length

    @Attribute
    def min_fuselage_radius(self) -> float:
        """
        Minimum fuselage radius so every payload item fits inside [m].
        A 5 % clearance margin is applied on top of the geometric minimum.
        """
        clearance_factor = 1.05
        max_half = 0.0
        for item in self.items:
            hy, hz = item.cross_section_envelope
            hpyt   = math.sqrt(hy**2 + hz**2)
            max_half = max(max_half, hy, hz, hpyt)
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
              f"(start x = {self.payload_start_x:.3f} m, "
              f"gap = {self.payload_gap*1000:.1f} mm)")
        print(f"MIN FUSELAGE LENGTH NEEDED  : {self.min_fuselage_length:.3f} m")
        print(f"MIN FUSELAGE RADIUS NEEDED  : {self.min_fuselage_radius:.4f} m  "
              f"(incl. 5 % clearance)")
        print("=" * 80)


# =============================================================================
# EXAMPLES
# =============================================================================

if __name__ == "__main__":
    from parapy.gui import display

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

    display([p1, p2])
