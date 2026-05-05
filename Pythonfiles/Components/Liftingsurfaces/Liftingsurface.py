from math import radians, tan
import numpy as np

from parapy.core import Input, Attribute, Part
from parapy.geom import GeomBase, LoftedSolid, translate, rotate

from Pythonfiles.Components.Liftingsurfaces.Airfoil import Airfoil
from Pythonfiles.Components.Liftingsurfaces.Wingbox import Wingbox
from Pythonfiles.Components.Frame import Frame


class LiftingSurface(GeomBase):

    # ------------------------------------------------------------ #
    # PRIMARY INPUTS
    # NOTE: wing_area is the TOTAL area (both halves combined) [m²]
    #       semi_span is the half-span of ONE side [m]
    # ------------------------------------------------------------ #

    wing_area: float = Input()      # total reference area, both sides [m²]
    semi_span: float = Input()      # half-span, one side [m]

    fuselage_length: float = Input()
    fuselage_radius: float = Input()

    is_tail: bool = Input()
    is_vertical_tail: bool = Input()

    mesh_deflection: float = Input(1e-4)
    
    color_wingbox: str = Input()
    color_liftingsurface: str = Input()

    # ------------------------------------------------------------ #
    # GEOMETRY
    # ------------------------------------------------------------ #

    taper_ratio: float = Input()
    sweep_le: float = Input()
    twist: float = Input()
    dihedral: float = Input()

    thickness_to_chord: float = Input()
    maximum_camber: float = Input()
    maximum_camber_position: float = Input()

    # ------------------------------------------------------------ #
    # AIRFOIL SCALING
    # ------------------------------------------------------------ #

    t_factor_root: float = Input()
    t_factor_tip: float = Input()

    # ------------------------------------------------------------ #
    # SPARS
    # ------------------------------------------------------------ #

    front_spar_position: float = Input()
    rear_spar_position: float = Input()

    # ------------------------------------------------------------ #
    # ROSKAM TAIL PARAMETERS
    # ------------------------------------------------------------ #

    tail_volume_coefficient_h: float = Input()
    tail_volume_coefficient_v: float = Input()
    tail_aspect_ratio_h: float = Input()
    tail_aspect_ratio_v: float = Input()

    # ------------------------------------------------------------ #
    # WING REFERENCE OBJECT
    # ------------------------------------------------------------ #

    wing_ref: "LiftingSurface" = Input()

    # ------------------------------------------------------------ #
    # ATTACH POSITION
    # ------------------------------------------------------------ #

    @Attribute
    def attach_x(self):
        if self.is_tail:
            return self.wing_ref.attach_x + self.tail_arm
        # Wing: place the quarter-chord of the MAC at a fixed fuselage station
        return 0.40 * self.fuselage_length  # or make this an Input

    @Attribute
    def attach_z(self):
        if self.is_tail and not self.is_vertical_tail:
            return 0
        
        elif self.is_vertical_tail:
            return 0 #self.fuselage_radius * 0.5
        
        else:
            return 0 #-self.fuselage_radius * 0.5

    # ------------------------------------------------------------ #
    # TAIL ARM
    # ------------------------------------------------------------ #

    @Attribute
    def tail_arm(self):
        if not self.is_tail:
            return None

        span = self.wing_ref.effective_span
        fuselage_length = self.fuselage_length
        wing_x = self.wing_ref.attach_x 

        max_arm = fuselage_length - wing_x  # can't go beyond the tail tip
        return min(0.65 * span, max_arm)

    # ------------------------------------------------------------ #
    # ROSKAM EFFECTIVE AREA
    # NOTE: all areas here are TOTAL (both sides) [m²]
    # ------------------------------------------------------------ #

    @Attribute
    def effective_area(self):
        """Total reference area (both sides) [m²]."""

        if not self.is_tail:
            return self.wing_area       # wing_area is already total

        if self.wing_ref is None:
            raise ValueError("Tail requires wing_ref")

        wing = self.wing_ref

        S_w = wing.wing_area            # total wing area  [m²]
        b_w = 2 * wing.semi_span        # full wing span   [m]
        c_w = wing.mean_aerodynamic_chord

        if self.is_vertical_tail:
            Vv = self.tail_volume_coefficient_v
            return (Vv * S_w * b_w) / self.tail_arm

        else:
            Vh = self.tail_volume_coefficient_h
            return (Vh * S_w * c_w) / self.tail_arm

    # ------------------------------------------------------------ #
    # SPAN
    # ------------------------------------------------------------ #

    @Attribute
    def effective_span(self):
        if not self.is_tail:
            return self.semi_span

        if self.is_vertical_tail:
            full_span = np.sqrt(self.tail_aspect_ratio_v * self.effective_area)
            return full_span  # semi-span for one side

        # Horizontal tail
        full_span = np.sqrt(self.tail_aspect_ratio_h * self.effective_area)
        return full_span / 2
    
    # ------------------------------------------------------------ #
    # PLANFORM
    # NOTE: effective_area is total; effective_span is semi-span
    #   S_total = semi_span * c_root * (1 + taper)
    #   → c_root = S_total / (semi_span * (1 + taper))
    # ------------------------------------------------------------ #

    @Attribute
    def c_root(self):
        return self.effective_area / (self.effective_span * (1 + self.taper_ratio))

    @Attribute
    def c_tip(self):
        return self.c_root * self.taper_ratio

    @Attribute
    def aspect_ratio(self):
        # AR = full_span² / S_total = (2*semi_span)² / effective_area
        return (2 * self.effective_span) ** 2 / self.effective_area

    @Attribute
    def mean_aerodynamic_chord(self):
        tr = self.taper_ratio
        return (2 / 3) * self.c_root * (1 + tr + tr**2) / (1 + tr)

    @Attribute
    def mac_spanwise_position(self):
        tr = self.taper_ratio
        return self.effective_span * (1 + 2 * tr) / (3 * (1 + tr))

    @Attribute
    def mac_x_offset(self):
        return self.mac_spanwise_position * tan(radians(self.sweep_le))

    # ------------------------------------------------------------ #
    # POSITIONING
    # ------------------------------------------------------------ #

    @Attribute
    def _root_position(self):

        base = translate(
            self.position,
            "x", self.attach_x,
            "z", self.attach_z,
        )

        if not self.is_vertical_tail:
            return base

        # VT rotates upright (X-axis rotation)
        return rotate(base, "x", radians(90))

    @Attribute
    def _tip_position(self):
        """Tip position for the starboard (positive-Y) side."""

        if not self.is_vertical_tail:
            return rotate(
                translate(
                    self._root_position,
                    "y", self.effective_span,
                    "x", self.effective_span * tan(radians(self.sweep_le)),
                    "z", self.effective_span * np.sin(radians(self.dihedral)),
                ),
                "y",
                radians(self.twist),
            )

        # VT: span goes in Y, structure already rotated upright
        return rotate(
            translate(
                self._root_position,
                "y", self.effective_span,
                "x", self.effective_span * tan(radians(self.sweep_le)),
            ),
            "z",
            radians(self.twist),
        )

    @Attribute
    def _tip_position_mirrored(self):
        """Tip position for the port (negative-Y) side. None for vertical tail."""
        if self.is_vertical_tail:
            return None

        return rotate(
            translate(
                self._root_position,
                "y", -self.effective_span,
                "x", self.effective_span * tan(radians(self.sweep_le)),
                "z", self.effective_span * np.sin(radians(self.dihedral)),
            ),
            "y",
            radians(self.twist),
        )

    # ------------------------------------------------------------ #
    # AIRFOILS
    # ------------------------------------------------------------ #

    @Part
    def root_airfoil(self):
        return Airfoil(
            chord=self.c_root,
            maximum_camber=self.maximum_camber,
            camber_position=self.maximum_camber_position,
            thickness_to_chord=self.thickness_to_chord,
            export_dat=True,
            airfoil_name="root_airfoil",
            position=self._root_position,
        )

    @Part
    def tip_airfoil(self):
        """Starboard tip airfoil."""
        return Airfoil(
            chord=self.c_tip,
            maximum_camber=self.maximum_camber,
            camber_position=self.maximum_camber_position,
            thickness_to_chord=self.thickness_to_chord,
            export_dat=True,
            airfoil_name="tip_airfoil",
            position=self._tip_position,
        )

    @Part
    def tip_airfoil_mirrored(self):
        """Port tip airfoil — suppressed for vertical tail."""
        return Airfoil(
            chord=self.c_tip,
            maximum_camber=self.maximum_camber,
            camber_position=self.maximum_camber_position,
            thickness_to_chord=self.thickness_to_chord,
            export_dat=True,
            airfoil_name="tip_airfoil_mirrored",
            position=self._tip_position_mirrored,
            suppress=self.is_vertical_tail,
        )

    # ------------------------------------------------------------ #
    # SOLIDS
    # ------------------------------------------------------------ #

    @Part
    def solid(self):
        """Starboard wing solid."""
        return LoftedSolid(
            profiles=[self.root_airfoil.geometry, self.tip_airfoil.geometry],
            color=self.color_liftingsurface,
            mesh_deflection=self.mesh_deflection,
        )

    @Part
    def solid_mirrored(self):
        """Port wing solid — suppressed for vertical tail."""
        return LoftedSolid(
            profiles=[self.root_airfoil.geometry, self.tip_airfoil_mirrored.geometry],
            mesh_deflection=self.mesh_deflection,
            color=self.color_liftingsurface,
            transparency=0.6,
            suppress=self.is_vertical_tail,
        )

    # ------------------------------------------------------------ #
    # FRAME
    # ------------------------------------------------------------ #

    @Part
    def frame(self):
        return Frame(pos=self._root_position, hidden=False)

    # ------------------------------------------------------------ #
    # WINGBOX
    # ------------------------------------------------------------ #

    @Part
    def wingbox(self):
        """Starboard wingbox."""
        return Wingbox(
            c_root=self.c_root,
            c_tip=self.c_tip,
            semi_span=self.effective_span,
            sweep_le=self.sweep_le,
            dihedral=self.dihedral,
            twist=self.twist,
            front_spar_position=self.front_spar_position,
            rear_spar_position=self.rear_spar_position,
            airfoil_root=self.root_airfoil,
            airfoil_tip=self.tip_airfoil,
            color=self.color_wingbox,
        )

    @Part
    def wingbox_mirrored(self):
        """Port wingbox — suppressed for vertical tail."""
        return Wingbox(
            c_root=self.c_root,
            c_tip=self.c_tip,
            semi_span=-self.effective_span,
            sweep_le=-self.sweep_le,
            dihedral=-self.dihedral,
            twist=self.twist,
            front_spar_position=self.front_spar_position,
            rear_spar_position=self.rear_spar_position,
            airfoil_root=self.root_airfoil,
            airfoil_tip=self.tip_airfoil_mirrored,
            color=self.color_wingbox,
            suppress=self.is_vertical_tail,
        )


# ---------------------------------------------------------------------- #
# TEST
# ---------------------------------------------------------------------- #

if __name__ == "__main__":

    from parapy.gui import display

    # ============================================================ #
    # MAIN WING
    # ============================================================ #

    wing = LiftingSurface(
        label="main_wing",

        wing_area=20.0,
        semi_span=8.0,

        fuselage_length=10.0,
        fuselage_radius=1.0,

        is_tail=False,
        is_vertical_tail=False,

        mesh_deflection=1e-4,

        taper_ratio=0.40,
        sweep_le=5.0,
        twist=-2.0,
        dihedral=5.0,

        thickness_to_chord=0.15,
        maximum_camber=0.04,
        maximum_camber_position=0.4,

        t_factor_root=1.0,
        t_factor_tip=1.0,

        front_spar_position=0.15,
        rear_spar_position=0.60,

        wing_ref=None,
        
        color_wingbox="yellow",
        color_liftingsurface="orange"
    )

    # ============================================================ #
    # HORIZONTAL TAIL (fully Roskam-driven, correct usage)
    # ============================================================ #

    horizontal_tail = LiftingSurface(
        label="horizontal_tail",

        wing_ref=wing,
        is_tail=True,
        is_vertical_tail=False,

        fuselage_length=10.0,
        fuselage_radius=1.0,

        mesh_deflection=1e-4,

        wing_area=1.0,     # dummy but REQUIRED
        semi_span=1.0,     # dummy but REQUIRED

        taper_ratio=0.40,
        sweep_le=10.0,
        twist=0.0,
        dihedral=0.0,

        thickness_to_chord=0.15,
        maximum_camber=0.0,
        maximum_camber_position=0.4,

        t_factor_root=1.0,
        t_factor_tip=1.0,

        front_spar_position=0.15,
        rear_spar_position=0.60,

        tail_volume_coefficient_h=0.6,
        tail_aspect_ratio_h=4.5,

        tail_volume_coefficient_v=0.04,
        tail_aspect_ratio_v=1.8,
        
        color_wingbox="red",
        color_liftingsurface="green"
    )

    # ============================================================ #
    # VERTICAL TAIL
    # ============================================================ #

    vertical_tail = LiftingSurface(
        label="vertical_tail",

        wing_ref=wing,
        is_tail=True,
        is_vertical_tail=True,

        fuselage_length=10.0,
        fuselage_radius=1.0,

        mesh_deflection=1e-4,

        wing_area=1.0,
        semi_span=1.0,

        taper_ratio=0.40,
        sweep_le=35.0,
        twist=0.0,
        dihedral=0.0,

        thickness_to_chord=0.15,
        maximum_camber=0.0,
        maximum_camber_position=0.0,

        t_factor_root=1.0,
        t_factor_tip=1.0,

        front_spar_position=0.15,
        rear_spar_position=0.60,

        tail_volume_coefficient_v=0.04,
        tail_aspect_ratio_v=1.8,
        
        color_wingbox="blue",
        color_liftingsurface="purple"
    )

    display([wing, horizontal_tail, vertical_tail])