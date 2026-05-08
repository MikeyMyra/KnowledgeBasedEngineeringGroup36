from math import radians, tan, cos, sin
import numpy as np

from parapy.core import Input, Attribute, Part
from parapy.geom import GeomBase, LoftedSolid, translate, rotate

from Pythonfiles.Components.Liftingsurfaces.Airfoil import Airfoil
from Pythonfiles.Components.Liftingsurfaces.Wingbox import Wingbox
from Pythonfiles.Components.Frame import Frame


class LiftingSurface(GeomBase):
    """
    Lifting surface for wing, horizontal tail, and vertical tail.

    Aero / geometric split
    ----------------------
    For ALL surfaces (wing and tails):

        c_root_geometric  : chord at the root attachment line (centreline for
                            wing, fuselage side for tails).  The wingbox starts
                            here and extends to the tip — it passes through the
                            fuselage wall.

        c_root_aero       : chord at the fuselage wall (wing) or identical to
                            c_root_geometric (tails, which attach at the wall).
                            The lofted surface solid starts here.

        _effective_span   : exposed semi-span from fuselage wall to tip.
        _effective_area   : exposed trapezoid area (both sides for wing/HT,
                            full span for VT).

    Wing sizing (is_tail=False)
    ---------------------------
        effective_area and effective_span are direct inputs.

    Tail sizing (is_tail=True)
    --------------------------
        effective_area and effective_span are computed internally from the
        Roskam tail-volume coefficients (tail_volume_coefficient_h/v) and
        tail_aspect_ratio_h/v, driven by wing_ref geometry.
        Pass effective_area=None and effective_span=None (the defaults).

    Wingbox vs surface solid
    ------------------------
        wingbox          : spans from centreline (wing) or fuselage wall (tail)
                           all the way to the tip, using c_root_geometric.
        solid / mirrored : spans from fuselage wall to tip only, using
                           c_root_aero.  No skin is buried inside the fuselage.
    """

    # ------------------------------------------------------------ #
    # PRIMARY SIZING INPUTS
    # Wing: provide these. Tails: leave as None (Roskam sizing used).
    # ------------------------------------------------------------ #

    effective_area: float = Input(None)
    effective_span: float = Input(None)

    fuselage_length: float = Input()
    fuselage_radius: float = Input()

    # Optional callable x → local_radius [m] from Fuselage.local_radius_at.
    # When provided, tails use the cone radius at their attach_x rather than
    # the constant cylinder radius.  Wings always sit on the cylinder, so
    # they are unaffected even when this is set.
    # Pass None (default) to keep the original constant-radius behaviour.
    fuselage_cone_radius_fn: object = Input(None)

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
    # ROSKAM TAIL PARAMETERS  (tails only — keep original names)
    # ------------------------------------------------------------ #

    tail_volume_coefficient_h: float = Input(None)
    tail_volume_coefficient_v: float = Input(None)
    tail_aspect_ratio_h: float = Input(None)
    tail_aspect_ratio_v: float = Input(None)

    wing_ref: "LiftingSurface" = Input(None)

    # Kept for compatibility — ignored when Roskam sizing is active
    effective_wing_area: float = Input(None)
    effective_semi_span: float = Input(None)

    # ------------------------------------------------------------ #
    # ATTACH POSITION
    # ------------------------------------------------------------ #

    @Attribute
    def attach_x(self):
        if self.is_tail:
            raw = self.wing_ref.attach_x + self.tail_arm
            return min(raw, self.fuselage_length - self.c_root_aero)
        return 0.40 * self.fuselage_length

    @Attribute
    def attach_z(self):
        """
        Z-position of the surface root attachment on the fuselage [m].

        Wing            : 0.0  (mid-fuselage on the cylinder — no z offset needed)
        Horizontal tail : 0.0  (symmetric about the fuselage centreline)
        Vertical tail   : +fuselage_wall_radius  (sits on TOP of the cone surface)

        fuselage_wall_radius is the local cone radius at attach_x, so the VT
        root chord begins flush with the fuselage skin rather than floating in
        mid-air at a fixed cylinder radius.
        """
        if self.is_tail and self.is_vertical_tail:
            return self.fuselage_wall_radius
        return 0.0

    # ------------------------------------------------------------ #
    # TAIL ARM
    # ------------------------------------------------------------ #

    @Attribute
    def tail_arm(self):
        if not self.is_tail:
            return None
        wing_x = self.wing_ref.attach_x
        max_arm = self.fuselage_length - wing_x
        return min(0.65 * self.wing_ref._effective_span * 2, max_arm)

    # ------------------------------------------------------------ #
    # LOCAL FUSELAGE WALL RADIUS AT ATTACH POINT
    # ------------------------------------------------------------ #

    @Attribute
    def fuselage_wall_radius(self):
        """
        Fuselage cross-section radius [m] at this surface's attach_x.

        Wing  : always fuselage_radius (attaches on the constant cylinder).
        Tails : uses fuselage_cone_radius_fn(attach_x) when the callable is
                supplied, so the skin root and VT base track the converging
                tailcone profile exactly.  Falls back to fuselage_radius if
                the callable is not provided (preserves old behaviour).

        This value drives:
          - The y-offset of the HT skin root (±fuselage_wall_radius in y)
          - The z-offset of the VT skin root (+fuselage_wall_radius in z)
          - The wingbox burial depth (_geometric_span = _effective_span
            + fuselage_wall_radius) so the box reaches all the way to the
            structural centreline.
        """
        if self.is_tail and self.fuselage_cone_radius_fn is not None:
            return self.fuselage_cone_radius_fn(self.attach_x + self.c_root_aero)
        return self.fuselage_radius

    # ------------------------------------------------------------ #
    # RESOLVED EFFECTIVE AREA AND SPAN
    # ------------------------------------------------------------ #

    @Attribute
    def _effective_area(self):
        """
        Resolved exposed reference area [m^2].

        Wing  : direct from effective_area input.
        Tail  : Roskam tail-volume formula.
                HT: S_h = Vh * S_w * c_w / tail_arm
                VT: S_v = Vv * S_w * b_w / tail_arm
        """
        if not self.is_tail:
            if self.effective_area is None:
                raise ValueError("Wing requires effective_area input.")
            return self.effective_area

        if self.wing_ref is None:
            raise ValueError("Tail requires wing_ref.")

        S_w = self.wing_ref._effective_area
        b_w = 2 * self.wing_ref._effective_span
        c_w = self.wing_ref.mean_aerodynamic_chord

        if self.is_vertical_tail:
            if self.tail_volume_coefficient_v is None or self.tail_aspect_ratio_v is None:
                raise ValueError("VT requires tail_volume_coefficient_v and tail_aspect_ratio_v.")
            return (self.tail_volume_coefficient_v * S_w * b_w) / self.tail_arm
        else:
            if self.tail_volume_coefficient_h is None or self.tail_aspect_ratio_h is None:
                raise ValueError("HT requires tail_volume_coefficient_h and tail_aspect_ratio_h.")
            return (self.tail_volume_coefficient_h * S_w * c_w) / self.tail_arm

    @Attribute
    def _effective_span(self):
        """
        Resolved exposed semi-span [m]  (full span for VT).

        Wing  : direct from effective_span input.
        HT    : sqrt(AR_h * S_h) / 2
        VT    : sqrt(AR_v * S_v)
        """
        if not self.is_tail:
            if self.effective_semi_span is None:
                raise ValueError("Wing requires effective_span input.")
            return self.effective_semi_span

        if self.is_vertical_tail:
            return np.sqrt(self.tail_aspect_ratio_v * self._effective_area)
        else:
            return np.sqrt(self.tail_aspect_ratio_h * self._effective_area) / 2

    # ------------------------------------------------------------ #
    # CHORD
    # ------------------------------------------------------------ #

    @Attribute
    def c_root_aero(self):
        """
        Chord at the fuselage wall [m].

        Wing : linear interpolation from centreline to tip, evaluated at
               y = fuselage_radius.
        Tail : the entire exposed surface starts at the fuselage wall, so
               c_root_aero is derived directly from the exposed trapezoid:
               S_exposed = (c_root_aero + c_tip) * span
               c_root_aero = S / (span * (1 + taper))
        """
        return self._effective_area / (self._effective_span * (1 + self.taper_ratio))

    @Attribute
    def c_tip(self):
        return self.c_root_aero * self.taper_ratio

    @Attribute
    def c_root_geometric(self):
        """
        Chord at the structural root [m] — wingbox starts here.

        Extrapolates the taper inward from the fuselage wall by
        fuselage_wall_radius (the local cone radius at attach_x).
        For the wing this equals fuselage_radius; for tails on the
        converging tailcone it is the smaller local radius, so the
        buried chord length is correctly proportioned to the actual
        burial depth.
        """
        slope = (self.c_root_aero - self.c_tip) / self._effective_span
        return self.c_root_aero + slope * self.fuselage_wall_radius

    # ------------------------------------------------------------ #
    # FULL GEOMETRIC SEMI-SPAN  (including fuselage burial)
    # ------------------------------------------------------------ #

    @Attribute
    def _geometric_span(self):
        """
        Semi-span from structural centreline to tip [m].

        = _effective_span + fuselage_wall_radius

        fuselage_wall_radius is the local cone radius at attach_x, so the
        wingbox burial depth matches the actual fuselage cross-section at
        that station rather than the constant cylinder radius.
        """
        return self._effective_span + self.fuselage_wall_radius

    # ------------------------------------------------------------ #
    # DERIVED AERO PROPERTIES
    # ------------------------------------------------------------ #

    @Attribute
    def aspect_ratio(self):
        return (2 * self._effective_span) ** 2 / self._effective_area

    @Attribute
    def mean_aerodynamic_chord(self):
        tr = self.taper_ratio
        return (2 / 3) * self.c_root_aero * (1 + tr + tr ** 2) / (1 + tr)

    @Attribute
    def mac_spanwise_position(self):
        tr = self.taper_ratio
        return self._effective_span * (1 + 2 * tr) / (3 * (1 + tr))

    @Attribute
    def mac_x_offset(self):
        return self.mac_spanwise_position * tan(radians(self.sweep_le))

    # ------------------------------------------------------------ #
    # POSITIONING HELPERS
    # ------------------------------------------------------------ #

    def _spanwise_offsets(self, y_sign: float):
        """
        x/y/z offsets when stepping one fuselage_wall_radius outboard from
        the centreline attachment, following sweep and dihedral.
        y_sign = +1 for starboard, -1 for port.

        Uses fuselage_wall_radius (= local cone radius at attach_x) so that
        the skin root sits exactly on the fuselage surface at that station.
        For the wing this equals fuselage_radius; for tails it is smaller.
        """
        R  = self.fuselage_wall_radius
        dx = R * tan(radians(self.sweep_le))
        dy = y_sign * R
        dz = R * sin(radians(self.dihedral))
        return dx, dy, dz

    @Attribute
    def _root_position_wingbox(self):
        """
        Structural root position — at (attach_x, 0, attach_z) with NO
        sweep or dihedral pre-offset.

        Why no offset?  The Wingbox Part internally propagates sweep and
        dihedral over its full semi_span = _geometric_span.  It starts from
        this position and arrives at the same tip z/x as the skin, because:

            wb_root_z + geometric_span * sin(dihedral)
          = attach_z  + (effective_span + R) * sin(dihedral)
          = (attach_z + R*sin(d)) + effective_span * sin(d)
          = skin_root_z + effective_span * sin(d)
          = skin_tip_z                                         ✓

        Adding a -dz_out offset to wb_root_z would cause the wingbox tip to
        undershoot skin_tip_z by fuselage_radius * sin(dihedral), producing
        the misalignment that was observed.  The same cancellation holds for x.

        For the VT (no dihedral/twist): same logic, just rotated upright.
        """
        if not self.is_vertical_tail:
            return translate(
                self.position,
                "x", self.attach_x,
                "y", 0.0,
                "z", self.attach_z,
            )
        # VT: root at fuselage centreline, rotated upright
        base = translate(self.position, "x", self.attach_x, "z", self.attach_z)
        return rotate(base, "x", radians(90))

    @Attribute
    def _root_position(self):
        """
        Starboard fuselage-wall position — aero surface (skin) starts here.

        Moves one fuselage_radius outboard from the centreline attachment
        following sweep (dx) and dihedral (dz), so this position is exactly
        one fuselage_radius further outboard than _root_position_wingbox.
        """
        if not self.is_vertical_tail:
            dx, dy, dz = self._spanwise_offsets(+1)
            return translate(
                self.position,
                "x", self.attach_x + dx,
                "y", dy,
                "z", self.attach_z + dz,
            )
        # VT root is at the top of the fuselage wall, rotated upright.
        # attach_z is already set to fuselage_wall_radius by attach_z attribute,
        # so the base translate lands on the cone surface.  The small sweep dx
        # along x is also scaled by the local wall radius.
        R  = self.fuselage_wall_radius
        dx = R * tan(radians(self.sweep_le))
        dz = R  # step up to cone surface top — same as fuselage_wall_radius
        base = translate(self.position, "x", self.attach_x + dx, "z", self.attach_z + dz)
        return rotate(base, "x", radians(90))

    @Attribute
    def _root_position_mirrored(self):
        """
        Port fuselage-wall position — mirror of _root_position.

        y is negated (port side), but dz is identical: dihedral raises
        both wing panels upward symmetrically.
        """
        if self.is_vertical_tail:
            return self._root_position
        dx, dy, dz = self._spanwise_offsets(-1)
        return translate(
            self.position,
            "x", self.attach_x + dx,
            "y", dy,
            "z", self.attach_z + dz,
        )

    @Attribute
    def _tip_position(self):
        """Starboard tip position."""
        if not self.is_vertical_tail:
            return rotate(
                translate(
                    self._root_position,
                    "y",  self._effective_span,
                    "x",  self._effective_span * tan(radians(self.sweep_le)),
                    "z",  self._effective_span * np.sin(radians(self.dihedral)),
                ),
                "y", radians(self.twist),
            )
        return rotate(
            translate(
                self._root_position,
                "y", self._effective_span,
                "x", self._effective_span * tan(radians(self.sweep_le)),
            ),
            "z", radians(self.twist),
        )

    @Attribute
    def _tip_position_mirrored(self):
        """Port tip position — None for vertical tail."""
        if self.is_vertical_tail:
            return None
        return rotate(
            translate(
                self._root_position_mirrored,
                "y", -self._effective_span,
                "x",  self._effective_span * tan(radians(self.sweep_le)),
                "z",  self._effective_span * np.sin(radians(self.dihedral)),
            ),
            "y", radians(self.twist),
        )

    # ------------------------------------------------------------ #
    # AIRFOILS
    # ------------------------------------------------------------ #

    @Part
    def root_airfoil_wingbox(self):
        """Structural root airfoil — at fuselage wall minus one radius inboard."""
        return Airfoil(
            chord=self.c_root_geometric,
            maximum_camber=self.maximum_camber,
            camber_position=self.maximum_camber_position,
            thickness_to_chord=self.thickness_to_chord,
            export_dat=True,
            airfoil_name="root_airfoil_geometric",
            position=self._root_position_wingbox,
        )

    @Part
    def root_airfoil(self):
        """Fuselage-wall airfoil — starboard aero surface root."""
        return Airfoil(
            chord=self.c_root_aero,
            maximum_camber=self.maximum_camber,
            camber_position=self.maximum_camber_position,
            thickness_to_chord=self.thickness_to_chord,
            export_dat=True,
            airfoil_name="root_airfoil_aero",
            position=self._root_position,
        )

    @Part
    def root_airfoil_mirrored(self):
        """Fuselage-wall airfoil — port aero surface root."""
        return Airfoil(
            chord=self.c_root_aero,
            maximum_camber=self.maximum_camber,
            camber_position=self.maximum_camber_position,
            thickness_to_chord=self.thickness_to_chord,
            export_dat=True,
            airfoil_name="root_airfoil_aero_mirrored",
            position=self._root_position_mirrored,
            suppress=self.is_vertical_tail,
        )

    @Part
    def tip_airfoil(self):
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
    # SOLIDS  (fuselage wall to tip — no buried skin)
    # ------------------------------------------------------------ #

    @Part
    def solid(self):
        return LoftedSolid(
            profiles=[self.root_airfoil.geometry, self.tip_airfoil.geometry],
            color=self.color_liftingsurface,
            mesh_deflection=self.mesh_deflection,
        )

    @Part
    def solid_mirrored(self):
        return LoftedSolid(
            profiles=[self.root_airfoil_mirrored.geometry, self.tip_airfoil_mirrored.geometry],
            color=self.color_liftingsurface,
            transparency=0.6,
            mesh_deflection=self.mesh_deflection,
            suppress=self.is_vertical_tail,
        )

    # ------------------------------------------------------------ #
    # FRAME
    # ------------------------------------------------------------ #

    @Part
    def frame(self):
        return Frame(pos=self._root_position, hidden=False)

    # ------------------------------------------------------------ #
    # WINGBOX  (structural root to tip — passes through fuselage wall)
    # ------------------------------------------------------------ #

    @Part
    def wingbox(self):
        return Wingbox(
            c_root=self.c_root_geometric,
            c_tip=self.c_tip,
            semi_span=self._geometric_span,
            sweep_le=self.sweep_le,
            dihedral=self.dihedral,
            twist=self.twist,
            front_spar_position=self.front_spar_position,
            rear_spar_position=self.rear_spar_position,
            airfoil_root=self.root_airfoil_wingbox,
            airfoil_tip=self.tip_airfoil,
            color=self.color_wingbox,
        )

    @Part
    def wingbox_mirrored(self):
        return Wingbox(
            c_root=self.c_root_geometric,
            c_tip=self.c_tip,
            semi_span=-self._geometric_span,
            sweep_le=-self.sweep_le,
            dihedral=-self.dihedral,
            twist=self.twist,
            front_spar_position=self.front_spar_position,
            rear_spar_position=self.rear_spar_position,
            airfoil_root=self.root_airfoil_wingbox,
            airfoil_tip=self.tip_airfoil_mirrored,
            color=self.color_wingbox,
            suppress=self.is_vertical_tail,
        )


# ---------------------------------------------------------------------- #
# TEST
# ---------------------------------------------------------------------- #

if __name__ == "__main__":

    from parapy.gui import display

    wing = LiftingSurface(
        label="main_wing",

        effective_area=18.0,
        effective_span=7.4,

        fuselage_length=10.0,
        fuselage_radius=0.6,

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

        color_wingbox="yellow",
        color_liftingsurface="orange",
    )

    horizontal_tail = LiftingSurface(
        label="horizontal_tail",

        wing_ref=wing,
        is_tail=True,
        is_vertical_tail=False,

        # Roskam sizing — matches the calling convention in Aircraft
        tail_volume_coefficient_h=0.60,
        tail_volume_coefficient_v=0.04,
        tail_aspect_ratio_h=4.5,
        tail_aspect_ratio_v=1.8,

        fuselage_length=10.0,
        fuselage_radius=0.6,

        mesh_deflection=1e-4,

        taper_ratio=0.40,
        sweep_le=10.0,
        twist=0.0,
        dihedral=0.0,

        thickness_to_chord=0.12,
        maximum_camber=0.0,
        maximum_camber_position=0.4,

        t_factor_root=1.0,
        t_factor_tip=1.0,

        front_spar_position=0.15,
        rear_spar_position=0.60,

        color_wingbox="red",
        color_liftingsurface="green",
    )

    vertical_tail = LiftingSurface(
        label="vertical_tail",

        wing_ref=wing,
        is_tail=True,
        is_vertical_tail=True,

        tail_volume_coefficient_h=0.60,
        tail_volume_coefficient_v=0.04,
        tail_aspect_ratio_h=4.5,
        tail_aspect_ratio_v=1.8,

        fuselage_length=10.0,
        fuselage_radius=0.6,

        mesh_deflection=1e-4,

        taper_ratio=0.40,
        sweep_le=35.0,
        twist=0.0,
        dihedral=0.0,

        thickness_to_chord=0.12,
        maximum_camber=0.0,
        maximum_camber_position=0.0,

        t_factor_root=1.0,
        t_factor_tip=1.0,

        front_spar_position=0.15,
        rear_spar_position=0.60,

        color_wingbox="blue",
        color_liftingsurface="purple",
    )

    display([wing, horizontal_tail, vertical_tail])