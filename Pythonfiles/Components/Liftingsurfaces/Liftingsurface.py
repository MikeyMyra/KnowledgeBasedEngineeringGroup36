from math import radians, tan, cos, sqrt

import numpy as np
from parapy.core import Input, Attribute, Part, child
from parapy.geom import GeomBase, LoftedSolid, translate, rotate, Point

if __name__ != "__main__":
    from Liftingsurfaces.Airfoil import Airfoil
    from Liftingsurfaces.Wingbox import Wingbox
    from Frame import Frame


class LiftingSurface(GeomBase):
    """Lifting surface geometry: root/tip airfoils + lofted solid + frame.

    Positioning convention (matches Fuselage):
      - Fuselage nose is at (0, 0, 0)
      - Fuselage runs along the +X axis
      - Z is up, Y is spanwise (starboard positive)
      - Wing/tail root sits on the fuselage surface:
            x = fuselage_length * attach_x_pct / 100
            z = fuselage_radius
    """

    # ------------------------------------------------------------------ #
    # INPUTS — AIRFOIL
    # ------------------------------------------------------------------ #

    airfoil_root_name: str = Input("whitcomb")
    airfoil_tip_name: str = Input("whitcomb")

    t_factor_root: float = Input(1.0)
    t_factor_tip: float = Input(1.0)

    mesh_deflection: float = Input(1e-4)

    # ------------------------------------------------------------------ #
    # INPUTS — PLANFORM (provide these OR let sizing compute from area)
    # ------------------------------------------------------------------ #

    # Primary sizing inputs — always required
    wing_area: float = Input()          # reference area (one side) [m²]
    semi_span: float = Input()          # half-span [m]
    taper_ratio: float = Input(0.3)     # c_tip / c_root [-]

    # Angles
    sweep_le: float = Input(25.0)       # leading-edge sweep [deg]
    twist: float = Input(-2.0)          # washout at tip [deg], negative = wash-out
    dihedral: float = Input(5.0)        # dihedral [deg]

    # Aerofoil section properties (used for structural sizing)
    thickness_to_chord: float = Input(0.12)     # t/c at root [-]
    maximum_camber: float = Input(0.04)         # max camber [-]
    maximum_camber_position: float = Input(0.4) # x/c of max camber [-]

    # Wingbox spar positions (x/c fractions)
    front_spar_position: float = Input(0.15)    # x/c front spar [-]
    rear_spar_position: float = Input(0.60)     # x/c rear spar [-]

    # Materials
    material_wing: str = Input("aluminium")
    material_wingbox: str = Input("aluminium")

    # ------------------------------------------------------------------ #
    # INPUTS — FUSELAGE INTERFACE
    # ------------------------------------------------------------------ #

    fuselage_length: float = Input(30.0)    # total fuselage length [m]
    fuselage_radius: float = Input(2.0)     # fuselage radius at attach point [m]

    # Fraction of fuselage length from nose to main-gear attach;
    # used to estimate CG and derive wing x-position.
    # For a tail surface, set is_tail=True and the position is estimated differently.
    is_tail: bool = Input(False)

    # ------------------------------------------------------------------ #
    # ESTIMATED ATTACHMENT POSITION
    # ------------------------------------------------------------------ #

    @Attribute
    def attach_x(self) -> float:
        """Estimated x-position of the root LE along the fuselage [m].

        Main wing:  placed so the quarter-chord of the MAC is at ~40% of
                    fuselage length — a reasonable initial CG target for a
                    tube-and-wing layout.
        Tail:       placed at ~85% of fuselage length (typical for a
                    conventional empennage).

        Both are purely geometric first-estimates; update with a proper
        CG / static-margin calculation once masses are known.
        """
        if self.is_tail:
            return 0.85 * self.fuselage_length
        else:
            # x_LE = x_quarter_chord_MAC - 0.25 * MAC - mac_x_offset
            x_qc_mac = 0.40 * self.fuselage_length
            return x_qc_mac - 0.25 * self.mean_aerodynamic_chord - self.mac_x_offset

    @Attribute
    def attach_z(self) -> float:
        """Z-position of the root LE = fuselage radius (surface of fuselage) [m]."""
        return self.fuselage_radius

    # ------------------------------------------------------------------ #
    # PLANFORM SIZING
    # ------------------------------------------------------------------ #

    @Attribute
    def c_root(self) -> float:
        """Root chord derived from area, span and taper ratio [m]."""
        return (2 * self.wing_area) / (self.semi_span * (1 + self.taper_ratio))

    @Attribute
    def c_tip(self) -> float:
        """Tip chord [m]."""
        return self.c_root * self.taper_ratio

    @Attribute
    def aspect_ratio(self) -> float:
        """Aspect ratio based on full span [-]."""
        return (2 * self.semi_span) ** 2 / (2 * self.wing_area)

    @Attribute
    def mean_aerodynamic_chord(self) -> float:
        """Mean aerodynamic chord [m]."""
        tr = self.taper_ratio
        return (2 / 3) * self.c_root * (1 + tr + tr ** 2) / (1 + tr)

    @Attribute
    def mac_spanwise_position(self) -> float:
        """Spanwise position of the MAC from root [m]."""
        tr = self.taper_ratio
        return self.semi_span * (1 + 2 * tr) / (3 * (1 + tr))

    @Attribute
    def mac_x_offset(self) -> float:
        """Chordwise offset of MAC leading edge from root leading edge [m]
        (due to sweep)."""
        return self.mac_spanwise_position * tan(radians(self.sweep_le))

    @Attribute
    def sweep_quarter_chord(self) -> float:
        """Quarter-chord sweep angle [deg], derived from LE sweep."""
        # tan(sweep_c/4) = tan(sweep_LE) - (1/AR) * (1 - taper) / (1 + taper)
        tr = self.taper_ratio
        tan_qc = tan(radians(self.sweep_le)) - (1.0 / self.aspect_ratio) * (1 - tr) / (1 + tr)
        return float(np.degrees(np.arctan(tan_qc)))

    @Attribute
    def sweep_half_chord(self) -> float:
        """Half-chord sweep angle [deg]."""
        tr = self.taper_ratio
        tan_hc = tan(radians(self.sweep_le)) - (2.0 / self.aspect_ratio) * (1 - tr) / (1 + tr)
        return float(np.degrees(np.arctan(tan_hc)))

    # ------------------------------------------------------------------ #
    # WINGBOX SIZING
    # ------------------------------------------------------------------ #

    @Attribute
    def wingbox_chord_root(self) -> float:
        """Wingbox chord width at root [m]."""
        return (self.rear_spar_position - self.front_spar_position) * self.c_root

    @Attribute
    def wingbox_chord_tip(self) -> float:
        """Wingbox chord width at tip [m]."""
        return (self.rear_spar_position - self.front_spar_position) * self.c_tip

    @Attribute
    def wingbox_height_root(self) -> float:
        """Wingbox height at root (= t/c * local chord) [m]."""
        return self.thickness_to_chord * self.c_root

    @Attribute
    def wingbox_height_tip(self) -> float:
        """Wingbox height at tip [m]."""
        return self.thickness_to_chord * self.c_tip

    @Attribute
    def front_spar_x_root(self) -> float:
        """Absolute x-position of front spar at root (local frame) [m]."""
        return self.front_spar_position * self.c_root

    @Attribute
    def rear_spar_x_root(self) -> float:
        """Absolute x-position of rear spar at root (local frame) [m]."""
        return self.rear_spar_position * self.c_root

    # ------------------------------------------------------------------ #
    # MASS ESTIMATE  (Raymer simple wing weight, placeholder)
    # ------------------------------------------------------------------ #

    @Attribute
    def mass(self) -> float:
        """Rough structural mass estimate [kg].
        Replace with a proper class-II method when available.
        Uses: m ≈ 0.0215 * S^0.9 * AR^0.4  (S in m², result in kg, very approximate).
        """
        S_full = 2 * self.wing_area
        return 0.0215 * (S_full ** 0.9) * (self.aspect_ratio ** 0.4)

    # ------------------------------------------------------------------ #
    # POSITION HELPERS
    # ------------------------------------------------------------------ #

    @Attribute
    def _root_position(self):
        """Root LE position: on top of fuselage at the estimated attach station."""
        return translate(
            self.position,
            "x", self.attach_x,
            "z", self.attach_z,
        )

    @Attribute
    def _tip_position(self):
        """Tip airfoil position accounting for sweep, dihedral and twist."""
        return rotate(
            translate(
                self._root_position,
                "y", self.semi_span,
                "x", self.semi_span * tan(radians(self.sweep_le)),
                "z", self.semi_span * np.sin(radians(self.dihedral)),
            ), "y", radians(self.twist)
        )

    @Attribute
    def _tip_position_mirrored(self):
        """Tip position for the port (mirrored) side — y negated."""
        return rotate(
            translate(
                self._root_position,
                "y", -self.semi_span,
                "x", self.semi_span * tan(radians(self.sweep_le)),
                "z", self.semi_span * np.sin(radians(self.dihedral)),
            ), "y", radians(self.twist)
        )

    # ------------------------------------------------------------------ #
    # AIRFOIL PROFILES
    # ------------------------------------------------------------------ #

    @Part
    def root_airfoil(self):
        return Airfoil(
            airfoil_name=self.airfoil_root_name,
            chord=self.c_root,
            thickness_factor=self.t_factor_root,
            mesh_deflection=self.mesh_deflection,
            position=self._root_position,
        )

    @Part
    def tip_airfoil(self):
        return Airfoil(
            airfoil_name=self.airfoil_tip_name,
            chord=self.c_tip,
            thickness_factor=self.t_factor_tip,
            mesh_deflection=self.mesh_deflection,
            position=self._tip_position,
        )

    @Part
    def tip_airfoil_mirrored(self):
        return Airfoil(
            airfoil_name=self.airfoil_tip_name,
            chord=self.c_tip,
            thickness_factor=self.t_factor_tip,
            mesh_deflection=self.mesh_deflection,
            position=self._tip_position_mirrored,
        )

    # ------------------------------------------------------------------ #
    # LOFTED SOLIDS
    # ------------------------------------------------------------------ #

    @Attribute
    def _loft_profiles(self):
        return [self.root_airfoil.geometry, self.tip_airfoil.geometry]

    @Attribute
    def _loft_profiles_mirrored(self):
        return [self.root_airfoil.geometry, self.tip_airfoil_mirrored.geometry]

    @Part
    def solid(self):
        return LoftedSolid(
            profiles=self._loft_profiles,
            color="LightBlue",
            transparency=0.5,
            mesh_deflection=self.mesh_deflection,
        )

    @Part
    def solid_mirrored(self):
        return LoftedSolid(
            profiles=self._loft_profiles_mirrored,
            color="LightBlue",
            mesh_deflection=self.mesh_deflection,
        )

    # ------------------------------------------------------------------ #
    # FRAME VISUALISATION
    # ------------------------------------------------------------------ #

    @Part
    def frame(self):
        return Frame(
            pos=self._root_position,
            hidden=False,
        )

    # ------------------------------------------------------------------ #
    # WINGBOX
    # ------------------------------------------------------------------ #

    @Part
    def wingbox(self):
        return Wingbox(
            c_root=self.c_root,
            c_tip=self.c_tip,
            semi_span=self.semi_span,
            sweep_le=self.sweep_le,
            dihedral=self.dihedral,
            twist=self.twist,
            front_spar_position=self.front_spar_position,
            rear_spar_position=self.rear_spar_position,
            thickness_to_chord=self.thickness_to_chord,
            material=self.material_wingbox,
            mesh_deflection=self.mesh_deflection,
            position=self._root_position,
            airfoil_root=self.root_airfoil,
            airfoil_tip=self.tip_airfoil
        )

    @Part
    def wingbox_mirrored(self):
        return Wingbox(
            c_root=self.c_root,
            c_tip=self.c_tip,
            semi_span=-self.semi_span,      # negative Y = port side
            sweep_le=-self.sweep_le,        # mirror sweep
            dihedral=-self.dihedral,        # mirror dihedral
            twist=self.twist,
            front_spar_position=self.front_spar_position,
            rear_spar_position=self.rear_spar_position,
            thickness_to_chord=self.thickness_to_chord,
            material=self.material_wingbox,
            mesh_deflection=self.mesh_deflection,
            position=self._root_position,
            airfoil_root=self.root_airfoil,
            airfoil_tip=self.tip_airfoil_mirrored
        )


# ---------------------------------------------------------------------- #
# TEST
# ---------------------------------------------------------------------- #

if __name__ == "__main__":
    from parapy.gui import display

    from Airfoil import Airfoil
    from Frame import Frame
    from Wingbox import Wingbox

    ls = LiftingSurface(
        label="main_wing",
        wing_area=122.0,
        semi_span=17.0,
        taper_ratio=0.3,
        sweep_le=27.0,
        twist=-2.0,
        dihedral=5.0,
        thickness_to_chord=0.12,
        fuselage_length=37.0,
        fuselage_radius=2.0,
        mesh_deflection=1e-4,
    )
    display(ls)