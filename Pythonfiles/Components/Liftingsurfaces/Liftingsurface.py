from math import radians, tan

import numpy as np
from parapy.core import Input, Attribute, Part
from parapy.geom import GeomBase, LoftedSolid, translate, rotate

from Pythonfiles.Components.Liftingsurfaces.Airfoil import Airfoil
from Pythonfiles.Components.Liftingsurfaces.Wingbox import Wingbox
from Pythonfiles.Components.Frame import Frame


class LiftingSurface(GeomBase):
    """Lifting surface geometry: root/tip airfoils + lofted solid + frame.

    Roskam defaults (Vol. I):
    - taper_ratio       : Table 3.6 / Fig. 3.15 — typical UAV/GA: 0.40 (flagged)
    - sweep_le          : Table 3.6 — low-speed fixed-wing UAV: 5° (flagged)
    - twist             : Table 3.6 — typical washout: −2° (flagged)
    - dihedral          : Table 3.6 — low-wing GA/UAV: 5° (flagged)
    - thickness_to_chord: Table 3.5 — subsonic fixed-wing: 0.15 (flagged)
    - maximum_camber    : typical NACA 4-series stats: 0.04 (flagged)
    - front/rear spar   : Roskam Vol. I §4.1 structural layout guidelines (flagged)
    - attach_x (wing)   : Roskam Vol. I §3.2 — MAC quarter chord at 40% fuselage length
    - attach_x (tail)   : Roskam Vol. I §3.4 — tail LE at 85% fuselage length
    """

    airfoil_root_name: str = Input("whitcomb")
    airfoil_tip_name: str = Input("whitcomb")

    t_factor_root: float = Input(1.0)
    t_factor_tip: float = Input(1.0)

    mesh_deflection: float = Input(1e-4)

    # ------------------------------------------------------------------ #
    # PRIMARY SIZING — always required from user
    # ------------------------------------------------------------------ #

    wing_area: float = Input()      # reference area (one side) [m²]
    semi_span: float = Input()      # half-span [m]

    # ------------------------------------------------------------------ #
    # PLANFORM ANGLES & RATIOS — Roskam Vol. I statistical defaults
    # ------------------------------------------------------------------ #

    # Roskam Vol. I, Table 3.6: taper ratio for low-speed fixed-wing ~ 0.40.
    # NOTE (Roskam): value is statistical; adjust for high-AR sailplane/UAV.
    taper_ratio: float = Input(0.40)

    # Roskam Vol. I, Table 3.6: LE sweep for subsonic UAV typically 0–10°.
    # NOTE (Roskam): increase for higher cruise Mach; 5° is conservative default.
    sweep_le: float = Input(5.0)

    # Roskam Vol. I, Table 3.6: geometric twist (washout) typically −2° to −3°.
    # NOTE (Roskam): negative = wash-out at tip, improves stall characteristics.
    twist: float = Input(-2.0)

    # Roskam Vol. I, Table 3.6: dihedral for low-wing configuration ~ 5°.
    # NOTE (Roskam): high-wing configurations use 0–2°; mid-wing 2–4°.
    dihedral: float = Input(5.0)

    # ------------------------------------------------------------------ #
    # AIRFOIL SECTION PROPERTIES — Roskam Vol. I statistical defaults
    # ------------------------------------------------------------------ #

    # Roskam Vol. I, Table 3.5: t/c for subsonic fixed-wing typically 0.12–0.18.
    # NOTE (Roskam): thicker sections increase structural depth; 0.15 is midrange.
    thickness_to_chord: float = Input(0.15)

    # NOTE (Roskam default not explicit): 0.04 is representative of NACA 4-series
    # sections commonly used in GA/UAV; flag and verify against chosen airfoil.
    maximum_camber: float = Input(0.04)

    # NOTE (Roskam default not explicit): 0.4c is typical for NACA 4-series.
    maximum_camber_position: float = Input(0.4)

    # ------------------------------------------------------------------ #
    # WINGBOX SPAR POSITIONS — Roskam Vol. I §4.1 structural layout
    # ------------------------------------------------------------------ #

    # Roskam Vol. I, §4.1: front spar typically at 15–20% chord.
    # NOTE (Roskam): 0.15 is the forward practical limit to preserve LE devices.
    front_spar_position: float = Input(0.15)

    # Roskam Vol. I, §4.1: rear spar typically at 55–65% chord.
    # NOTE (Roskam): 0.60 preserves room for trailing-edge control surfaces.
    rear_spar_position: float = Input(0.60)

    # ------------------------------------------------------------------ #
    # FUSELAGE INTERFACE — required for attachment positioning
    # ------------------------------------------------------------------ #

    fuselage_length: float = Input(30.0)    # total fuselage length [m]
    fuselage_radius: float = Input(2.0)     # fuselage radius at attach point [m]

    is_tail: bool = Input(False)

    # ------------------------------------------------------------------ #
    # ESTIMATED ATTACHMENT POSITION
    # ------------------------------------------------------------------ #

    @Attribute
    def attach_x(self) -> float:
        """Estimated x-position of the root LE along the fuselage [m].

        Roskam Vol. I, §3.2 (wing): MAC quarter chord placed at ~40% fuselage length.
        Roskam Vol. I, §3.4 (tail): tail LE placed at ~85% fuselage length.
        """
        if self.is_tail:
            # Roskam Vol. I, §3.4: horizontal/vertical tail LE at 85% fuselage length.
            return 0.85 * self.fuselage_length
        else:
            # Roskam Vol. I, §3.2: wing quarter-chord of MAC at 40% fuselage length.
            x_qc_mac = 0.40 * self.fuselage_length
            return x_qc_mac - 0.25 * self.mean_aerodynamic_chord - self.mac_x_offset

    @Attribute
    def attach_z(self) -> float:
        """Z-position of the root LE = fuselage radius (surface of fuselage) [m]."""
        return -self.fuselage_radius * 0.5

    # ------------------------------------------------------------------ #
    # MASS ESTIMATE
    # ------------------------------------------------------------------ #

    @Attribute
    def mass(self) -> float:
        """Rough structural mass estimate [kg].

        NOTE (Roskam default not explicit): Raymer simple wing weight equation
        used as placeholder. Replace with Roskam Vol. V Chapter 10 for detail.
        """
        S_full = 1 * self.wing_area
        return 0.0215 * (S_full ** 0.9) * (self.aspect_ratio ** 0.4)

    # ------------------------------------------------------------------ #
    # PLANFORM SIZING
    # ------------------------------------------------------------------ #

    @Attribute
    def c_root(self) -> float:
        """Root chord derived from area, span and taper ratio [m]."""
        return (1 * self.wing_area) / (self.semi_span * (1 + self.taper_ratio))

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
            maximum_camber=self.maximum_camber,
            camber_position=self.maximum_camber_position,
            thickness_to_chord=self.thickness_to_chord,
            airfoil_name="root_airfoil",
            chord=self.c_root,
            thickness_factor=self.t_factor_root,
            mesh_deflection=self.mesh_deflection,
            position=self._root_position,
        )

    @Part
    def tip_airfoil(self):
        return Airfoil(
            maximum_camber=self.maximum_camber,
            camber_position=self.maximum_camber_position,
            thickness_to_chord=self.thickness_to_chord,
            airfoil_name="tip_airfoil",
            chord=self.c_tip,
            thickness_factor=self.t_factor_tip,
            mesh_deflection=self.mesh_deflection,
            position=self._tip_position,
        )

    @Part
    def tip_airfoil_mirrored(self):
        return Airfoil(
            maximum_camber=self.maximum_camber,
            camber_position=self.maximum_camber_position,
            thickness_to_chord=self.thickness_to_chord,
            airfoil_name="tip_airfoil_mirrored",
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
            mesh_deflection=self.mesh_deflection,
            airfoil_root=self.root_airfoil,
            airfoil_tip=self.tip_airfoil,
        )

    @Part
    def wingbox_mirrored(self):
        return Wingbox(
            c_root=self.c_root,
            c_tip=self.c_tip,
            semi_span=-self.semi_span,
            sweep_le=-self.sweep_le,
            dihedral=-self.dihedral,
            twist=self.twist,
            front_spar_position=self.front_spar_position,
            rear_spar_position=self.rear_spar_position,
            mesh_deflection=self.mesh_deflection,
            airfoil_root=self.root_airfoil,
            airfoil_tip=self.tip_airfoil_mirrored,
        )


# ---------------------------------------------------------------------- #
# TEST
# ---------------------------------------------------------------------- #

if __name__ == "__main__":
    from parapy.gui import display

    ls = LiftingSurface(
        # Required: geometry cannot be derived without these two
        wing_area=15.0,
        semi_span=5.0,
        # Everything below now has Roskam-based defaults and can be omitted
        fuselage_length=10.0,
        fuselage_radius=0.3,
        mesh_deflection=1e-4,
        label="test_liftingsurface",
    )
    display(ls)