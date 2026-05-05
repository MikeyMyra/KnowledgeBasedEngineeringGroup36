from math import radians, tan, pi, sqrt

from parapy.core import Input, Attribute, Part
from parapy.geom import GeomBase, translate, rotate, Vector

from Pythonfiles.Components.Engines.JetEngine import JetEngine
from Pythonfiles.Components.Engines.PropellerEngine import PropellerEngine


class Engine(GeomBase):
    """
    Engine selector with Roskam-driven type, count, and positioning.

    Decision rules (Roskam Vol. I §3.2, §3.6):
    ─────────────────────────────────────────────────────────────────────
    Engine type:    cruise_speed < 130 m/s (≈ Mach 0.40) → propeller
                    cruise_speed ≥ 130 m/s                → jet

    Engine count:   MTOW < 2700 kg   → 1 engine
                    2700 ≤ MTOW < 30000 kg → 2 engines
                    ≥ 30000 kg       → 4 engines  (outside UAV scope, safety net)

    Blade count:    Roskam Vol. I §3.6 disk solidity σ = n·c/(π·R)
                    Derived inside PropellerEngine; here we add a
                    cruise-speed nudge: high-speed props (>80 m/s) favour
                    more blades (lower tip speed per blade) → solidity bumped.

    Placement:
        n_engines == 1, jet      → dorsal rear mount (top of fuselage, aft)
        n_engines == 1, prop     → centreline nose tractor
        n_engines == 2, any      → wing-mounted (symmetric, ±Y)
    ─────────────────────────────────────────────────────────────────────
    """

    # ------------------------------------------------------------------ #
    # MISSION / SIZING INPUTS
    # ------------------------------------------------------------------ #

    cruise_speed: float = Input()       # cruise TAS [m/s]

    mtow: float = Input()
    thrust_to_weight: float = Input()

    rho: float = Input()
    g: float = Input()

    # ------------------------------------------------------------------ #
    # GEOMETRY CONTEXT  (supplied by Aircraft)
    # ------------------------------------------------------------------ #

    semi_span: float = Input()
    sweep_le: float = Input()
    dihedral: float = Input()

    fuselage_radius: float = Input()
    fuselage_length: float = Input()    # needed for dorsal-rear placement

    wing_root_x: float = Input()        # X of wing root LE in fuselage frame [m]
    wing_root_z: float = Input()        # Z of wing root LE in fuselage frame [m]

    # ------------------------------------------------------------------ #
    # ROSKAM / DESIGN PARAMETERS
    # ------------------------------------------------------------------ #

    disk_loading_uav: float = Input()
    target_solidity: float = Input()

    # ------------------------------------------------------------------ #
    # GEOMETRY OVERRIDES
    # ------------------------------------------------------------------ #

    nacelle_length_override: float = Input()
    nacelle_radius_override: float = Input()

    n_blades_override: int = Input()
    blade_length_override: float = Input()
    blade_root_chord_override: float = Input()
    blade_sweep: float = Input()

    # ------------------------------------------------------------------ #
    # DISPLAY CONTROL
    # ------------------------------------------------------------------ #

    taper_sections: int = Input(8)
    color_nacelle: str = Input("Silver")
    color_spinner: str = Input("White")
    color_blade: str = Input("DarkGray")
    color_fan: str = Input("DarkGray")
    color_nozzle: str = Input("Gray")
    mesh_deflection: float = Input(1e-4)

    inlet_radius_ratio: float = Input(0.85)
    nozzle_radius_ratio: float = Input(0.70)

    # ------------------------------------------------------------------ #
    # ROSKAM DECISION RULES
    # ------------------------------------------------------------------ #

    @Attribute
    def engine_type(self) -> str:
        """Roskam Vol. I §3.2 / §3.6: prop below Mach 0.40, jet above."""
        return "propeller" if self.cruise_speed < 130.0 else "jet"

    @Attribute
    def n_engines(self) -> int:
        """Roskam Vol. I §3.2 Table 3.1: engine count from MTOW bands."""
        if self.mtow < 2700:
            return 1
        elif self.mtow < 30000:
            return 2
        return 4   # outside UAV scope; safety net only

    @Attribute
    def target_solidity_effective(self) -> float:
        """Roskam Vol. I §3.6: higher cruise speed → more blades to limit tip speed.
        Above 80 m/s cruise, bump solidity to nudge blade count upward."""
        if self.cruise_speed > 80.0:
            return min(self.target_solidity * 1.4, 0.30)
        return self.target_solidity

    # ------------------------------------------------------------------ #
    # POSITIONING HELPERS
    # ------------------------------------------------------------------ #

    @Attribute
    def _wing_mount_y(self) -> float:
        """Spanwise station for wing-mounted engines: 35% semi-span (Roskam)."""
        return self.semi_span * 0.35

    @Attribute
    def _wing_mount_x(self) -> float:
        """X at the wing-mount station, following LE sweep."""
        return self.wing_root_x + self._wing_mount_y * tan(radians(self.sweep_le))

    @Attribute
    def _wing_mount_z(self) -> float:
        """Z at the wing-mount station, following dihedral."""
        return self.wing_root_z + self._wing_mount_y * tan(radians(self.dihedral))

    @Attribute
    def _dorsal_x(self) -> float:
        """X of dorsal-rear jet: 85% of fuselage length."""
        return self.fuselage_length * 0.85

    @Attribute
    def _dorsal_z(self) -> float:
        """Z of dorsal-rear jet: on top of fuselage."""
        return self.fuselage_radius

    @Attribute
    def _position_starboard(self):
        """Position for starboard (or single centreline/dorsal) engine.
        No rotation here — subclasses apply rotate90('y') via _engine_base."""
        if self.n_engines == 1:
            if self.engine_type == "jet":
                return translate(
                    self.position,
                    Vector(1, 0, 0), self._dorsal_x,
                    Vector(0, 0, 1), self._dorsal_z,
                )
            else:
                # Nose tractor: origin of fuselage frame
                return self.position

        # n_engines == 2: wing-mounted starboard
        return translate(
            self.position,
            Vector(1, 0, 0), self._wing_mount_x,
            Vector(0, 1, 0), self._wing_mount_y,
            Vector(0, 0, 1), self._wing_mount_z,
        )

    @Attribute
    def _position_port(self):
        """Port (mirrored) position for twin installations.
        No rotation here — subclasses apply rotate90('y') via _engine_base."""
        return translate(
            self.position,
            Vector(1, 0, 0), self._wing_mount_x,
            Vector(0, 1, 0), - self._wing_mount_y,
            Vector(0, 0, 1), self._wing_mount_z,
        )

    # ------------------------------------------------------------------ #
    # SHARED KWARG BUILDERS
    # ------------------------------------------------------------------ #

    def _jet_kwargs(self, position) -> dict:
        return dict(
            position=position,
            mtow=self.mtow,
            n_engines=self.n_engines,
            thrust_to_weight=self.thrust_to_weight,
            rho=self.rho,
            g=self.g,
            semi_span=self.semi_span,
            sweep_le=self.sweep_le,
            dihedral=self.dihedral,
            fuselage_radius=self.fuselage_radius,
            attach_spanwise_pct=0.0,
            attach_x_offset=0.0,
            attach_z_offset=0.0,
            nacelle_length_override=self.nacelle_length_override,
            nacelle_radius_override=self.nacelle_radius_override,
            taper_sections=self.taper_sections,
            color_nacelle=self.color_nacelle,
            color_fan=self.color_fan,
            color_nozzle=self.color_nozzle,
            inlet_radius_ratio=self.inlet_radius_ratio,
            nozzle_radius_ratio=self.nozzle_radius_ratio,
            mesh_deflection=self.mesh_deflection,
        )

    def _prop_kwargs(self, position) -> dict:
        return dict(
            position=position,
            mtow=self.mtow,
            n_engines=self.n_engines,
            thrust_to_weight=self.thrust_to_weight,
            rho=self.rho,
            g=self.g,
            semi_span=self.semi_span,
            sweep_le=self.sweep_le,
            dihedral=self.dihedral,
            fuselage_radius=self.fuselage_radius,
            attach_spanwise_pct=0.0,
            attach_x_offset=0.0,
            attach_z_offset=0.0,
            disk_loading_uav=self.disk_loading_uav,
            target_solidity=self.target_solidity_effective,
            nacelle_length_override=self.nacelle_length_override,
            nacelle_radius_override=self.nacelle_radius_override,
            n_blades_override=None,
            blade_length_override=self.blade_length_override,
            blade_root_chord_override=self.blade_root_chord_override,
            blade_sweep=self.blade_sweep,
            taper_sections=self.taper_sections,
            color_nacelle=self.color_nacelle,
            color_spinner=self.color_spinner,
            color_blade=self.color_blade,
            inlet_radius_ratio=self.inlet_radius_ratio,
            nozzle_radius_ratio=self.nozzle_radius_ratio,
            mesh_deflection=self.mesh_deflection,
        )

    # ------------------------------------------------------------------ #
    # PARTS  —  four slots, suppress unused ones
    # ------------------------------------------------------------------ #

    @Part
    def jet_starboard(self):
        return JetEngine(
            label="jet_starboard",
            suppress=self.engine_type != "jet",
            **self._jet_kwargs(self._position_starboard),
        )

    @Part
    def jet_port(self):
        return JetEngine(
            label="jet_port",
            suppress=self.engine_type != "jet" or self.n_engines < 2,
            **self._jet_kwargs(self._position_port),
        )

    @Part
    def prop_starboard(self):
        return PropellerEngine(
            label="prop_starboard",
            suppress=self.engine_type != "propeller",
            **self._prop_kwargs(self._position_starboard),
        )

    @Part
    def prop_port(self):
        return PropellerEngine(
            label="prop_port",
            suppress=self.engine_type != "propeller" or self.n_engines < 2,
            **self._prop_kwargs(self._position_port),
        )


# ---------------------------------------------------------------------- #
# TEST
# ---------------------------------------------------------------------- #

if __name__ == "__main__":
    from parapy.gui import display

    # ------------------------------------------------------------------ #
    # CASE 1 — slow, light UAV → single nose tractor prop
    # MTOW 80 kg  (<2700)  → n_engines = 1
    # cruise 25 m/s        → engine_type = "propeller"
    # ------------------------------------------------------------------ #

    single_prop = Engine(
        label="single_prop_uav",

        cruise_speed=25.0,
        mtow=80.0,
        thrust_to_weight=0.35,
        rho=1.225,
        g=9.81,

        semi_span=4.0,
        sweep_le=3.0,
        dihedral=3.0,
        fuselage_radius=0.18,
        fuselage_length=2.5,

        wing_root_x=1.0,
        wing_root_z=-0.18,

        disk_loading_uav=80.0,
        target_solidity=0.15,

        nacelle_length_override=None,
        nacelle_radius_override=None,
        n_blades_override=None,
        blade_length_override=None,
        blade_root_chord_override=None,
        blade_sweep=5.0,

        taper_sections=8,
        color_nacelle="Silver",
    )

    # ------------------------------------------------------------------ #
    # CASE 2 — medium-speed UAV → twin wing-mounted props
    # MTOW 3000 kg (≥2700) → n_engines = 2
    # cruise 90 m/s        → engine_type = "propeller"
    # ------------------------------------------------------------------ #

    twin_prop = Engine(
        label="twin_prop_uav",

        cruise_speed=90.0,
        mtow=3000.0,
        thrust_to_weight=0.30,
        rho=1.225,
        g=9.81,

        semi_span=9.0,
        sweep_le=5.0,
        dihedral=4.0,
        fuselage_radius=0.50,
        fuselage_length=8.0,

        wing_root_x=3.2,
        wing_root_z=-0.50,

        disk_loading_uav=80.0,
        target_solidity=0.15,

        nacelle_length_override=None,
        nacelle_radius_override=None,
        n_blades_override=None,
        blade_length_override=None,
        blade_root_chord_override=None,
        blade_sweep=5.0,
        
        inlet_radius_ratio=0.85,
        nozzle_radius_ratio=0.7,

        taper_sections=8,
        color_nacelle="Silver",
    )

    # ------------------------------------------------------------------ #
    # CASE 3 — high-speed UAV → single dorsal-rear jet
    # MTOW 800 kg  (<2700) → n_engines = 1
    # cruise 200 m/s       → engine_type = "jet"
    # ------------------------------------------------------------------ #

    single_jet = Engine(
        label="single_jet_uav",

        cruise_speed=200.0,
        mtow=800.0,
        thrust_to_weight=0.45,
        rho=1.225,
        g=9.81,

        semi_span=4.5,
        sweep_le=25.0,
        dihedral=2.0,
        fuselage_radius=0.40,
        fuselage_length=6.0,

        wing_root_x=2.5,
        wing_root_z=-0.40,

        disk_loading_uav=80.0,
        target_solidity=0.15,

        nacelle_length_override=None,
        nacelle_radius_override=None,
        n_blades_override=None,
        blade_length_override=None,
        blade_root_chord_override=None,
        blade_sweep=0.0,
        
        inlet_radius_ratio=0.85,
        nozzle_radius_ratio=0.7,

        taper_sections=10,
        color_nacelle="Silver",
    )

    # ------------------------------------------------------------------ #
    # CASE 4 — large high-speed UAV → twin wing-mounted jets
    # MTOW 5000 kg (≥2700) → n_engines = 2
    # cruise 220 m/s       → engine_type = "jet"
    # ------------------------------------------------------------------ #

    twin_jet = Engine(
        label="twin_jet_uav",

        cruise_speed=220.0,
        mtow=5000.0,
        thrust_to_weight=0.40,
        rho=1.225,
        g=9.81,

        semi_span=8.0,
        sweep_le=20.0,
        dihedral=3.0,
        fuselage_radius=0.70,
        fuselage_length=10.0,

        wing_root_x=4.0,
        wing_root_z=-0.70,

        disk_loading_uav=80.0,
        target_solidity=0.15,

        nacelle_length_override=None,
        nacelle_radius_override=None,
        n_blades_override=None,
        blade_length_override=None,
        blade_root_chord_override=None,
        blade_sweep=0.0,
        
        inlet_radius_ratio=0.85,
        nozzle_radius_ratio=0.7,

        taper_sections=10,
        color_nacelle="Silver",
    )

    display([single_prop, twin_prop, single_jet, twin_jet])