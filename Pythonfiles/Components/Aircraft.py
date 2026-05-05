import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from parapy.core import Input, Attribute, Part
from parapy.geom import GeomBase

from Pythonfiles.Components.Liftingsurfaces.Liftingsurface import LiftingSurface
from Pythonfiles.Components.Fuselage.Fuselage import Fuselage
from Pythonfiles.Components.Engines.Engine import Engine


class Aircraft(GeomBase):

    # ============================================================ #
    # GLOBAL
    # ============================================================ #

    cruise_speed: float = Input()
    aircraft_mass: float = Input()
    mesh_deflection: float = Input(1e-4)
    g: float = Input(9.81)

    # ============================================================ #
    # FUSELAGE
    # ============================================================ #
    fuselage_cylinder_start: float = Input()
    fuselage_cylinder_end: float = Input()

    undercarriage_retractible: bool = Input()

    fuselage_cones_color = Input()
    fuselage_cylinder_color = Input()

    undercarriage_color_tyre: str = Input()
    undercarriage_color_axle: str = Input()
    undercarriage_color_strut: str = Input()

    # ============================================================ #
    # MAIN WING
    # ============================================================ #

    wing_area: float = Input()
    wing_semi_span: float = Input()

    wing_taper_ratio: float = Input()
    wing_sweep_le: float = Input()
    wing_twist: float = Input()
    wing_dihedral: float = Input()

    wing_thickness_to_chord: float = Input()
    wing_maximum_camber: float = Input()
    wing_maximum_camber_position: float = Input()

    wing_t_factor_root: float = Input()
    wing_t_factor_tip: float = Input()

    wing_front_spar_position: float = Input()
    wing_rear_spar_position: float = Input()

    main_wing_color_wingbox: str = Input()
    main_wing_color_liftingsurface: str = Input()

    # ============================================================ #
    # TAIL INPUTS
    # ============================================================ #

    tail_area: float = Input()
    tail_semi_span: float = Input()

    tail_taper_ratio: float = Input()
    tail_sweep_le: float = Input()
    tail_twist: float = Input()
    tail_dihedral: float = Input()

    tail_thickness_to_chord: float = Input()
    tail_maximum_camber: float = Input()
    tail_maximum_camber_position: float = Input()

    tail_t_factor_root: float = Input()
    tail_t_factor_tip: float = Input()

    tail_front_spar_position: float = Input()
    tail_rear_spar_position: float = Input()

    tail_volume_coefficient_h: float = Input()
    tail_volume_coefficient_v: float = Input()
    tail_aspect_ratio_h: float = Input()
    tail_aspect_ratio_v: float = Input()

    tail_h_color_wingbox: str = Input()
    tail_h_color_liftingsurface: str = Input()
    tail_v_color_wingbox: str = Input()
    tail_v_color_liftingsurface: str = Input()
    
    tail_h_color_wingbox: str = Input()
    tail_h_color_liftingsurface: str = Input()
    tail_v_color_wingbox: str = Input()
    tail_v_color_liftingsurface: str = Input()

    # ============================================================ #
    # ENGINE (CLEANED INTERFACE)
    # ============================================================ #

    thrust_to_weight: float = Input()

    rho: float = Input()

    disk_loading_uav: float = Input()
    target_solidity: float = Input()

    nacelle_length_override: float = Input(None)
    nacelle_radius_override: float = Input(None)

    n_blades_override: int = Input(None)
    blade_length_override: float = Input(None)
    blade_root_chord_override: float = Input(None)

    blade_sweep: float = Input()
    
    inlet_radius_ratio: float = Input()
    nozzle_radius_ratio: float = Input()

    engine_color_nacelle: str = Input("Silver")

    # ============================================================ #
    # PARTS
    # ============================================================ #

    @Part
    def fuselage(self):
        return Fuselage(
            aircraft_mass=self.aircraft_mass,
            cylinder_start=self.fuselage_cylinder_start,
            cylinder_end=self.fuselage_cylinder_end,
            color_taper=self.fuselage_cones_color,
            color_cylinder=self.fuselage_cylinder_color,
            undercarriage_retractible=self.undercarriage_retractible,
            undercarriage_color_tyre=self.undercarriage_color_tyre,
            undercarriage_color_axle=self.undercarriage_color_axle,
            undercarriage_color_strut=self.undercarriage_color_strut,
        )

    @Part
    def main_wing(self):
        return LiftingSurface(
            label="main_wing",
            wing_area=self.wing_area,
            semi_span=self.wing_semi_span,

            fuselage_length=self.fuselage.length,
            fuselage_radius=self.fuselage.radius,

            is_tail=False,
            is_vertical_tail=False,

            taper_ratio=self.wing_taper_ratio,
            sweep_le=self.wing_sweep_le,
            twist=self.wing_twist,
            dihedral=self.wing_dihedral,

            thickness_to_chord=self.wing_thickness_to_chord,
            maximum_camber=self.wing_maximum_camber,
            maximum_camber_position=self.wing_maximum_camber_position,

            t_factor_root=self.wing_t_factor_root,
            t_factor_tip=self.wing_t_factor_tip,

            front_spar_position=self.wing_front_spar_position,
            rear_spar_position=self.wing_rear_spar_position,

            wing_ref=None,

            color_wingbox=self.main_wing_color_wingbox,
            color_liftingsurface=self.main_wing_color_liftingsurface,
        )

    @Part
    def horizontal_tail(self):
        return LiftingSurface(
            label="horizontal_tail",
            wing_ref=self.main_wing,
            is_tail=True,
            is_vertical_tail=False,

            fuselage_length=self.fuselage.length,
            fuselage_radius=self.fuselage.radius,

            wing_area=self.tail_area or 1.0,
            semi_span=self.tail_semi_span or 1.0,

            taper_ratio=self.tail_taper_ratio,
            sweep_le=self.tail_sweep_le,
            twist=self.tail_twist,
            dihedral=self.tail_dihedral,

            thickness_to_chord=self.tail_thickness_to_chord,
            maximum_camber=self.tail_maximum_camber,
            maximum_camber_position=self.tail_maximum_camber_position,

            t_factor_root=self.tail_t_factor_root,
            t_factor_tip=self.tail_t_factor_tip,

            front_spar_position=self.tail_front_spar_position,
            rear_spar_position=self.tail_rear_spar_position,

            tail_volume_coefficient_h=self.tail_volume_coefficient_h,
            tail_volume_coefficient_v=self.tail_volume_coefficient_v,
            tail_aspect_ratio_h=self.tail_aspect_ratio_h,
            tail_aspect_ratio_v=self.tail_aspect_ratio_v,
            
            color_wingbox=self.tail_h_color_wingbox,
            color_liftingsurface=self.tail_h_color_liftingsurface,
        )

    @Part
    def vertical_tail(self):
        return LiftingSurface(
            label="vertical_tail",
            wing_ref=self.main_wing,
            is_tail=True,
            is_vertical_tail=True,

            fuselage_length=self.fuselage.length,
            fuselage_radius=self.fuselage.radius,

            wing_area=self.tail_area or 1.0,
            semi_span=self.tail_semi_span or 1.0,

            taper_ratio=self.tail_taper_ratio,

            sweep_le=35.0,
            twist=0.0,
            dihedral=0.0,

            thickness_to_chord=self.tail_thickness_to_chord,
            maximum_camber=0.0,
            maximum_camber_position=self.tail_maximum_camber_position,

            t_factor_root=self.tail_t_factor_root,
            t_factor_tip=self.tail_t_factor_tip,

            front_spar_position=self.tail_front_spar_position,
            rear_spar_position=self.tail_rear_spar_position,

            tail_volume_coefficient_h=self.tail_volume_coefficient_h,
            tail_volume_coefficient_v=self.tail_volume_coefficient_v,
            tail_aspect_ratio_h=self.tail_aspect_ratio_h,
            tail_aspect_ratio_v=self.tail_aspect_ratio_v,
            
            color_wingbox=self.tail_v_color_wingbox,
            color_liftingsurface=self.tail_v_color_liftingsurface,
        )

    @Part
    def engines(self):
        return Engine(

            cruise_speed=self.cruise_speed,
            mtow=self.aircraft_mass,
            thrust_to_weight=self.thrust_to_weight,

            rho=self.rho,
            g=self.g,

            semi_span=self.main_wing.semi_span,
            sweep_le=self.main_wing.sweep_le,
            dihedral=self.main_wing.dihedral,

            fuselage_length=self.fuselage.length,
            fuselage_radius=self.fuselage.radius,
            
            wing_root_x=self.main_wing._root_position.x,
            wing_root_z=self.main_wing._root_position.z,

            disk_loading_uav=self.disk_loading_uav,
            target_solidity=self.target_solidity,

            nacelle_length_override=self.nacelle_length_override,
            nacelle_radius_override=self.nacelle_radius_override,
            
            inlet_radius_ratio=self.inlet_radius_ratio,
            nozzle_radius_ratio=self.nozzle_radius_ratio,

            n_blades_override=self.n_blades_override,
            blade_length_override=self.blade_length_override,
            blade_root_chord_override=self.blade_root_chord_override,

            blade_sweep=self.blade_sweep,

            color_nacelle=self.engine_color_nacelle,  
        )


# ---------------------------------------------------------------------- #
# TEST
# ---------------------------------------------------------------------- #

if __name__ == "__main__":
    from parapy.gui import display

    ac = Aircraft(

        # ========================================================= #
        # MISSION
        # ========================================================= #
        
        cruise_speed=220.0,
        aircraft_mass=2000,          # ✔ mission sizing driver (payload + fuel + structure assumption)

        wing_area=20.0,              # ✔ driven by wing loading requirement (W/S)
        wing_semi_span=8.0,          # ✔ aspect ratio / airport constraint / mission geometry

        thrust_to_weight=0.35,      # ✔ performance requirement (takeoff/climb requirement)
        
        rho=1.225,                  # ✔ ISA sea level (or mission altitude if refined later)

        # ========================================================= #
        # ROSKAM
        # ========================================================= #
        disk_loading_uav=80.0,      # ✔ Roskam / UAV empirical disk loading range
        target_solidity=0.15,       # ✔ Roskam propeller design rule (~0.1–0.2)

        tail_volume_coefficient_h=0.6,   # ✔ Roskam horizontal tail sizing
        tail_volume_coefficient_v=0.04,  # ✔ Roskam vertical tail sizing

        tail_aspect_ratio_h=4.5,         # ✔ Roskam typical HT range (3–5)
        tail_aspect_ratio_v=1.8,         # ✔ Roskam vertical tail range (1.2–2.5)

        wing_taper_ratio=0.40,           # ✔ typical efficient subsonic wing (0.3–0.5)
        wing_sweep_le=5.0,               # ✔ low-speed aircraft assumption (almost straight wing)
        wing_dihedral=5.0,               # ✔ stability rule-of-thumb
        wing_twist=0.0,
        wing_thickness_to_chord=0.15,    # ✔ subsonic structural/aero compromise
        wing_maximum_camber=0.04,        # ✔ typical cambered airfoil range
        wing_maximum_camber_position=0.4,# ✔ NACA-style default

        tail_taper_ratio=0.40,           # ✔ same logic as wing
        tail_sweep_le=10.0,              # ✔ slightly more swept tail (stability margin)
        tail_thickness_to_chord=0.15,
        tail_maximum_camber_position=0,
        tail_maximum_camber=0,
        tail_dihedral=0,
        tail_twist=0,

        blade_sweep=5.0,                 # ✔ propeller/rotor empirical aero smoothing

        # ========================================================= #
        # USER SET
        # ========================================================= #
        fuselage_cylinder_start=10.0,# ⚙ geometry partitioning (model structure choice)
        fuselage_cylinder_end=70.0,  # ⚙ same

        undercarriage_retractible=False,  # ⚙ design choice (simplification vs realism)

        # ---------------- COLORS (PURE VISUAL ONLY) ----------------
        fuselage_cones_color="steelblue",
        fuselage_cylinder_color="blue",
        undercarriage_color_tyre="black",
        undercarriage_color_axle="white",
        undercarriage_color_strut="silver",

        main_wing_color_wingbox="black",
        main_wing_color_liftingsurface="yellow",

        tail_h_color_wingbox="black",
        tail_h_color_liftingsurface="silver",
        tail_v_color_wingbox="black",
        tail_v_color_liftingsurface="white",

        engine_color_nacelle="Silver",

        # ========================================================= #
        # FIXED
        # ========================================================= #
        wing_front_spar_position=0.15,   # structural convention
        wing_rear_spar_position=0.60,    # structural convention

        tail_front_spar_position=0.15,    # structural convention
        tail_rear_spar_position=0.60,     # structural convention
        
        inlet_radius_ratio=0.85,
        nozzle_radius_ratio=0.7,
        
        g=9.81,                      # ✔ physical constant (always fixed on Earth)
    )

    display(ac)