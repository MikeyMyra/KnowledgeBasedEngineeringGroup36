"""
main.py
=======
Entry point for the UAV KBE tool.

All sizing logic, payload rules, mission analysis and geometry are in Drone.py.
This file only configures the drone instance and launches the ParaPy GUI.

Required inputs
---------------
cruise_speed      [m/s]   true airspeed at cruise (10–350 m/s)
mission_altitude  [m]     cruise altitude above MSL (0–20 000 m)
mission_range     [km]    one-way range (1–25 000 km)
mission_endurance [hr]    loiter endurance (0.1–120 hr)

Everything else has a Roskam/Raymer default and can be overridden below.
"""

import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from Pythonfiles.Drone import Drone
from parapy.gui import display


if __name__ == "__main__":

    d = Drone(
        # ------------------------------------------------------------------ #
        # MISSION  — required
        # ------------------------------------------------------------------ #
        cruise_speed=200,          # [m/s]
        mission_altitude=2000,      # [m]
        mission_range=500,          # [km]
        mission_endurance=5,        # [hr]

        # ------------------------------------------------------------------ #
        # PAYLOAD INTENT  — optional overrides
        # ------------------------------------------------------------------ #
        payload_role="Strike",         # ISR / Strike / SEAD / Mapping / COMMS relay / Patrol
        weapon_count=1,             # 0 = unarmed; max 6

        # ------------------------------------------------------------------ #
        # OPTIONAL OVERRIDES  (uncomment to change from Roskam/Raymer default)
        # ------------------------------------------------------------------ #

        # --- Mission tweaks ---
        # specific_fuel=0.5,          # [1/hr]  SFC (piston default)
        # prop_efficiency=0.8,        # propeller efficiency [-]
        # oswald_factor=0.8,          # Oswald efficiency factor [-]
        # reserve_time=0.5,           # [hr] fuel reserve

        # --- Fuel tank ---
        # fuel_type="auto",           # "auto" | "avgas_100ll" | "jet_a" | "jp8" | "lipo_battery"
        # fuel_tank_aspect_ratio=3.0, # tank length-to-diameter ratio (1.1–10)

        # --- Wing geometry ---
        # wing_taper_ratio=0.40,
        # wing_sweep_le=5.0,          # [deg]
        # wing_dihedral=5.0,          # [deg]
        # wing_twist=0.0,             # [deg]
        # wing_thickness_to_chord=0.15,
        # wing_maximum_camber=0.04,
        # wing_maximum_camber_position=0.40,

        # --- Tail geometry ---
        # tail_taper_ratio=0.40,
        # tail_sweep_le=10.0,         # [deg]
        # tail_volume_coefficient_h=0.60,
        # tail_volume_coefficient_v=0.04,
        # tail_aspect_ratio_h=4.5,
        # tail_aspect_ratio_v=1.8,

        # --- Fuselage ---
        # fuselage_cylinder_start=10.0,   # [% of fuselage length]
        # fuselage_cylinder_end=70.0,     # [% of fuselage length]
        # undercarriage_retractible=False,

        # --- Propeller / engine ---
        # thrust_to_weight=0.35,
        # disk_loading_uav=80.0,
        # target_solidity=0.15,
        # blade_sweep=5.0,            # [deg]

        # --- Colors ---
        # fuselage_cones_color="steelblue",
        # fuselage_cylinder_color="blue",
        # main_wing_color_liftingsurface="yellow",
        # engine_color_nacelle="silver",
    )

    display(d)
