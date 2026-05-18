from Pythonfiles.Drone import Drone
from parapy.gui import display


if __name__ == "__main__":

    d = Drone(
        
        # ------------------------------------------------------------------ #
        # MISSION  — required
        # ------------------------------------------------------------------ #
        cruise_speed=50,            # [m/s]
        mission_altitude=1000,      # [m]
        mission_range=100,          # [km]
        mission_endurance=1,        # [hr]

        # ------------------------------------------------------------------ #
        # PAYLOAD INTENT  — required
        # ------------------------------------------------------------------ #
        payload_role="Strike",      # ISR / Strike / SEAD / Mapping / COMMS relay / Patrol
        weapon_count=1,             # 0 = unarmed; max 6
    )

    display(d)
