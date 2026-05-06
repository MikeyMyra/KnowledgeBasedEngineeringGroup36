import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from parapy.core import Input, Attribute, Part
from parapy.geom import GeomBase

from Pythonfiles.Components.Aircraft import Aircraft

# ============================================================ #
# CALCULATIONS
# ============================================================ #

def engine_type(self) -> str:
    """Roskam Vol. I §3.2 / §3.6: prop below Mach 0.40, jet above."""
    return "propeller" if self.cruise_speed < 130.0 else "jet"
# ADD ALTITUDE CALCULATION FOR MACH 

# ADD PAYLOAD WEIGHT CALCULATION (Mike)

# ADD MTOW ESTIMATION CALL MISSION.PY

# ADD WING AREA ESTIMATION CALL MISSION.PY

# ADD L/D CALCULATION CALL MISSION.PY 

# ADD AIRFOIL FITTING ITERATION WITH Q3D (Mike)

# CALL AIRCRAFT CLASS FOR GEOMETRY VISUALISATION