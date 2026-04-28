import os
import matlab.engine

MATLAB_Q3D_ENGINE = matlab.engine.start_matlab()

# Get the directory of this file, then navigate to Q3D relative to it
base_dir = os.path.dirname(os.path.abspath(__file__))
q3d_path = os.path.join(base_dir, 'Matlabfiles', 'Q3D')
MATLAB_Q3D_ENGINE.cd(q3d_path)


from .ref_frame import Frame
from .fuselage import Fuselage
from .airfoil import Airfoil
from .liftingsurface import LiftingSurface
from .aircraft import Aircraft