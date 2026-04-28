# import, initialise MATLAB engine
import matlab.engine
MATLAB_Q3D_ENGINE = matlab.engine.start_matlab()
# The Matlab engine runs in a separate process and has a separate working
# directory. Our Matlab code is in the "Q3D" sub-directory, so if we change
# its working directory to it here, it can find all relevant files directly
MATLAB_Q3D_ENGINE.cd(r'Q3D')


from .ref_frame import Frame
from .fuselage import Fuselage
from .airfoil import Airfoil
from .liftingsurface import LiftingSurface
from .aircraft import Aircraft