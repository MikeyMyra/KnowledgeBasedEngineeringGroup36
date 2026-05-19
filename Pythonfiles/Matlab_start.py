"""
MATLAB engine initialisation for Q3D aerodynamic analysis.

Starts a shared MATLAB engine session at import time and changes the working
directory to the Q3D folder so that vortex-lattice scripts can be called
directly from Python via MATLAB_Q3D_ENGINE throughout the application.
"""

import os
import matlab.engine

MATLAB_Q3D_ENGINE = matlab.engine.start_matlab()

# Get the directory of this file, then navigate to Q3D relative to it
base_dir = os.path.dirname(os.path.abspath(__file__))
q3d_path = os.path.join(base_dir, '..', 'Q3D')
MATLAB_Q3D_ENGINE.cd(q3d_path)