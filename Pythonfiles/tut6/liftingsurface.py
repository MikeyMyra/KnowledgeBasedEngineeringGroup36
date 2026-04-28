#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright (C) 2016 ParaPy Holding B.V.
#
# This file is subject to the terms and conditions defined in
# the license agreement that you have received with this source code
#
# THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
# KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR
# PURPOSE.
from math import radians, tan

import numpy as np
from parapy.geom import LoftedSolid, translate, rotate
from parapy.core import Input, Attribute, Part

from tut6 import Airfoil
from tut6 import Frame

# Matlab library, in order to use e.g. `matlab.double()`
import matlab
# Imports the Matlab engine instance created in the module's `__init__.py`
from tut6 import MATLAB_Q3D_ENGINE


class LiftingSurface(LoftedSolid):  # note use of loftedSolid as superclass
    airfoil_root_name: str = Input("whitcomb")
    airfoil_tip_name: str = Input("simm_airfoil")

    c_root: float = Input()
    c_tip: float = Input()
    t_factor_root: float = Input(1.)
    t_factor_tip: float = Input(1.)

    semi_span: float = Input()
    sweep: float = Input(0)
    twist: float = Input(0)
    dihedral: float = Input(0)

    mesh_deflection: float = Input(1e-4)

    # required slot for the superclass LoftedSolid
    # (usually an @Input, but we're turning it into an @Attribute)
    @Attribute
    def profiles(self):
        return [self.root_airfoil, self.tip_airfoil,]

    @Attribute
    def q3d_data(self):
        """All inputs and results from running Q3D (MATLAB)"""
        #! Note: The file `runq3d.m` hard-codes the airfoil shapes a,d a lot of other things
        #! To make this fully operational, you'd need to update the Matlab code such that it
        #! accepts all relevant information (airfoil shape, flight speed, M, Re…) as input
        return MATLAB_Q3D_ENGINE.run_q3d(matlab.double([[0, 0, 0, self.c_root, 0],
                                                        [self.semi_span*np.cos(self.dihedral),
                                                         self.semi_span,
                                                         self.semi_span*np.cos(self.dihedral),
                                                         self.c_tip, self.twist]
                                                       ]),
                                                       # in MATLAB 2021, double values are defined as
                                                       # rectangular nested sequence
                                         matlab.double([1]),
                                         nargout=2 # specify number of outputs if >1
                                        )

    @Attribute
    def q3d_res(self) -> dict:
        """q3d results"""
        return self.q3d_data[0]

    @Attribute
    def q3d_ac(self) -> dict:
        """q3d inputs"""
        return self.q3d_data[1]

    @Attribute
    def wing_cl(self) -> float:
        return self.q3d_res["CLwing"]

    @Attribute
    def wing_cd(self) -> float:
        return self.q3d_res["CDwing"]

    @Part
    def frame(self):
        """to visualize the given lifting surface reference frame"""
        return Frame(pos=self.position,
                     hidden=False)

    @Part
    def root_airfoil(self):  # root airfoil will receive self.position as default
        return Airfoil(airfoil_name=self.airfoil_root_name,
                       chord=self.c_root,
                       thickness_factor=self.t_factor_root,
                       mesh_deflection=self.mesh_deflection)

    @Part
    def tip_airfoil(self):
        return Airfoil(airfoil_name=self.airfoil_tip_name,
                       chord=self.c_tip,
                       thickness_factor=self.t_factor_tip,
                       position=translate(
                           rotate(self.position, "y", radians(self.twist)),  # apply twist angle
                           "y", self.semi_span,
                           "x", self.semi_span * tan(radians(self.sweep))),  # apply sweep
                       mesh_deflection=self.mesh_deflection)


if __name__ == '__main__':
    from parapy.gui import display
    ls = LiftingSurface(label="lifting surface",
                        c_root=5,
                        c_tip=2.5,
                        semi_span=27,
                        mesh_deflection=1e-4)
    display(ls)
