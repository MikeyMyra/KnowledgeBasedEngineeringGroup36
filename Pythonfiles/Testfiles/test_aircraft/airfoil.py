# !/usr/bin/env python
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

from parapy.geom import FittedCurve, Point
from parapy.core import Attribute, Part, Input
from test_aircraft import Frame

import os # for path operations (opening airfoil definition files)


class Airfoil(FittedCurve):  # note the use of FittedCurve as superclass

    chord: float = Input(1.)
    airfoil_name: str = Input("whitcomb")
    thickness_factor: float = Input(1.)
    mesh_deflection: float = Input(1e-4)

    # airfoil files are located in 'test_aircraft' subdirectory, but this can be changed
    airfoil_dir = Input(os.path.abspath('Pythonfiles/Testfiles/test_aircraft'))
    # using abspath to create an absolute path makes this robust against
    # changes in working directory

    @Attribute
    def points(self) -> [Point]:  # required input to the FittedCurve superclass
        """List of points defining the airfoil shape, read from a file"""
        with open(os.path.join(self.airfoil_dir, self.airfoil_name + ".dat"), 'r') as f:
            point_lst = []
            for line in f:
                x, z = line.split(' ', 1)  # the cartesian coordinates are directly interpreted as X and Z coordinates
                point_lst.append(self.position.translate(
                    "x", float(x) * self.chord,  # the x points are scaled according to the airfoil chord length
                    "z", float(z) * self.chord * self.thickness_factor)) # the y points are scaled according to the /
                                                                         # thickness factor
        return point_lst

    @Part
    def frame(self):  # to visualize the given airfoil reference frame
        return Frame(pos=self.position,
                     hidden=False)


if __name__ == '__main__':
    from parapy.gui import display

    obj = Airfoil(label="airfoil")
    display(obj)