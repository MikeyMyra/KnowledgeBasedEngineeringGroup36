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

from typing import List

from parapy.geom import LoftedSolid, LoftedSurface, Circle, Vector, translate
from parapy.core import Input, Attribute, Part, child



class Fuselage(LoftedSolid):
    """Fuselage geometry, a loft through circles.

    Note the use of LoftedSolid as superclass. It means that every
    Fuselage instance defines a lofted geometry. A required input for LoftedSolid
    is a list of profiles, so either an @Attribute or a @Part sequence
    called "profiles" must be present in the body of the class. Use 'Display
    node' in the root node of the object tree to visualise the (yellow) loft
    in the GUI graphical viewer

    Examples:
        >>> obj = Fuselage(pass_down="fu_radius, fu_sections, fu_length",
        ...                    color="Green",
        ...                    mesh_deflection=0.0001
        ...                    )
    """

    #: fuselage radius
    #: :type: float
    radius: float = Input()

    #: fuselage sections (percentage of nominal radius, at evenly-spaced stations)
    #: :type: collections.Sequence[float]
    sections: list[float] = Input([10, 90, 100, 100, 100, 100, 100, 100, 95, 70, 5])
    #: fuselage length (m)
    #: :type: float
    length: float = Input()

    # Accuracy for graphical representation
    mesh_deflection: float = Input(1e-4)

    @Attribute
    def section_radius(self) -> List[float]:
        """Section radius multiplied by the radius distribution
        through the length. Note that the numbers are percentages.

        Returns:
            List[float]: section radii in percentage along fuselage length
        """
        return [i * self.radius / 100. for i in self.sections]

    @Attribute
    def section_length(self) -> float:
        """The section length is determined by dividing the fuselage
        length by the number of fuselage sections.

        Returns:
            float: length of each fuselage section
        """
        return self.length / (len(self.sections) - 1)

    # Required slot of the superclass LoftedSolid.
    # Originally, this is an Input slot, but any slot type is fine as long as it contains
    # an iterable of the profiles for the loft.
    @Part
    def profiles(self):
        return Circle(quantify=len(self.sections), color="Black",
                      radius=self.section_radius[child.index],
                      # fuselage along the X axis, nose in XOY
                      position=translate(self.position.rotate90('y'),  # circles are in XY plane, thus need rotation
                                         Vector(1, 0, 0),
                                         child.index * self.section_length))


if __name__ == '__main__':
    from parapy.gui import display

    obj = Fuselage(label="fuselage", mesh_deflection=0.0001)
    display(obj)
