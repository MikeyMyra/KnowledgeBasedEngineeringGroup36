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

import os
from math import radians

from parapy.core import (Input, Attribute, Part, child)
from parapy.exchange import STEPWriter
from parapy.geom import (GeomBase, translate, rotate90, rotate, MirroredShape,
                         Rectangle, ProjectedCurve, SubtractedSolid,
                         FusedSolid, Solid)

from tut6 import Frame
from tut6 import Fuselage
from tut6 import LiftingSurface

# This is a global variable *within this module*
maindir = os.path.dirname(__file__)


class Aircraft(GeomBase):
    """Define a simple aircraft model with a fuselage, the main wing,
    horizontal and vertical tail planes"""


    #: radius of the fuselage (m)
    fu_radius: float = Input()

    #: list of fuselage section locations
    fu_sections: list[float] = Input([10, 90, 100, 100, 100, 100, 100, 100, 95, 70, 5])

    #: fuselage length (m)
    fu_length: float = Input()

    #: root and tip airfoil names (requires "<name>.dat" file with airfoil coordinates!)
    airfoil_root_name: str = Input("whitcomb")
    airfoil_tip_name: str = Input("simm_airfoil")

    #:
    w_c_root: float = Input()
    w_c_tip: float = Input()

    #: to reduce/increase the thickness of the airfoil from the .dat file
    t_factor_root: float = Input(1)
    t_factor_tip: float = Input(1)

    #:
    w_semi_span: float = Input()

    #:
    w_sweep: float = Input(0)

    #:
    w_twist: float = Input(0)

    #:
    w_dihedral: float = Input(0)

    #: longitudinal position w.r.t. fuselage length. (% of fus length)
    wing_position_fraction_long: float = Input(0.4)

    #: vertical position w.r.t. to fus  (% of fus radius)
    wing_position_fraction_vrt: float = Input(0.8)

    #: longitudinal position of the vertical tail, as % of fus length
    vt_long: float = Input(0.8)
    vt_taper: float = Input(0.4)

    mesh_deflection: float = Input(1e-4)

    @Part
    def frame(self):
        """Axis system to help visualise the wing local reference frame

        Returns:
            Frame:
        """
        return Frame(pos=self.position)

    @Part
    def fuselage(self):
        return Fuselage(
            radius=self.fu_radius,
            sections=self.fu_sections,
            length=self.fu_length,
            color="Green",
            mesh_deflection=self.mesh_deflection
        )

    @Part
    def right_wing(self):
        """Right side main wing

        Returns:
            LiftingSurface:
        """
        return LiftingSurface(pass_down="airfoil_root_name, airfoil_tip_name,"
                                        "t_factor_root, t_factor_tip",
                              sweep = self.w_sweep,
                              twist = self.w_twist,
                              c_root=self.w_c_root,
                              c_tip=self.w_c_tip,
                              semi_span=self.w_semi_span,
                              dihedral=self.w_dihedral,
                              position=rotate(translate(  # longitudinal and vertical translation of position w.r.t. fuselage
                                  self.position,
                                  "x", self.wing_position_fraction_long * self.fu_length,
                                  "z", self.wing_position_fraction_vrt * - self.fu_radius
                                                       ),
                                              "x", radians(self.w_dihedral) # dihedral applied by rotation of position
                                              #? (…is it a good idea to do this here rather than in the LiftingSurface class?)
                                             ),
                              mesh_deflection=self.mesh_deflection
                             )

    @Part
    def left_wing(self):
        return Solid(MirroredShape(shape_in=self.right_wing,
                             reference_point=self.position,
                             # Two vectors and a point to define the mirror plane
                             vector1=self.position.Vz,
                             vector2=self.position.Vx,
                             mesh_deflection=self.mesh_deflection,
                             ))

    @Part
    def vert_tail(self):
        return LiftingSurface(
            c_root=self.w_c_root,
            c_tip=self.w_c_root * self.vt_taper,
            airfoil_root_name="simm_airfoil",
            airfoil_tip_name="simm_airfoil",
            t_factor_root=0.9 * self.t_factor_root,
            t_factor_tip=0.9 * self.t_factor_tip,
            semi_span=self.w_semi_span / 3,
            sweep=45,
            twist=0,
            position=rotate(translate
                            (self.position,
                             "x", self.vt_long * self.fu_length,
                             "z", self.fu_radius * 0.7),
                            "x",
                            radians(90)),
            mesh_deflection=self.mesh_deflection)

    @Part
    def h_tail_right(self):
        return LiftingSurface(c_root=self.w_c_root / 1.5,
                              c_tip=self.w_c_tip / 2,
                              airfoil_root_name="simm_airfoil",
                              airfoil_tip_name="simm_airfoil",
                              t_factor_root=0.9 * self.t_factor_root,
                              t_factor_tip=0.9 * self.t_factor_tip,
                              semi_span=self.w_semi_span / 2.5,
                              sweep=self.w_sweep + 10,
                              twist=0,
                              dihedral=self.w_dihedral + 5,
                              position=rotate(translate(self.position,
                                                        "x",
                                                        self.fu_length - self.w_c_root
                                                       ),
                                              "x", radians(child.dihedral)
                                             ),
                              mesh_deflection=self.mesh_deflection)

    @Part
    def h_tail_left(self):
        return Solid(MirroredShape(shape_in=self.h_tail_right,
                             reference_point=self.position,
                             # Two vectors and a point to define the mirror plane
                             vector1=self.position.Vz,
                             vector2=self.position.Vx,
                             mesh_deflection=self.mesh_deflection))

    @Part
    def rectangle(self):
        return Rectangle(width=0.1 * self.w_c_root,
                         length=2 * child.width,
                         #! Hey, you can use `child` even in Parts that don't use `quantify`!
                         #! …and you can use it to refer to values of other Part attributes (if there's no dependency loop)!
                         position=translate(
                             rotate90(self.position, 'x'),
                             # self.right_wing.position,
                             'x',
                             self.wing_position_fraction_long * self.fu_length + 0.5 * self.w_c_root,
                             'z', child.length),
                         hidden=False)

    @Part
    def over_wing_exit(self):
        """only one result is shown, but there are actually two wires. See `right_exit` and `left_exit` below."""
        return ProjectedCurve(source=self.rectangle,
                              target=self.fuselage,
                              direction=self.position.Vy)

    @Attribute(in_tree=True)
    def right_exit(self):
        """This is the wire not visualized by over_wing_exit"""
        return self.over_wing_exit.wires[1]

    @Attribute(in_tree=True)
    def left_exit(self):
        """same as over_wing_exit"""
        return self.over_wing_exit.wires[0]

    @Part
    def aircraft_solid(self):
        """Merged solid of all aircraft parts"""
        #! right wing and HTP are merged first because merging left and right
        #! wing can lead to issues, since they touch at the symmetry plane,
        #! which may create extremely thin "residual" volumes.
        #! After merging the right-side parts with the fuselage, there are no
        #! "touching" surfaces anymore.
        return FusedSolid(shape_in=FusedSolid(self.fuselage,
                                              [self.right_wing,
                                              self.h_tail_right]),
                          tool=[self.left_wing,
                                self.vert_tail,
                                self.h_tail_left],
                          mesh_deflection=self.mesh_deflection,
                          color="white")

    @Part
    def step_writer_components(self):
        """Exports the components as separate parts, can be triggered through GUI or via its .write() method"""
        return STEPWriter(default_directory=maindir,
                          nodes=[self.fuselage,
                                 self.left_wing,
                                 self.right_wing,
                                 self.vert_tail,
                                 self.h_tail_left,
                                 self.h_tail_right,
                                 self.over_wing_exit])

    @Part
    def step_writer_fused(self):
        """Exports the aircraft as single part, can be triggered through GUI or via its .write() method"""
        return STEPWriter(default_directory=maindir,
                          nodes=[self.aircraft_solid])


if __name__ == '__main__':
    from parapy.gui import display

    ac = Aircraft(label="aircraft",
                   fu_radius=2.5,
                   #fu_sections=[10, 90, 100, 100, 100, 100, 100, 100, 95, 70, 10],
                   fu_length=50.65,
                   airfoil_root_name="whitcomb",
                   airfoil_tip_name="simm_airfoil",
                   w_c_root=6., w_c_tip=2.3,
                   t_factor_root=1, t_factor_tip=1,
                   w_semi_span=27.,
                   w_sweep=20, w_twist=-5, w_dihedral=3,
                   wing_position_fraction_long=0.4, wing_position_fraction_vrt=0.8,
                   vt_long=0.8, vt_taper=0.4)
    display(ac)
