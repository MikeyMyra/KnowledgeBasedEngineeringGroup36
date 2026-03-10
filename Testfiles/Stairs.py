#!/usr/bin/env python
# -*- coding: utf-8 -*-

from math import pi, cos, sin
from parapy.geom import (
    GeomBase, Cylinder, Position,
    ExtrudedSolid, LineSegment, Arc, Face, Point
)
from parapy.core import Input, Attribute, Part


class SpiralStairCase(GeomBase):
    """Spiral staircase with wedge-shaped steps around a central column."""

    n_step: int = Input()
    w_step: float = Input()
    h_step: float = Input()
    t_step: float = Input()
    radius: float = Input()
    n_revol: float = Input()
    delta_w: float = Input(0)
    colors: list[str] = Input(["red", "green", "blue", "yellow", "orange"])

    @Attribute
    def angle_step(self):
        """Angular step between consecutive steps"""
        return self.n_revol * 2 * pi / (self.n_step - 1)

    @Attribute
    def total_height(self):
        """Total height of the staircase"""
        return self.h_step * self.n_step

    @Attribute
    def max_outer_radius(self):
        """Maximum outer radius at the top step"""
        return self.radius + self.w_step + self.delta_w * (self.n_step - 1)

    def get_step_face(self, i):
        """Create the wedge-shaped face for step i"""
        theta0 = i * self.angle_step
        theta1 = (i + 1) * self.angle_step
        theta_mid = (theta0 + theta1) / 2
        r_outer = self.radius + self.w_step + i * self.delta_w

        # Inner arc points
        p_inner_start = Point(
            self.radius * cos(theta0),
            self.radius * sin(theta0),
            0
        )
        p_inner_mid = Point(
            self.radius * cos(theta_mid),
            self.radius * sin(theta_mid),
            0
        )
        p_inner_end = Point(
            self.radius * cos(theta1),
            self.radius * sin(theta1),
            0
        )

        # Outer arc points
        p_outer_start = Point(
            r_outer * cos(theta0),
            r_outer * sin(theta0),
            0
        )
        p_outer_mid = Point(
            r_outer * cos(theta_mid),
            r_outer * sin(theta_mid),
            0
        )
        p_outer_end = Point(
            r_outer * cos(theta1),
            r_outer * sin(theta1),
            0
        )

        # Create arcs
        inner_arc = Arc(p_inner_start, p_inner_mid, p_inner_end)
        outer_arc = Arc(p_outer_start, p_outer_mid, p_outer_end)

        # Create radial lines
        radial_line1 = LineSegment(p_inner_start, p_outer_start)
        radial_line2 = LineSegment(p_inner_end, p_outer_end)

        # Create face
        return Face([
            inner_arc,
            radial_line2,
            outer_arc.reversed,
            radial_line1
        ])

    @Part(parse=False)
    def steps(self):
        steps = []
        for i in range(self.n_step):
            step = ExtrudedSolid(
                island=self.get_step_face(i),
                distance=self.t_step,
                position=Position(location=Point(0, 0, i * self.h_step)),
                color=self.colors[i % len(self.colors)],
                parent=self
            )
            steps.append(step)
        return steps

    @Part
    def inner_column(self):
        return Cylinder(
            radius=self.radius,
            height=self.total_height,
            color="black"
        )

    @Part
    def outer_column(self):
        return Cylinder(
            radius=self.max_outer_radius,
            height=self.total_height,
            color="black",
            transparency=0.8
        )


if __name__ == '__main__':
    from parapy.gui import display

    sp_stairs = SpiralStairCase(
        n_step=30,
        w_step=1.5,
        h_step=0.25,
        t_step=0.18,
        radius=1,
        n_revol=2,
        delta_w=0.1,
        colors=["red", "green", "blue", "yellow"]
    )

    display(sp_stairs)
