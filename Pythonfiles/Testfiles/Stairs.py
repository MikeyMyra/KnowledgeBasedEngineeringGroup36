from math import pi, cos, sin
from parapy.geom import (GeomBase, Cylinder, Compound,
                         ExtrudedSolid, LineSegment, FittedCurve,
                         Point, Wire, Vector)
from parapy.core import Input, Attribute, Part


class SpiralStairCase(GeomBase):
    """
    A parametric spiral staircase with customizable dimensions and appearance.

    Parameters:
    - number_of_steps: Total number of steps in the staircase
    - step_width: Radial width of each step (meters)
    - step_height: Vertical rise between consecutive steps (meters)
    - step_thickness: Thickness of each step tread (meters)
    - inner_radius: Radius of the inner column (meters)
    - number_of_revolutions: Number of complete rotations the staircase makes
    - width_increment: How much wider each step becomes (meters per step)
    - step_colors: List of colors to cycle through for the steps
    """

    # Input parameters with clear names
    number_of_steps = Input(30)
    step_width = Input(1.5)
    step_height = Input(0.25)
    step_thickness = Input(0.25)
    inner_radius = Input(1.0)
    number_of_revolutions = Input(2)
    width_increment = Input(0.1)
    step_colors = Input(["red", "green", "blue", "yellow"])

    # -------------------------------------------------
    # Calculated Parameters
    # -------------------------------------------------
    @Attribute
    def angular_step(self):
        """Angle between consecutive steps (radians)"""
        return self.number_of_revolutions * 2 * pi / self.number_of_steps

    @Attribute
    def total_staircase_height(self):
        """Total vertical height of the staircase"""
        return self.step_height * self.number_of_steps

    @Attribute
    def maximum_outer_radius(self):
        """Outer radius at the top step (accounts for width increment)"""
        return self.inner_radius + self.step_width + self.width_increment * (self.number_of_steps - 1)

    # -------------------------------------------------
    # Step Wire Generation
    # -------------------------------------------------
    def create_step_wire(self, step_index):
        """
        Create a wire outline for a single step at the given index.

        Args:
            step_index: Index of the step (0 to number_of_steps - 1)

        Returns:
            Wire object defining the step's horizontal outline
        """
        # Calculate angles for this step
        angle_start = step_index * self.angular_step
        angle_end = (step_index + 1) * self.angular_step
        angle_middle = (angle_start + angle_end) / 2

        # Calculate outer radius for this step (grows with each step if width_increment > 0)
        outer_radius = self.inner_radius + self.step_width + step_index * self.width_increment

        # Calculate vertical position (height) for this step
        step_z_position = step_index * self.step_height

        # Inner arc points (along inner column)
        inner_point_start = Point(self.inner_radius * cos(angle_start),
                                  self.inner_radius * sin(angle_start),
                                  step_z_position)
        inner_point_end = Point(self.inner_radius * cos(angle_end),
                                self.inner_radius * sin(angle_end),
                                step_z_position)
        inner_point_mid = Point(self.inner_radius * cos(angle_middle),
                                self.inner_radius * sin(angle_middle),
                                step_z_position)

        # Outer arc points (along outer edge)
        outer_point_start = Point(outer_radius * cos(angle_start),
                                  outer_radius * sin(angle_start),
                                  step_z_position)
        outer_point_end = Point(outer_radius * cos(angle_end),
                                outer_radius * sin(angle_end),
                                step_z_position)
        outer_point_mid = Point(outer_radius * cos(angle_middle),
                                outer_radius * sin(angle_middle),
                                step_z_position)

        # Create curved edges using FittedCurve (smooth arc through 3 points)
        inner_arc = FittedCurve(points=[inner_point_start, inner_point_mid, inner_point_end])
        outer_arc = FittedCurve(points=[outer_point_end, outer_point_mid, outer_point_start])

        # Create straight radial edges connecting inner and outer arcs
        radial_line_end = LineSegment(start=inner_point_end, end=outer_point_end)
        radial_line_start = LineSegment(start=outer_point_start, end=inner_point_start)

        # Construct closed wire (counter-clockwise)
        step_wire = Wire([inner_arc, radial_line_end, outer_arc, radial_line_start])

        return step_wire

    # -------------------------------------------------
    # Step Solids Generation
    # -------------------------------------------------
    @Attribute
    def all_step_solids(self):
        """Generate all step solids as a list"""
        step_list = []

        for step_index in range(self.number_of_steps):
            # Get the color for this step (cycle through the color list)
            step_color = self.step_colors[step_index % len(self.step_colors)]

            # Create the step solid by extruding the wire downward
            # The wire is at the top of the step, we extrude down to create the riser
            step_solid = ExtrudedSolid(
                island=self.create_step_wire(step_index),
                direction=Vector(0, 0, -2*self.step_thickness),
                color=step_color
            )

            step_list.append(step_solid)

        return step_list

    # -------------------------------------------------
    # Parts (visible in ParaPy tree)
    # -------------------------------------------------
    @Part
    def steps(self):
        """All steps combined into a single compound geometry"""
        return Compound(built_from=self.all_step_solids)

    @Part
    def inner_column(self):
        """Central support column"""
        return Cylinder(
            radius=self.inner_radius,
            height=self.total_staircase_height,
            color="black"
        )

    @Part
    def outer_column(self):
        """Outer transparent column showing the envelope"""
        return Cylinder(
            radius=self.maximum_outer_radius,
            height=self.total_staircase_height,
            transparency=0.8,
            color="gray"
        )


if __name__ == '__main__':
    from parapy.gui import display

    # Create a spiral staircase instance
    my_staircase = SpiralStairCase(
        number_of_steps=30,
        step_width=1.5,
        step_height=0.25,
        step_thickness=0.25,
        inner_radius=1.0,
        number_of_revolutions=2,
        width_increment=0,
        step_colors=["red", "green", "blue", "yellow"]
    )

    display(my_staircase)