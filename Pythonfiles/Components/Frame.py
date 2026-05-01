from parapy.geom import GeomBase, LineSegment, translate, Vector
from parapy.core import Input, Attribute, Part, child


class Frame(GeomBase):
    """Clean reference frame visualisation (consistent aircraft-style structure)."""
    
    pos = Input()  # Position object (default external frame)
    
    # ------------------------------------------------------------------ #
    # FRAME VECTORS
    # ------------------------------------------------------------------ #
    
    @Attribute
    def colors(self):
        return ["red", "green", "blue"]
    
    @Attribute
    def axes(self):
        return [
            self.pos.Vx,
            self.pos.Vy,
            self.pos.Vz
        ]
    
    # ------------------------------------------------------------------ #
    # AXES VISUALISATION
    # ------------------------------------------------------------------ #
    
    @Part
    def vectors(self):
        return LineSegment(
            quantify=3,
            start=self.pos.location,
            end=translate(
                self.pos.location,
                self.axes[child.index],
                0.3
            ),
            color=self.colors[child.index],
            line_thickness=2
        )


# ---------------------------------------------------------------------- #
# TEST
# ---------------------------------------------------------------------- #

if __name__ == "__main__":
    from parapy.gui import display

    frame = Frame()
    display(frame)