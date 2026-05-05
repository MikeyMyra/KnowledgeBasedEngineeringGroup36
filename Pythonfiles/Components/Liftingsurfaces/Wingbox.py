from math import radians, tan, sin

from parapy.core import Input, Attribute, Part
from parapy.geom import GeomBase, LoftedSolid, translate, rotate, Point, Polygon

from Pythonfiles.Components.Liftingsurfaces.Airfoil import Airfoil
from Pythonfiles.Components.Frame import Frame


class Wingbox(GeomBase):
    """Wingbox fitted inside airfoil using real local thickness."""
    
    c_root: float = Input()
    c_tip: float = Input()
    semi_span: float = Input()
    sweep_le: float = Input()
    dihedral: float = Input()
    twist: float = Input()
    
    front_spar_position: float = Input()
    rear_spar_position: float = Input()
    
    # airfoil objects
    airfoil_root: "Airfoil" = Input()
    airfoil_tip: "Airfoil" = Input()
    
    mesh_deflection: float = Input(1e-4)
    
    color: str = Input()
    
    # ------------------------------------------------------------------ #
    # WIDTH 
    # ------------------------------------------------------------------ #
    
    @Attribute
    def width_root(self):
        return (self.rear_spar_position - self.front_spar_position) * self.c_root
    
    @Attribute
    def width_tip(self):
        return (self.rear_spar_position - self.front_spar_position) * self.c_tip
    
    # ------------------------------------------------------------------ #
    # SPAR POSITIONS
    # ------------------------------------------------------------------ #
    
    @Attribute
    def front_spar_x_root(self):
        return self.front_spar_position * self.c_root
    
    @Attribute
    def front_spar_x_tip(self):
        return self.front_spar_position * self.c_tip
    
    # ------------------------------------------------------------------ #
    # CORNERS
    # ------------------------------------------------------------------ #
    
    @Attribute
    def _root_corners(self):
        x_front = self.front_spar_position
        x_rear = self.rear_spar_position
        
        zf_u, zf_l = self.airfoil_root.surface_z_at(x_front)
        zr_u, zr_l = self.airfoil_root.surface_z_at(x_rear)
        
        xf = x_front * self.c_root
        xr = x_rear * self.c_root
        
        pos = self.airfoil_root.position.location
        
        return [
            Point(pos.x + xf, pos.y, pos.z + zf_l),
            Point(pos.x + xr, pos.y, pos.z + zr_l),
            Point(pos.x + xr, pos.y, pos.z + zr_u),
            Point(pos.x + xf, pos.y, pos.z + zf_u),
        ]
    
    @Attribute
    def _tip_corners(self):
        x_front = self.front_spar_position
        x_rear = self.rear_spar_position
        
        zf_u, zf_l = self.airfoil_tip.surface_z_at(x_front)
        zr_u, zr_l = self.airfoil_tip.surface_z_at(x_rear)
        
        xf = x_front * self.c_tip
        xr = x_rear * self.c_tip
        
        # 1. Base spanwise position ONLY (NO dihedral here)
        base_pos = rotate(
            translate(
                self.airfoil_root.position,
                "x", abs(self.semi_span) * tan(radians(abs(self.sweep_le))),
                "y", self.semi_span,
                "z", abs(self.semi_span) * sin(radians(abs(self.dihedral))),
            ), "y", radians(self.twist)
        )
        
        p0 = base_pos.location
        
        # 2. Local section (airfoil box in its own frame)
        corners = [
            Point(xf, 0, zf_l),
            Point(xr, 0, zr_l),
            Point(xr, 0, zr_u),
            Point(xf, 0, zf_u),
        ]
        
        # 3. Apply twist first (local rotation)
        twisted = [
            rotate(corner, "y", radians(self.twist))
            for corner in corners
        ]
        
        # 4. Apply dihedral as global rotation around root X-axis
        dihedraled = [
            rotate(c, "x", radians(self.dihedral))
            for c in twisted
        ]
        
        # 5. Translate into position
        return [
            Point(p0.x + c.x, p0.y, p0.z + c.z)
            for c in dihedraled
        ]
    
    # ------------------------------------------------------------------ #
    # SECTIONS
    # ------------------------------------------------------------------ #
    
    @Part
    def root_section(self):
        return Polygon(
            points=self._root_corners,
            color=self.color,
            mesh_deflection=self.mesh_deflection,
        )
    
    @Part
    def tip_section(self):
        return Polygon(
            points=self._tip_corners,
            color=self.color,
            mesh_deflection=self.mesh_deflection,
        )
    
    # ------------------------------------------------------------------ #
    # SOLID
    # ------------------------------------------------------------------ #
    
    @Part
    def solid(self):
        return LoftedSolid(
            profiles=[self.root_section, self.tip_section],
            color=self.color,
            mesh_deflection=self.mesh_deflection,
        )
    
    # ------------------------------------------------------------------ #
    # FRAME VISUALISATION
    # ------------------------------------------------------------------ #
    
    @Part
    def Frame_root(self):
        return Frame(
            pos=self.position,
            hidden=False,
        )
    
    @Part
    def Frame_tip(self):
        return Frame(
            pos=self.position,
            hidden=False,
        )


# ---------------------------------------------------------------------- #
# TEST
# ---------------------------------------------------------------------- #

if __name__ == "__main__":
    from parapy.gui import display
    
    root_af = Airfoil(
        label="test_root_airfoil",
        chord=5.0,
        maximum_camber=0.00,
        camber_position=0.0,
        thickness_to_chord=0.12,
        export_dat=False,
        airfoil_name="naca_0012",
    )
    
    tip_af = Airfoil(
        label="test_tip_airfoil",
        chord=2.0,
        maximum_camber=0.00,
        camber_position=0.0,
        thickness_to_chord=0.12,
        export_dat=False,
        airfoil_name="naca_0012",
    )
    
    wb = Wingbox(
        c_root=5.0,
        c_tip=2.0,
        semi_span=15.0,
        sweep_le=0,
        dihedral=0,
        twist=0,
        front_spar_position=0.15,
        rear_spar_position=0.6,
        airfoil_root=root_af,
        airfoil_tip=tip_af,
        label="test_wingbox",
    )
    display(wb)