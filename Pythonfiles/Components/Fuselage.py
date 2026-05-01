import math
from parapy.core import Input, Attribute, Part, child
from parapy.geom import GeomBase, LoftedSolid, Circle, Vector, translate
from Fuselage.Undercarriage import Undercarriage


class Fuselage(GeomBase):
    """Fuselage has nosecone (tapered) part, then main (cylinder) part and then the tailcone (tapered) part"""
    
    aircraft_mass = Input()
    
    radius: float = Input()
    cylinder_start: float = Input()
    cylinder_end: float = Input()
    length: float = Input()
    
    min_radius_pct: float = Input(0.0001)
    
    taper_sections: int = Input(10)
    color_taper: str = Input("Orange")
    
    cylinder_sections: int = Input(4)
    color_cylinder: str = Input("Yellow")
    
    material_skin: str = Input()
    undercarriage_retractible: bool = Input()
    mesh_deflection: float = Input(1e-4)
    
    # ------------------------------------------------------------------ #
    # Child components
    # ------------------------------------------------------------------ #
    
    @Part
    def undercarriage(self):
        return Undercarriage(
            retractible=self.undercarriage_retractible,
            aircraft_mass=self.aircraft_mass,
            fuselage_length=self.length,
            fuselage_radius=self.radius,
            label="undercarriage",
    )
    
    # ------------------------------------------------------------------ #
    # Nose cone geometry definition  
    # ------------------------------------------------------------------ #
    
    # Calculate positions of nose cone circles                                                        
    @Attribute
    def _nose_positions(self) -> list[float]:
        cs = self.cylinder_start / 100.0
        n  = self.taper_sections
        return [(i / (n - 1)) * cs * self.length for i in range(n)]
    
    # Calculate radii of nose cone circles
    @Attribute
    def _nose_radii(self) -> list[float]:
        r_min = self.min_radius_pct / 100.0
        n     = self.taper_sections
        def ellipse_blend(t):
            return r_min + (1.0 - r_min) * math.sqrt(max(0.0, 1.0 - t ** 2))
        return [ellipse_blend(1.0 - i / (n - 1)) * self.radius for i in range(n)]
    
    # Create nose cone profile
    @Part
    def nose_profiles(self):
        return Circle(
            quantify=self.taper_sections,
            color=self.color_taper,
            radius=self._nose_radii[child.index],
            position=translate(
                self.position.rotate90('y'),
                Vector(1, 0, 0),
                self._nose_positions[child.index],
            ),
        )
    
    # Create nose cone solid
    @Part
    def nose(self):
        return LoftedSolid(
            profiles=self.nose_profiles,
            color=self.color_taper,
            mesh_deflection=self.mesh_deflection,
        )
    
    # ------------------------------------------------------------------ #
    # Cylindrical part geometry definition  
    # ------------------------------------------------------------------ #
    
    # Calculate positions of main part circles
    @Attribute
    def _cyl_positions(self) -> list[float]:
        cs = self.cylinder_start / 100.0
        ce = self.cylinder_end   / 100.0
        nc = self.cylinder_sections
        # include junction at cs (i=0) through ce (i=nc)
        return [( cs + (i / nc) * (ce - cs) ) * self.length for i in range(nc + 1)]
    
    # Calculate radii of main part circles
    @Attribute
    def _cyl_radii(self) -> list[float]:
        return [self.radius] * (self.cylinder_sections + 1)
    
    # Create main part profile
    @Part
    def cyl_profiles(self):
        return Circle(
            quantify=self.cylinder_sections + 1,
            color=self.color_cylinder,
            radius=self._cyl_radii[child.index],
            position=translate(
                self.position.rotate90('y'),
                Vector(1, 0, 0),
                self._cyl_positions[child.index],
            ),
        )
    
    # Create main part solid
    @Part
    def cylinder(self):
        return LoftedSolid(
            profiles=self.cyl_profiles,
            color=self.color_cylinder,
            mesh_deflection=self.mesh_deflection,
        )
    
    # ------------------------------------------------------------------ #
    # Tail cone geometry definition  
    # ------------------------------------------------------------------ #
    
    # Calculate positions of tail cone circles 
    @Attribute
    def _tail_positions(self) -> list[float]:
        ce = self.cylinder_end / 100.0
        n  = self.taper_sections
        # i=0 is the ce junction, i=n-1 is the tip
        return [(ce + (i / (n - 1)) * (1.0 - ce)) * self.length for i in range(n)]
    
    # Calculate radii of tail cone circles 
    @Attribute
    def _tail_radii(self) -> list[float]:
        r_min = self.min_radius_pct / 100.0
        n     = self.taper_sections
        def ellipse_blend(t):
            return r_min + (1.0 - r_min) * math.sqrt(max(0.0, 1.0 - t ** 2))
        return [ellipse_blend(i / (n - 1)) * self.radius for i in range(n)]
    
    # Create tail cone profile
    @Part
    def tail_profiles(self):
        return Circle(
            quantify=self.taper_sections,
            color=self.color_taper,
            radius=self._tail_radii[child.index],
            position=translate(
                self.position.rotate90('y'),
                Vector(1, 0, 0),
                self._tail_positions[child.index],
            ),
        )
    
    # Create tail cone solid
    @Part
    def tail(self):
        return LoftedSolid(
            profiles=self.tail_profiles,
            color=self.color_taper,
            mesh_deflection=self.mesh_deflection,
        )
    
    # ------------------------------------------------------------------ #
    # Calculations #TODO: Give these useful output
    # ------------------------------------------------------------------ #
    
    # Calculate mass
    @Attribute
    def calculate_mass(self):
        return 1
    
    # Calculate volume
    @Attribute
    def calculate_volume(self):
        return 1
    
    # Calculate skin friction
    @Attribute
    def calculate_skin_friction(self):
        return 1


# Test
if __name__ == '__main__':
    
    from parapy.gui import display
    obj = Fuselage(
        aircraft_mass=50000,
        radius=1,
        cylinder_start=10,
        cylinder_end=70,
        length=20,
        taper_sections=10,
        cylinder_sections=4,
        material_skin="base_material",
        undercarriage_retractible=False,
        label="base_fuselage",
    )
    display(obj)