import os
import numpy as np

from parapy.geom import GeomBase, FittedCurve
from parapy.core import Input, Attribute, Part

from Pythonfiles.Components.Frame import Frame


class Airfoil(GeomBase):
    """Parametric NACA 4-series airfoil with automatic .dat export."""

    # ------------------------------------------------------------------ #
    # INPUTS
    # ------------------------------------------------------------------ #

    chord: float = Input(1.0)

    maximum_camber: float = Input(0.02)          # m (e.g. 0.02 = 2%)
    camber_position: float = Input(0.4)          # p (0–1)
    thickness_to_chord: float = Input(0.12)      # t (e.g. 0.12 = 12%)

    n_points: int = Input(100)
    thickness_factor: float = Input(1.0)
    mesh_deflection: float = Input(1e-4)

    export_dat: bool = Input(True)
    airfoil_name: str = Input("custom_airfoil")

    # ------------------------------------------------------------------ #
    # NACA GENERATION
    # ------------------------------------------------------------------ #

    @Attribute
    def x_distribution(self):
        """Cosine spacing for better LE resolution."""
        beta = np.linspace(0, np.pi, self.n_points)
        return 0.5 * (1 - np.cos(beta))

    @Attribute
    def thickness_distribution(self):
        """Thickness distribution (NACA formula)."""
        x = self.x_distribution
        t = self.thickness_to_chord

        yt = 5 * t * (
            0.2969 * np.sqrt(x)
            - 0.1260 * x
            - 0.3516 * x**2
            + 0.2843 * x**3
            - 0.1015 * x**4
        )
        return yt

    @Attribute
    def camber_line(self):
        """Camber line and slope."""
        x = self.x_distribution
        m = self.maximum_camber
        p = self.camber_position

        yc = np.zeros_like(x)
        dyc_dx = np.zeros_like(x)

        for i, xi in enumerate(x):
            if xi < p:
                yc[i] = m / (p**2) * (2 * p * xi - xi**2)
                dyc_dx[i] = 2 * m / (p**2) * (p - xi)
            else:
                yc[i] = m / ((1 - p)**2) * (1 - 2 * p + 2 * p * xi - xi**2)
                dyc_dx[i] = 2 * m / ((1 - p)**2) * (p - xi)

        return yc, dyc_dx

    @Attribute
    def normalized_coordinates(self):
        """Full airfoil coordinates (x, z)."""
        x = self.x_distribution
        yt = self.thickness_distribution
        yc, dyc_dx = self.camber_line

        theta = np.arctan(dyc_dx)

        xu = x - yt * np.sin(theta)
        zu = yc + yt * np.cos(theta)

        xl = x + yt * np.sin(theta)
        zl = yc - yt * np.cos(theta)

        # Combine upper + lower (TE -> LE -> TE)
        x_full = np.concatenate([xu[::-1], xl[1:]])
        z_full = np.concatenate([zu[::-1], zl[1:]])

        return np.vstack((x_full, z_full)).T

    # ------------------------------------------------------------------ #
    # EXPORT .DAT FILE
    # ------------------------------------------------------------------ #

    @Attribute
    def dat_file_path(self):
        base_dir = os.path.dirname(__file__)
        folder = os.path.join(base_dir, "Airfoils")

        if not os.path.exists(folder):
            os.makedirs(folder)

        filename = f"{self.airfoil_name}.dat"
        return os.path.join(folder, filename)

    @Attribute
    def write_dat_file(self):
        if not self.export_dat:
            return None

        coords = self.normalized_coordinates

        with open(self.dat_file_path, "w") as f:
            f.write(f"{self.airfoil_name}\n")
            for x, z in coords:
                f.write(f"{x:.6f} {z:.6f}\n")

        return self.dat_file_path

    # ------------------------------------------------------------------ #
    # GEOMETRY POINTS
    # ------------------------------------------------------------------ #

    @Attribute
    def raw_points(self):
        coords = self.normalized_coordinates

        pts = []
        for x, z in coords:
            pts.append(
                self.position.translate(
                    "x", x * self.chord,
                    "z", z * self.chord * self.thickness_factor,
                )
            )
        return pts

    # ------------------------------------------------------------------ #
    # SURFACE SPLIT
    # ------------------------------------------------------------------ #

    @Attribute
    def upper_lower_surfaces(self):
        coords = self.normalized_coordinates
        le_idx = np.argmin(coords[:, 0])

        upper = coords[:le_idx + 1][::-1]
        lower = coords[le_idx:]

        return upper, lower

    # ------------------------------------------------------------------ #
    # THICKNESS QUERY
    # ------------------------------------------------------------------ #

    def surface_z_at(self, x_c: float):
        upper, lower = self.upper_lower_surfaces

        x_u, z_u = upper[:, 0], upper[:, 1]
        x_l, z_l = lower[:, 0], lower[:, 1]

        z_upper = np.interp(x_c, x_u, z_u)
        z_lower = np.interp(x_c, x_l, z_l)

        return (
            z_upper * self.thickness_factor * self.chord,
            z_lower * self.thickness_factor * self.chord,
        )

    # ------------------------------------------------------------------ #
    # GEOMETRY
    # ------------------------------------------------------------------ #

    @Part
    def geometry(self):
        return FittedCurve(
            points=self.raw_points,
            mesh_deflection=self.mesh_deflection,
        )

    # ------------------------------------------------------------------ #
    # FRAME
    # ------------------------------------------------------------------ #

    @Part
    def frame(self):
        return Frame(pos=self.position, hidden=False)


# ---------------------------------------------------------------------- #
# TEST
# ---------------------------------------------------------------------- #

if __name__ == "__main__":
    from parapy.gui import display
    
    airfoil = Airfoil(
        label="test_airfoil",
        chord=1.5,
        maximum_camber=0.04,
        camber_position=0.4,
        thickness_to_chord=0.15,
        airfoil_name="naca_4415",
    )
    display(airfoil)