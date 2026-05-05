import os
import numpy as np
from math import comb

from parapy.geom import GeomBase, FittedCurve
from parapy.core import Input, Attribute, Part

from Pythonfiles.Components.Frame import Frame


class Airfoil(GeomBase):
    """Parametric NACA 4-series airfoil with automatic .dat export."""

    # ------------------------------------------------------------------ #
    # INPUTS
    # ------------------------------------------------------------------ #

    chord: float = Input()

    maximum_camber: float = Input()          # m (e.g. 0.02 = 2%)
    camber_position: float = Input()          # p (0–1)
    thickness_to_chord: float = Input()      # t (e.g. 0.12 = 12%)

    thickness_factor: float = Input(1)
    n_points: int = Input(100)
    mesh_deflection: float = Input(1e-4)

    export_dat: bool = Input()
    airfoil_name: str = Input()

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
    # CST COEFFICIENTS
    # ------------------------------------------------------------------ #
    
    def bernstein_poly(self, i, n, x):
        return comb(n, i) * (x**i) * ((1 - x)**(n - i))


    def class_function(self, x, N1=0.5, N2=1.0):
        return (x**N1) * ((1 - x)**N2)


    def fit_cst(self, x, z, n_coeff=6):
        n = n_coeff - 1

        C = self.class_function(x)
        B = np.zeros((len(x), n_coeff))

        for i in range(n_coeff):
            B[:, i] = self.bernstein_poly(i, n, x)

        A = C[:, None] * B

        # 🔧 Tikhonov regularization
        lambda_reg = 1e-8
        ATA = A.T @ A + lambda_reg * np.eye(n_coeff)
        ATz = A.T @ z

        coeffs = np.linalg.solve(ATA, ATz)
        return coeffs
    
    @Attribute
    def CST_coefficients(self):
        """Return CST coefficients: (upper[6], lower[6])"""

        upper, lower = self.upper_lower_surfaces

        # Extract
        x_u, z_u = upper[:, 0], upper[:, 1]
        x_l, z_l = lower[:, 0], lower[:, 1]

        # Sort
        idx_u = np.argsort(x_u)
        idx_l = np.argsort(x_l)

        x_u, z_u = x_u[idx_u], z_u[idx_u]
        x_l, z_l = x_l[idx_l], z_l[idx_l]

        eps = 1e-6

        # Remove problematic endpoints
        mask_u = (x_u > eps) & (x_u < 1 - eps)
        mask_l = (x_l > eps) & (x_l < 1 - eps)

        x_u, z_u = x_u[mask_u], z_u[mask_u]
        x_l, z_l = x_l[mask_l], z_l[mask_l]

        # Normalize (extra safety)
        x_u = np.clip(x_u, eps, 1 - eps)
        x_l = np.clip(x_l, eps, 1 - eps)

        coeffs_upper = self.fit_cst(x_u, z_u, n_coeff=6)

        # IMPORTANT: fit lower surface as absolute then restore sign
        coeffs_lower = self.fit_cst(x_l, -z_l, n_coeff=6)
        coeffs_lower = -coeffs_lower

        return coeffs_upper, coeffs_lower
    
    @Attribute
    def CST_vector(self):
        cu, cl = self.CST_coefficients
        return np.concatenate([cu, cl])

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
        chord=1.0,
        maximum_camber=0.00,
        camber_position=0.0,
        thickness_to_chord=0.12,
        export_dat=True,
        airfoil_name="naca_0012",
    )
    display(airfoil)