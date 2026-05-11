import os
import subprocess
from math import comb

import numpy as np
import matplotlib.pyplot as plt

from parapy.core import Attribute, Input, Part, action
from parapy.geom import FittedCurve, GeomBase

from Pythonfiles.Components.Frame import Frame

_XFOIL_DIR  = os.path.abspath("XFOIL6.99")
_AIRFOIL_DIR = os.path.join(_XFOIL_DIR, "Airfoils")
_XFOIL_EXE  = os.path.join(_XFOIL_DIR, "xfoil.exe")


class Airfoil(GeomBase):
    """Parametric NACA 4-series airfoil with CST fitting and XFoil polar."""

    # ------------------------------------------------------------------ #
    # INPUTS
    # ------------------------------------------------------------------ #

    chord:              float = Input()
    maximum_camber:     float = Input()   # m  (e.g. 0.04 = 4 %)
    camber_position:    float = Input()   # p  (0–1)
    thickness_to_chord: float = Input()   # t  (e.g. 0.12 = 12 %)

    thickness_factor:   float = Input(1.0)
    n_points:           int   = Input(100)
    mesh_deflection:    float = Input(1e-4)

    export_dat:   bool  = Input(True)
    mach:         float = Input()
    reynolds:     float = Input()
    
    alpha_cruise: float = Input()

    # airfoil_name defaults to the auto-derived NACA string but can be
    # overridden (e.g. airfoil_name="root_airfoil_geometric") so that
    # the saved .dat / polar files use a meaningful filename.
    @Attribute
    def _naca_name(self):
        """Auto-derived NACA 4-digit string, e.g. 'naca2412'."""
        m = round(self.maximum_camber * 100)
        p = round(self.camber_position * 10)
        t = round(self.thickness_to_chord * 100)
        return f"naca{m}{p}{t:02d}"

    airfoil_name: str = Input(None)   # None → falls back to _naca_name

    # ------------------------------------------------------------------ #
    # DERIVED NAMES / PATHS
    # ------------------------------------------------------------------ #

    @Attribute
    def resolved_name(self):
        """The name actually used for all file I/O."""
        return self.airfoil_name if self.airfoil_name is not None else self._naca_name

    @Attribute
    def dat_file_path(self):
        os.makedirs(_AIRFOIL_DIR, exist_ok=True)
        return os.path.join(_AIRFOIL_DIR, f"{self.resolved_name}.dat")

    @Attribute
    def polar_file_path(self):
        return os.path.join(_AIRFOIL_DIR, f"{self.resolved_name}_polar.txt")

    @Attribute
    def dump_file_path(self):
        return os.path.join(_AIRFOIL_DIR, f"{self.resolved_name}_dump.txt")

    # ------------------------------------------------------------------ #
    # NACA GEOMETRY
    # ------------------------------------------------------------------ #

    @Attribute
    def x_distribution(self):
        """Cosine-spaced x stations for better LE resolution."""
        beta = np.linspace(0, np.pi, self.n_points)
        return 0.5 * (1 - np.cos(beta))

    @Attribute
    def thickness_distribution(self):
        x = self.x_distribution
        t = self.thickness_to_chord
        return 5 * t * (
              0.2969 * np.sqrt(x)
            - 0.1260 * x
            - 0.3516 * x**2
            + 0.2843 * x**3
            - 0.1015 * x**4
        )

    @Attribute
    def camber_line(self):
        """Returns (yc, dyc_dx). Guards m=0 or p=0 (symmetric)."""
        x  = self.x_distribution
        m  = self.maximum_camber
        p  = self.camber_position

        yc     = np.zeros_like(x)
        dyc_dx = np.zeros_like(x)

        if m == 0.0 or p == 0.0:
            return yc, dyc_dx

        fore = x < p
        yc[fore]     = m / p**2       * (2 * p * x[fore] - x[fore]**2)
        dyc_dx[fore] = 2 * m / p**2   * (p - x[fore])

        aft = ~fore
        yc[aft]     = m / (1-p)**2    * (1 - 2*p + 2*p*x[aft] - x[aft]**2)
        dyc_dx[aft] = 2 * m / (1-p)**2 * (p - x[aft])

        return yc, dyc_dx

    @Attribute
    def normalized_coordinates(self):
        """Full (x, z) airfoil in unit-chord space."""
        x          = self.x_distribution
        yt         = self.thickness_distribution * self.thickness_factor
        yc, dyc_dx = self.camber_line
        theta      = np.arctan(dyc_dx)

        xu = x - yt * np.sin(theta);  zu = yc + yt * np.cos(theta)
        xl = x + yt * np.sin(theta);  zl = yc - yt * np.cos(theta)

        x_full = np.concatenate([xu[::-1], xl[1:]])
        z_full = np.concatenate([zu[::-1], zl[1:]])
        return np.vstack((x_full, z_full)).T

    @Attribute
    def upper_lower_surfaces(self):
        coords = self.normalized_coordinates
        le     = np.argmin(coords[:, 0])
        return coords[:le + 1][::-1], coords[le:]   # (upper, lower)

    def surface_z_at(self, x_c: float):
        """Interpolated upper/lower z at chord fraction x_c (physical units)."""
        upper, lower = self.upper_lower_surfaces
        z_u = np.interp(x_c, upper[:, 0], upper[:, 1]) * self.chord
        z_l = np.interp(x_c, lower[:, 0], lower[:, 1]) * self.chord
        return z_u, z_l

    # ------------------------------------------------------------------ #
    # .DAT EXPORT
    # ------------------------------------------------------------------ #

    @Attribute
    def write_dat_file(self):
        """Write the Selig-format .dat file and return its path (or None if export_dat=False)."""
        if not self.export_dat:
            return None

        path   = self.dat_file_path
        coords = self.normalized_coordinates

        with open(path, "w") as f:
            f.write(f"{self.resolved_name}\n")
            for x, z in coords:
                f.write(f"{x:.6f} {z:.6f}\n")

        print(f"DAT written → {path}")
        return path

    # ------------------------------------------------------------------ #
    # CST FITTING
    # ------------------------------------------------------------------ #

    def _bernstein(self, i, n, x):
        return comb(n, i) * x**i * (1 - x)**(n - i)

    def _class_fn(self, x, N1=0.5, N2=1.0):
        return x**N1 * (1 - x)**N2

    def _fit_cst(self, x, z, n_coeff=6):
        n   = n_coeff - 1
        C   = self._class_fn(x)
        B   = np.column_stack([self._bernstein(i, n, x) for i in range(n_coeff)])
        A   = C[:, None] * B
        lam = 1e-8
        return np.linalg.solve(A.T @ A + lam * np.eye(n_coeff), A.T @ z)

    @Attribute
    def CST_coefficients(self):
        upper, lower = self.upper_lower_surfaces
        eps = 1e-6

        def _prep(surf):
            idx  = np.argsort(surf[:, 0])
            x, z = surf[idx, 0], surf[idx, 1]
            mask = (x > eps) & (x < 1 - eps)
            return np.clip(x[mask], eps, 1 - eps), z[mask]

        xu, zu = _prep(upper)
        xl, zl = _prep(lower)

        cu = self._fit_cst(xu,  zu)
        cl = self._fit_cst(xl, -zl)
        cl = -cl
        return cu, cl

    @Attribute
    def CST_vector(self):
        """Flat 12-element array [upper(6), lower(6)] for Q3D."""
        cu, cl = self.CST_coefficients
        return np.concatenate([cu, cl])

    # ------------------------------------------------------------------ #
    # XFOIL
    # ------------------------------------------------------------------ #

    def run_xfoil(
        self,
        reynolds:    float = None,
        mach:        float = None,
        alpha_start: float = -4.0,
        alpha_end:   float = 20.0,
        alpha_step:  float = 0.5,
        n_iter:      int   = 200,
    ):
        """
        Run XFoil for the airfoil.  Writes the polar to ``polar_file_path``
        and the BL dump to ``dump_file_path``.  Returns (alphas, cls).

        ``reynolds`` and ``mach`` default to the values set on the instance
        (Input attributes) so callers never need to repeat them.
        """
        reynolds = reynolds if reynolds is not None else self.reynolds
        mach     = mach     if mach     is not None else self.mach

        # 1. Make sure the .dat file exists
        _ = self.write_dat_file

        dat_name   = f"{self.resolved_name}.dat"
        polar_path = self.polar_file_path
        dump_path  = self.dump_file_path

        # Remove stale output files so XFoil doesn't append
        for p in (polar_path, dump_path):
            if os.path.exists(p):
                os.remove(p)

        # 2. Build the XFoil command script
        #    - Paths are relative to _XFOIL_DIR (cwd for the subprocess)
        #    - PLOP G turns off the graphics window
        #    - VISC must come BEFORE MACH in OPER
        #    - Two blank PACC lines: first opens, second closes the polar file
        rel_dat   = os.path.join("Airfoils", dat_name)
        rel_polar = os.path.join("Airfoils", f"{self.resolved_name}_polar.txt")
        rel_dump  = os.path.join("Airfoils", f"{self.resolved_name}_dump.txt")

        script = "\n".join([
            "PLOP",
            "G",
            "",
            f"LOAD {rel_dat}",
            "PANE",
            "OPER",
            f"VISC {reynolds:.0f}",
            f"MACH {mach:.4f}",
            f"ITER {n_iter}",
            "PACC",
            rel_polar,
            rel_dump,
            f"ASEQ {alpha_start:.1f} {alpha_end:.1f} {alpha_step:.2f}",
            "PACC",
            "",
            "QUIT",
        ])

        # 3. Run
        try:
            result = subprocess.run(
                [_XFOIL_EXE],
                input=script,
                text=True,
                capture_output=True,
                cwd=_XFOIL_DIR,
                timeout=120,
            )
        except FileNotFoundError:
            raise RuntimeError(f"XFoil executable not found: {_XFOIL_EXE}")
        except subprocess.TimeoutExpired:
            raise RuntimeError("XFoil timed out after 120 s")

        if result.returncode != 0:
            print("XFoil stderr:\n", result.stderr)
            raise RuntimeError(f"XFoil exited with code {result.returncode}")

        # 4. Parse and return
        return self._parse_polar(polar_path)

    # ------------------------------------------------------------------ #
    # POLAR PARSING
    # ------------------------------------------------------------------ #

    @staticmethod
    def _parse_polar(polar_path: str):
        """
        Read an XFoil polar file and return (alphas, cls).

        XFoil polar header looks like:
          alpha    CL        CD       CDp       CM     Top_Xtr  Bot_Xtr
          ------  -------   -------  -------  -------  -------  -------
        Data starts after the dashed separator line.
        """
        alphas, cls = [], []

        if not os.path.exists(polar_path):
            print(f"WARNING: polar file not found: {polar_path}")
            return alphas, cls

        with open(polar_path) as fh:
            past_header = False
            for line in fh:
                s = line.strip()
                # The separator line looks like "------  -------  ..."
                if not past_header:
                    if s.startswith("---"):
                        past_header = True
                    continue

                if not s:
                    continue

                parts = s.split()
                if len(parts) >= 2:
                    try:
                        alphas.append(float(parts[0]))
                        cls.append(float(parts[1]))
                    except ValueError:
                        pass

        print(f"Parsed {len(alphas)} polar points from {polar_path}")
        return alphas, cls

    # ------------------------------------------------------------------ #
    # PLOT  (reads saved polar file)
    # ------------------------------------------------------------------ #

    @action(label="Plot XFoil CL-alpha curve")
    def plot_cl_alpha(self):
        """Plot CL-alpha curve with cruise (input α) and stall (from data)."""

        self.run_xfoil()
        alphas, cls = self._parse_polar(self.polar_file_path)

        if not alphas:
            print("No polar data found – run XFoil first.")
            return

        alphas = np.array(alphas)
        cls = np.array(cls)

        fig, ax = plt.subplots(figsize=(7, 5))

        # ------------------------------------------------------------
        # MAIN CURVE
        # ------------------------------------------------------------
        ax.plot(alphas, cls, marker="o", linewidth=2, markersize=4, label="XFoil")

        # ------------------------------------------------------------
        # CRUISE POINT (FROM INPUT α → INTERPOLATE CL)
        # ------------------------------------------------------------
        cruise_alpha = None
        cruise_cl = None

        if self.alpha_cruise is not None:

            # clamp within valid range to avoid extrapolation weirdness
            alpha_min, alpha_max = alphas.min(), alphas.max()
            alpha_target = np.clip(self.alpha_cruise, alpha_min, alpha_max)

            cruise_cl = np.interp(alpha_target, alphas, cls)
            cruise_alpha = alpha_target

            ax.scatter(
                cruise_alpha,
                cruise_cl,
                color="green",
                s=90,
                zorder=5,
                label="Cruise"
            )

            ax.annotate(
                f"Cruise\nα={cruise_alpha:.2f}°\nCl={cruise_cl:.3f}",
                (cruise_alpha, cruise_cl),
                textcoords="offset points",
                xytext=(10, 10)
            )

        # ------------------------------------------------------------
        # STALL POINT (FROM MAX CL)
        # ------------------------------------------------------------
        idx_max = np.argmax(cls)
        alpha_stall = alphas[idx_max]
        cl_max = cls[idx_max]

        ax.scatter(
            alpha_stall,
            cl_max,
            color="red",
            s=90,
            zorder=5,
            label="Stall (Clmax)"
        )

        ax.annotate(
            f"Stall\nα={alpha_stall:.2f}°\nCl={cl_max:.3f}",
            (alpha_stall, cl_max),
            textcoords="offset points",
            xytext=(10, -15)
        )

        # ------------------------------------------------------------
        # AXES
        # ------------------------------------------------------------
        ax.set_xlabel("α [deg]")
        ax.set_ylabel("$C_l$ [-]")
        ax.set_title(f"XFoil polar – {self.resolved_name.upper()}")
        ax.grid(True, linestyle="--", alpha=0.6)
        ax.legend()

        plt.tight_layout()
        plt.show()

    # ------------------------------------------------------------------ #
    # PARAPY GEOMETRY
    # ------------------------------------------------------------------ #

    @Attribute
    def raw_points(self):
        coords = self.normalized_coordinates
        return [
            self.position.translate("x", x * self.chord, "z", z * self.chord)
            for x, z in coords
        ]

    @Part
    def geometry(self):
        return FittedCurve(
            points=self.raw_points,
            mesh_deflection=self.mesh_deflection,
        )

    @Part
    def frame(self):
        return Frame(pos=self.position, hidden=False)


# ---------------------------------------------------------------------- #
# QUICK TEST
# ---------------------------------------------------------------------- #

if __name__ == "__main__":
    from parapy.gui import display

    # Auto-named: files saved as naca2412.dat / naca2412_polar.txt
    af = Airfoil(
        label="test_airfoil",
        chord=1.0,
        maximum_camber=0.04,
        camber_position=0.4,
        thickness_to_chord=0.12,
        mach=0.1,
        reynolds=1_500_000,
        export_dat=True,
        alpha_cruise=4,
    )

    # Explicit name: files saved as root_airfoil_geometric.dat / ..._polar.txt
    # af = Airfoil(
    #     label="root_airfoil",
    #     chord=2.5,
    #     maximum_camber=0.04,
    #     camber_position=0.4,
    #     thickness_to_chord=0.12,
    #     thickness_factor=1.0,
    #     export_dat=True,
    #     airfoil_name="root_airfoil_geometric",
    #     mach=0.78,
    #     reynolds=3_000_000,
    # )

    display(af)