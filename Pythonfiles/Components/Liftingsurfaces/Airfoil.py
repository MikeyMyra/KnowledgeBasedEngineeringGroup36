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

        # 4. Parse and return (alphas, cls, cds, cms)
        return self._parse_polar(polar_path)

    # ------------------------------------------------------------------ #
    # POLAR PARSING
    # ------------------------------------------------------------------ #

    @staticmethod
    def _parse_polar(polar_path: str):
        """
        Read an XFoil polar file and return (alphas, cls, cds, cms).

        XFoil polar column layout:
          alpha    CL        CD       CDp       CM     Top_Xtr  Bot_Xtr
          ------  -------   -------  -------  -------  -------  -------
        Indices:  [0]       [1]      [2]      [3]      [4]
        Data starts after the dashed separator line.
        """
        alphas, cls, cds, cms = [], [], [], []

        if not os.path.exists(polar_path):
            print(f"WARNING: polar file not found: {polar_path}")
            return alphas, cls, cds, cms

        with open(polar_path) as fh:
            past_header = False
            for line in fh:
                s = line.strip()
                if not past_header:
                    if s.startswith("---"):
                        past_header = True
                    continue
                if not s:
                    continue
                parts = s.split()
                if len(parts) >= 5:
                    try:
                        alphas.append(float(parts[0]))
                        cls.append(float(parts[1]))
                        cds.append(float(parts[2]))
                        cms.append(float(parts[4]))
                    except ValueError:
                        pass

        print(f"Parsed {len(alphas)} polar points from {polar_path}")
        return alphas, cls, cds, cms

    # ------------------------------------------------------------------ #
    # PLOT  (reads saved polar file)
    # ------------------------------------------------------------------ #

    # ------------------------------------------------------------------ #
    # POLAR FIGURE BUILDER  (shared by action and save method)
    # ------------------------------------------------------------------ #

    def _build_polar_figure(self):
        """
        Run XFoil, build and return the 4-panel polar figure (without showing
        or saving it).  Returns None if XFoil produced no data.
        """
        alphas, cls, cds, cms = self.run_xfoil()

        if not alphas:
            print("No polar data – XFoil produced no output.")
            return None

        alphas = np.array(alphas)
        cls    = np.array(cls)
        cds    = np.array(cds)
        cms    = np.array(cms)

        with np.errstate(divide="ignore", invalid="ignore"):
            ld = np.where(cds > 1e-10, cls / cds, 0.0)

        # Cruise interpolation
        cruise_alpha = None
        has_cruise = (self.alpha_cruise is not None and
                      not np.isnan(float(self.alpha_cruise)))
        if has_cruise:
            cruise_alpha = float(np.clip(self.alpha_cruise,
                                         alphas.min(), alphas.max()))
            cruise_cl = float(np.interp(cruise_alpha, alphas, cls))
            cruise_cd = float(np.interp(cruise_alpha, alphas, cds))
            cruise_cm = float(np.interp(cruise_alpha, alphas, cms))
            cruise_ld = cruise_cl / cruise_cd if cruise_cd > 1e-10 else 0.0

        # Stall point
        idx_stall   = int(np.argmax(cls))
        alpha_stall = float(alphas[idx_stall])
        cl_stall    = float(cls[idx_stall])

        fig, axes = plt.subplots(2, 2, figsize=(12, 9))
        fig.suptitle(
            f"XFoil polars – {self.resolved_name.upper()}"
            f"   (Re = {self.reynolds:.2e}, M = {self.mach:.3f})",
            fontsize=13,
        )

        panels = [
            (axes[0, 0], cls,  r"$C_l$ [–]",       "CL – α"),
            (axes[0, 1], cds,  r"$C_d$ [–]",       "CD – α"),
            (axes[1, 0], cms,  r"$C_m$ [–]",       "Cm – α"),
            (axes[1, 1], ld,   r"$C_l/C_d$ [–]",   "L/D – α"),
        ]
        cruise_vals = (
            [cruise_cl, cruise_cd, cruise_cm, cruise_ld]
            if has_cruise else [None] * 4
        )

        for (ax, ydata, ylabel, title), c_val in zip(panels, cruise_vals):
            ax.plot(alphas, ydata,
                    color="steelblue", linewidth=2,
                    marker="o", markersize=3, label="XFoil")

            if ydata is cls:
                ax.scatter(alpha_stall, cl_stall,
                           color="red", s=80, zorder=5, label="Stall (max CL)")
                ax.annotate(
                    f"α={alpha_stall:.1f}°\nCl={cl_stall:.3f}",
                    (alpha_stall, cl_stall),
                    textcoords="offset points", xytext=(8, -18),
                    fontsize=8, color="red",
                )

            if has_cruise and c_val is not None:
                ax.scatter(cruise_alpha, c_val,
                           color="green", s=90, zorder=6,
                           label=f"Cruise  α={cruise_alpha:.2f}°")
                ax.axvline(cruise_alpha, color="green",
                           linestyle="--", linewidth=0.8, alpha=0.5)
                ax.annotate(
                    f"α={cruise_alpha:.2f}°\n{c_val:.4f}",
                    (cruise_alpha, c_val),
                    textcoords="offset points", xytext=(8, 8),
                    fontsize=8, color="darkgreen",
                )

            ax.axhline(0, color="black", linewidth=0.6, linestyle=":")
            ax.set_xlabel("α [deg]")
            ax.set_ylabel(ylabel)
            ax.set_title(title)
            ax.grid(True, linestyle="--", alpha=0.5)
            ax.legend(fontsize=8)

        plt.tight_layout()
        return fig

    @action(label="Plot XFoil polars")
    def plot_cl_alpha(self):
        """
        Run XFoil and display the 4-panel polar figure:
          top-left   CL – α  (cruise & stall markers)
          top-right  CD – α
          bottom-left  Cm – α
          bottom-right  L/D – α
        """
        fig = self._build_polar_figure()
        if fig is not None:
            plt.show()

    def save_polar_figure(self, save_path: str):
        """Save the 4-panel XFoil polar figure to *save_path* without displaying it."""
        fig = self._build_polar_figure()
        if fig is not None:
            fig.savefig(save_path, dpi=150, bbox_inches='tight')
            plt.close(fig)
            print(f"Wing polars saved → {save_path}")

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