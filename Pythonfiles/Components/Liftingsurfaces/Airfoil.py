"""
Airfoil.py — Parametric NACA 4-series airfoil with CST fitting and XFoil polars.

Generates airfoil coordinates from camber/thickness inputs, fits a CST curve
for ParaPy geometry, writes a .dat file, and drives the bundled XFoil 6.99
executable to compute Cl/Cd polars. Polars are cached; Mach is capped at 0.70.
"""
import os
import subprocess
from math import comb

import numpy as np
import matplotlib.pyplot as plt

from parapy.core import Attribute, Input, Part, action
from parapy.geom import FittedCurve, GeomBase

from Pythonfiles.Components.Frame import Frame

_XFOIL_DIR  = os.path.abspath("XFOIL6.99")
_AIRFOIL_DIR = os.path.join("Inputfiles", "Airfoils")
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

    @Attribute
    def _naca_name(self):
        """Auto-derived NACA 4-digit string, e.g. 'naca2412'."""
        m = round(self.maximum_camber * 100)
        p = round(self.camber_position * 10)
        t = round(self.thickness_to_chord * 100)
        return f"naca{m}{p}{t:02d}"

    airfoil_name: str = Input(None)   # None → falls back to _naca_name / dat filename
    
    dat_path_override: str = Input(None)

    # ------------------------------------------------------------------ #
    # DERIVED NAMES / PATHS
    # ------------------------------------------------------------------ #

    @Attribute
    def resolved_name(self):
        """The name actually used for all file I/O."""
        if self.airfoil_name is not None:
            return self.airfoil_name
        if self.dat_path_override is not None:
            # Derive from the dat filename: "Inputfiles/Airfoils/naca2412.dat" → "naca2412"
            return os.path.splitext(os.path.basename(self.dat_path_override))[0]
        return self._naca_name

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

    @staticmethod
    def _read_dat_coordinates(path: str):
        coords = []
        with open(path) as fh:
            for i, line in enumerate(fh):
                if i == 0:          # name / header row
                    continue
                parts = line.strip().split()
                if len(parts) >= 2:
                    try:
                        coords.append([float(parts[0]), float(parts[1])])
                    except ValueError:
                        pass
        if not coords:
            raise RuntimeError(f"[Airfoil] No coordinates found in '{path}'.")
        return np.array(coords)

    @Attribute
    def normalized_coordinates(self):
        if self.dat_path_override is not None:
            if os.path.exists(self.dat_path_override):
                return self._read_dat_coordinates(self.dat_path_override)
            print(f"[Airfoil] WARNING: dat_path_override '{self.dat_path_override}' "
                  f"not found — falling back to computed NACA geometry.")

        # Compute from NACA 4-series parameters
        x     = self.x_distribution
        yt    = self.thickness_distribution * self.thickness_factor
        yc, _ = self.camber_line

        xu = x;  zu = yc + yt
        xl = x;  zl = yc - yt

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

    XFOIL_MACH_MAX: float = 0.70 # Mach is capped at ``XFOIL_MACH_MAX`` (0.70) — Xfoil's panel method diverges for transonic flows and returns no data above that limit.

    def run_xfoil(
        self,
        reynolds:    float = None,
        mach:        float = None,
        alpha_start: float = -4.0,
        alpha_end:   float = 20.0,
        alpha_step:  float = 0.5,
        n_iter:      int   = 200,
    ):
        
        reynolds     = reynolds if reynolds is not None else self.reynolds
        mach_actual  = mach     if mach     is not None else self.mach
        if mach_actual > self.XFOIL_MACH_MAX:
            print(f"[Xfoil] Mach {mach_actual:.3f} exceeds XFoil validity limit "
                  f"(M = {self.XFOIL_MACH_MAX}). Aborting XFoil run.")
            try:
                import tkinter as tk
                from tkinter import messagebox
                _root = tk.Tk()
                _root.withdraw()
                messagebox.showwarning(
                    "XFoil Mach Limit Exceeded",
                    f"Cruise Mach {mach_actual:.3f} exceeds XFoil's valid range "
                    f"(M ≤ {self.XFOIL_MACH_MAX}).\n\n"
                    f"XFoil cannot produce reliable results at transonic speeds. "
                    f"No polar will be generated.",
                )
                _root.destroy()
            except Exception:
                pass
            return [], [], [], []   # abort — do not run XFoil
        mach = mach_actual

        # 1. Write the .dat
        xfoil_airfoil_dir = os.path.join(_XFOIL_DIR, "Airfoils")
        os.makedirs(xfoil_airfoil_dir, exist_ok=True)
        dat_name   = f"{self.resolved_name}.dat"
        dat_path   = os.path.join(xfoil_airfoil_dir, dat_name)
        with open(dat_path, "w") as _df:
            _df.write(f"{self.resolved_name}\n")
            for _x, _z in self.normalized_coordinates:
                _df.write(f"{_x:.6f} {_z:.6f}\n")
        print(f"[XFoil] DAT written → {dat_path}")

        polar_path = os.path.join(xfoil_airfoil_dir, f"{self.resolved_name}_polar.txt")
        dump_path  = os.path.join(xfoil_airfoil_dir, f"{self.resolved_name}_dump.txt")

        # Remove stale output files so XFoil doesn't append
        for p in (polar_path, dump_path):
            if os.path.exists(p):
                os.remove(p)

        # 2. Build the XFoil command script
        rel_dat   = "Airfoils/" + dat_name
        rel_polar = "Airfoils/" + f"{self.resolved_name}_polar.txt"
        rel_dump  = "Airfoils/" + f"{self.resolved_name}_dump.txt"

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
    # POLAR FIGURE BUILDER
    # ------------------------------------------------------------------ #

    def _build_polar_figure(self):
        """
        Run XFoil, build and return the 4-panel polar figure (without showing
        or saving it).  Returns None if XFoil produced no data.
        """
        alphas, cls, cds, cms = self.run_xfoil()

        if not alphas:
            print("No polar data – XFoil produced no output.")
            # Only show the "diverged" popup when Mach was within the valid range;
            # if Mach was the reason run_xfoil aborted, that popup already fired.
            if self.mach <= self.XFOIL_MACH_MAX:
                try:
                    import tkinter as tk
                    from tkinter import messagebox
                    _root = tk.Tk()
                    _root.withdraw()
                    messagebox.showwarning(
                        "XFoil Diverged",
                        f"XFoil produced no polar data for "
                        f"{self.resolved_name.upper()}.\n\n"
                        f"The viscous solver likely diverged — this commonly "
                        f"happens with high-camber or very thin airfoils "
                        f"(e.g. camber > 6%).\n\n"
                        f"Try reducing the camber range in the airfoil sweep inputs.",
                    )
                    _root.destroy()
                except Exception:
                    pass
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
        ret