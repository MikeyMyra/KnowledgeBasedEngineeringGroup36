"""
vn_diagram.py
=============
Maneuver V-n diagram for UAV initial sizing.

Usage
-----
Run standalone with defaults:
    python vn_diagram.py

Or import and call from Drone:
    from vn_diagram import plot_vn_diagram
    plot_vn_diagram(
        MTOW=self.MTOW,
        wing_area=self.wing_area,
        rho=self.air_density,
        n_pos=self.maximum_load_factor,
    )

References
----------
Raymer §V.2, Roskam Vol. V §4.2, EASA CS-LUAS ACJ VLA 333.
"""

import datetime
import glob
import math
import os
import shutil

import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D


# ── default parameters (edit or override via function arguments) ──────────── #

DEFAULTS = dict(
    MTOW        = 500.0,   # [kg]   maximum take-off mass
    wing_area   = 8.5,     # [m²]   reference wing area
    rho         = 1.225,   # [kg/m³] air density at cruise altitude
    n_pos       = 3.5,     # [–]    positive limit load factor
    n_neg       = -1.5,    # [–]    negative limit load factor  (negative value)
    CLmax_pos   = 1.4,     # [–]    maximum lift coefficient (positive stall)
    CLmax_neg   = 1.0,     # [–]    maximum lift coefficient (negative stall, |value|)
    Vc_factor   = 1.35,    # [–]    Vc = Vc_factor * Va  (Raymer heuristic)
    Vd_factor   = 1.20,    # [–]    Vd = Vd_factor * Vc  (EASA CS-LUAS)
    output_dir  = None,    # str or None — directory for timestamped PNG output
)


def compute_speeds(MTOW, wing_area, rho, n_pos, n_neg,
                   CLmax_pos, CLmax_neg, Vc_factor, Vd_factor):
    """Return dict of all characteristic speeds and wing loading."""
    g   = 9.80665
    W   = MTOW * g          # [N]
    WoS = W / wing_area     # [N/m²] wing loading

    # stall speed at n=1
    Vs1     = math.sqrt(2.0 * WoS / (rho * CLmax_pos))
    Vs1_neg = math.sqrt(2.0 * WoS / (rho * CLmax_neg))

    # corner (maneuver) speeds
    Va     = Vs1     * math.sqrt(abs(n_pos))
    Va_neg = Vs1_neg * math.sqrt(abs(n_neg))

    # design cruise and dive speeds
    Vc = Vc_factor * Va
    Vd = Vd_factor * Vc

    return dict(Vs1=Vs1, Vs1_neg=Vs1_neg, Va=Va, Va_neg=Va_neg,
                Vc=Vc, Vd=Vd, WoS=WoS, W=W)


def _archive_previous(output_dir: str, pattern: str) -> None:
    """Move existing files matching *pattern* in *output_dir* to a data/ sub-folder."""
    archive_dir = os.path.join(output_dir, "data")
    os.makedirs(archive_dir, exist_ok=True)
    for existing in glob.glob(os.path.join(output_dir, pattern)):
        dest = os.path.join(archive_dir, os.path.basename(existing))
        shutil.move(existing, dest)


def plot_vn_diagram(
    MTOW       = DEFAULTS["MTOW"],
    wing_area  = DEFAULTS["wing_area"],
    rho        = DEFAULTS["rho"],
    n_pos      = DEFAULTS["n_pos"],
    n_neg      = DEFAULTS["n_neg"],
    CLmax_pos  = DEFAULTS["CLmax_pos"],
    CLmax_neg  = DEFAULTS["CLmax_neg"],
    Vc_factor  = DEFAULTS["Vc_factor"],
    Vd_factor  = DEFAULTS["Vd_factor"],
    output_dir = DEFAULTS["output_dir"],
):
    """
    Generate, display, and save a maneuver V-n diagram.

    Always calls plt.show(). If output_dir is given, also saves a timestamped
    PNG there (any previous vn_diagram_*.png is archived to output_dir/data/).

    Parameters
    ----------
    MTOW       : float      Maximum take-off mass [kg]
    wing_area  : float      Reference wing area [m²]
    rho        : float      Air density at cruise altitude [kg/m³]
    n_pos      : float      Positive limit load factor [–]
    n_neg      : float      Negative limit load factor (negative value) [–]
    CLmax_pos  : float      Max lift coefficient, positive stall [–]
    CLmax_neg  : float      Max lift coefficient, negative stall (magnitude) [–]
    Vc_factor  : float      Vc = Vc_factor × Va [–]
    Vd_factor  : float      Vd = Vd_factor × Vc [–]
    output_dir : str|None   Directory to save timestamped PNG. None = no file saved.
    """
    g  = 9.80665
    sp = compute_speeds(MTOW, wing_area, rho, n_pos, n_neg,
                        CLmax_pos, CLmax_neg, Vc_factor, Vd_factor)
    Vs1, Vs1_neg = sp["Vs1"], sp["Vs1_neg"]
    Va, Va_neg   = sp["Va"],  sp["Va_neg"]
    Vc, Vd       = sp["Vc"],  sp["Vd"]
    W, WoS       = sp["W"],   sp["WoS"]

    # ── stall parabolas ──────────────────────────────────────────────────── #
    def n_from_V(V_arr, CLmax, sign=1.0):
        return sign * (0.5 * rho * V_arr**2 * CLmax * wing_area) / W

    V_pos_stall = np.linspace(0.0, Va,     300)
    V_neg_stall = np.linspace(0.0, Va_neg, 300)
    n_pos_stall = n_from_V(V_pos_stall, CLmax_pos,  +1.0)
    n_neg_stall = n_from_V(V_neg_stall, CLmax_neg,  -1.0)

    # ── figure ───────────────────────────────────────────────────────────── #
    fig, ax = plt.subplots(figsize=(10, 6.5))
    fig.patch.set_facecolor("#f8f8f6")
    ax.set_facecolor("#f8f8f6")

    # stall boundaries
    ax.plot(V_pos_stall, n_pos_stall,
            color="#1a6eb5", lw=2.2, label=f"Positive stall  (CLmax = {CLmax_pos})")
    ax.plot(V_neg_stall, n_neg_stall,
            color="#c0392b", lw=2.2, label=f"Negative stall  (CLmax = {CLmax_neg})")

    # structural limit lines
    ax.plot([Va,     Vd], [n_pos, n_pos], color="#2c3e50", lw=1.6, ls="--", label="n+ structural limit")
    ax.plot([Va_neg, Vd], [n_neg, n_neg], color="#7f0000", lw=1.6, ls="--", label="n− structural limit")
    ax.plot([Vd,     Vd], [n_neg, n_pos], color="#2c3e50", lw=1.6, ls="--")

    # zero-lift line
    ax.axhline(0, color="#888", lw=0.8, ls="-", zorder=0)

    # ── speed annotation lines ───────────────────────────────────────────── #
    y_top   = n_pos * 1.22
    tick_kw = dict(lw=1.0, ls=":", zorder=2)

    def speed_tick(V, label, color):
        ax.axvline(V, color=color, alpha=0.55, **tick_kw)
        ax.text(V, y_top, label, color=color,
                ha="center", va="bottom", fontsize=8.5)

    speed_tick(Vs1, f"Vs₁\n{Vs1:.1f} m/s", "#27ae60")
    speed_tick(Va,  f"Va\n{Va:.1f} m/s",    "#e67e22")
    speed_tick(Vc,  f"Vc\n{Vc:.1f} m/s",    "#2980b9")
    speed_tick(Vd,  f"Vd\n{Vd:.1f} m/s",    "#922b21")

    # ── n-limit labels (right-hand side) ─────────────────────────────────── #
    ax.text(Vd * 1.01, n_pos, f"  n+ = {n_pos:.1f} g",
            va="center", fontsize=9, color="#2c3e50")
    ax.text(Vd * 1.01, n_neg, f"  n− = {n_neg:.1f} g",
            va="center", fontsize=9, color="#7f0000")

    # ── axes and labels ───────────────────────────────────────────────────── #
    ax.set_xlim(left=0, right=Vd * 1.15)
    ax.set_ylim(bottom=n_neg * 1.35, top=n_pos * 1.40)
    ax.set_xlabel("Equivalent airspeed  V  [m/s]", fontsize=11)
    ax.set_ylabel("Load factor  n  [g]", fontsize=11)
    ax.set_title("V-n Maneuver Diagram", fontsize=13, fontweight="bold", pad=10)
    ax.grid(True, color="#cccccc", lw=0.6, ls="-")
    ax.minorticks_on()
    ax.grid(True, which="minor", color="#e8e8e8", lw=0.3)

    # ── legend ────────────────────────────────────────────────────────────── #
    legend_elements = [
        Line2D([0], [0], color="#1a6eb5", lw=2.2, label=f"Positive stall  (CLmax+ = {CLmax_pos})"),
        Line2D([0], [0], color="#c0392b", lw=2.2, label=f"Negative stall  (CLmax− = {CLmax_neg})"),
        Line2D([0], [0], color="#2c3e50", lw=1.6, ls="--", label="n+ structural limit"),
        Line2D([0], [0], color="#7f0000", lw=1.6, ls="--", label="n− structural limit"),
    ]
    ax.legend(handles=legend_elements, loc="upper left", fontsize=8.5, framealpha=0.85)

    # ── info box — bottom left ────────────────────────────────────────────── #
    info = (
        f"MTOW = {MTOW:.0f} kg\n"
        f"S = {wing_area:.2f} m²\n"
        f"W/S = {WoS:.0f} N/m²\n"
        f"ρ = {rho:.4f} kg/m³"
    )
    ax.text(0.02, 0.03, info, transform=ax.transAxes,
            ha="left", va="bottom", fontsize=8.5,
            bbox=dict(boxstyle="round,pad=0.4", fc="white", ec="#aaa", alpha=0.85))

    plt.tight_layout()

    # ── save with timestamp (archive previous) ───────────────────────────── #
    if output_dir is not None:
        os.makedirs(output_dir, exist_ok=True)
        _archive_previous(output_dir, "vn_diagram_*.png")
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        png_path  = os.path.join(output_dir, f"vn_diagram_{timestamp}.png")
        fig.savefig(png_path, dpi=150, bbox_inches="tight")
        print(f"✓ V-n diagram saved: {png_path}")

    plt.show()

    return fig, ax


# ── print summary to terminal ─────────────────────────────────────────────── #

def print_summary(**kwargs):
    p = {**DEFAULTS, **kwargs}
    sp = compute_speeds(
        p["MTOW"], p["wing_area"], p["rho"],
        p["n_pos"], p["n_neg"],
        p["CLmax_pos"], p["CLmax_neg"],
        p["Vc_factor"], p["Vd_factor"],
    )
    print("=" * 48)
    print("  V-n Diagram — Characteristic Speeds")
    print("=" * 48)
    print(f"  Wing loading  W/S : {sp['WoS']:>8.1f} N/m²")
    print(f"  Stall speed   Vs₁ : {sp['Vs1']:>8.2f} m/s  ({sp['Vs1']*3.6:.1f} km/h)")
    print(f"  Corner speed  Va  : {sp['Va']:>8.2f} m/s  ({sp['Va']*3.6:.1f} km/h)")
    print(f"  Cruise speed  Vc  : {sp['Vc']:>8.2f} m/s  ({sp['Vc']*3.6:.1f} km/h)")
    print(f"  Dive speed    Vd  : {sp['Vd']:>8.2f} m/s  ({sp['Vd']*3.6:.1f} km/h)")
    print(f"  n+  limit         : {p['n_pos']:>8.1f} g")
    print(f"  n−  limit         : {p['n_neg']:>8.1f} g")
    print("=" * 48)


# ── standalone entry point ────────────────────────────────────────────────── #

if __name__ == "__main__":
    print_summary()
    # saves to an Outputfiles/ folder next to this script when run directly
    _here = os.path.dirname(os.path.abspath(__file__))
    plot_vn_diagram(output_dir=os.path.join(_here, "Outputfiles"))