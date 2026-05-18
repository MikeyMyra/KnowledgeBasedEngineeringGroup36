from itertools import product
from math import radians, tan

import numpy as np

import matlab
#import matlab.engine

from parapy.core import Attribute, Input, Part, action
from parapy.geom import GeomBase, LoftedSolid, rotate, translate

from Pythonfiles.Components.Frame import Frame
from Pythonfiles.Components.Liftingsurfaces.Airfoil import Airfoil
from Pythonfiles.Components.Liftingsurfaces.Wingbox import Wingbox
from Pythonfiles.Matlab_start import MATLAB_Q3D_ENGINE


class LiftingSurface(GeomBase):
    """
    Parametric lifting surface (wing, horizontal tail, vertical tail).

    Airfoil selection
    -----------------
    Normal mode  (run_airfoil_sweep=False):
        maximum_camber_input / maximum_camber_position_input / thickness_to_chord_input
        are used directly.

    Sweep mode   (run_airfoil_sweep=True  OR  via the 'Run Airfoil Sweep' action):
        A Q3D sweep over camber_range × position_range × thickness_range picks
        the feasible NACA 4-series shape with the highest L/D margin above
        ld_required.  Results are stored in sweep_result_* and picked up
        automatically by the resolved Attributes below.

    Structural mass (Roskam Vol. I §8.4 / §8.5):
    - Wing : W_w  = 0.036 * S^0.758 * AR^0.6 * (t/c)^-0.3   (Roskam Table 8.1)
    - HT   : W_ht = 0.016 * S_ht^0.873 * AR_ht^0.357         (Roskam Table 8.1)
    - VT   : W_vt = 0.073 * S_vt^0.873 * AR_vt^0.357         (Roskam Table 8.1)
    CG of each surface at 40% MAC from the LE of MAC (Roskam §8.4 statistical).
    """

    # ------------------------------------------------------------------ #
    # FLIGHT CONDITIONS
    # ------------------------------------------------------------------ #

    weight:   float = Input(1000)
    velocity: float = Input(100)
    altitude: float = Input(10000)
    use_cl:   bool  = Input(True)
    alpha:    float = Input(0.0)
    maximum_load_factor: float = Input(1)

    # ------------------------------------------------------------------ #
    # PRIMARY SIZING
    # ------------------------------------------------------------------ #

    effective_area:      float  = Input(None)
    effective_semi_span: float  = Input(None)

    fuselage_length: float  = Input()
    fuselage_radius: float  = Input()

    fuselage_cone_radius_fn: object = Input(None)

    is_tail:          bool  = Input()
    is_vertical_tail: bool  = Input()

    mesh_deflection: float = Input(1e-4)

    color_wingbox:        str = Input()
    color_liftingsurface: str = Input()

    # ------------------------------------------------------------------ #
    # PLANFORM GEOMETRY
    # ------------------------------------------------------------------ #

    taper_ratio: float = Input()
    sweep_le:    float = Input()
    twist:       float = Input()
    dihedral:    float = Input()

    # ------------------------------------------------------------------ #
    # AIRFOIL PARAMETERS
    # ------------------------------------------------------------------ #

    maximum_camber_input:          float = Input(0.04)
    maximum_camber_position_input: float = Input(0.40)
    thickness_to_chord_input:      float = Input(0.12)

    # NACA 4-series code forwarded from Drone → Aircraft (e.g. "2412").
    # User interaction and validation live in Drone._parsed_wing_naca.
    # Priority (highest → lowest):
    #   1. sweep_result_* (set after run_sweep action)
    #   2. naca_input     (forwarded from Drone.wing_naca_input)
    #   3. maximum_camber_input / maximum_camber_position_input / thickness_to_chord_input
    naca_input: str = Input(None)

    # Path to the .dat file that the rendered airfoil Parts should read their
    # geometry from.  Set by Drone._active_wing_dat_path (default: naca0012.dat).
    # Sweep Airfoil instances (created inside best_airfoil_params) do NOT receive
    # this — they compute geometry from their explicit NACA params.
    active_dat_path: str = Input(None)

    sweep_result_camber:    float = Input(None)
    sweep_result_position:  float = Input(None)
    sweep_result_thickness: float = Input(None)

    pre_sweep_camber:    float = Input(None)
    pre_sweep_position:  float = Input(None)
    pre_sweep_thickness: float = Input(None)

    # ------------------------------------------------------------------ #
    # AIRFOIL SWEEP CONTROLS
    # ------------------------------------------------------------------ #

    run_airfoil_sweep: bool  = Input(False)
    ld_required:       float = Input(None)
    camber_range:    list = Input([0.01, 0.02, 0.03, 0.04])
    position_range:  list = Input([0.3, 0.4, 0.5, 0.6])
    thickness_range: list = Input([0.08, 0.10, 0.12, 0.14])
    g: float = Input(9.81)

    # ------------------------------------------------------------------ #
    # THICKNESS SCALING
    # ------------------------------------------------------------------ #

    t_factor_root: float = Input()
    t_factor_tip:  float = Input()

    # ------------------------------------------------------------------ #
    # SPAR POSITIONS
    # ------------------------------------------------------------------ #

    front_spar_position: float = Input()
    rear_spar_position:  float = Input()

    # ------------------------------------------------------------------ #
    # TAIL SIZING
    # ------------------------------------------------------------------ #

    tail_volume_coefficient_h: float = Input(None)
    tail_volume_coefficient_v: float = Input(None)
    tail_aspect_ratio_h:       float = Input(None)
    tail_aspect_ratio_v:       float = Input(None)

    wing_ref: "LiftingSurface" = Input(None)

    # ------------------------------------------------------------------ #
    # RESOLVED AIRFOIL PARAMETERS
    # ------------------------------------------------------------------ #

    @Attribute
    def _parsed_naca_input(self):
        """
        Parse the NACA 4-series string passed in via naca_input.

        Validation (with user-facing dialog) is handled upstream in Drone so
        that the error is surfaced at the top-level object.  Here we only do
        a silent parse: bad strings are logged and return None so the fallback
        camber/position/thickness inputs remain active without disrupting the
        geometry.

        Accepted formats: "2412", "NACA2412", "naca 2412" (spaces stripped).
        Returns dict {camber, position, thickness, naca_str} or None.
        """
        raw = self.naca_input
        if raw is None or str(raw).strip() == "":
            return None

        s = str(raw).strip().lower().replace(" ", "")
        if s.startswith("naca"):
            s = s[4:]

        if len(s) != 4 or not s.isdigit() or int(s[0]) > 6 or int(s[2:4]) == 0:
            print(f"[LiftingSurface] Ignoring unparseable naca_input '{raw}'.")
            return None

        m = int(s[0]) / 100.0
        p = int(s[1]) / 10.0 if int(s[1]) > 0 else 0.4
        t = int(s[2:4]) / 100.0

        return {"camber": m, "position": p, "thickness": t, "naca_str": f"naca{s}"}

    @Attribute
    def maximum_camber(self):
        if self.sweep_result_camber is not None:
            return self.sweep_result_camber
        parsed = self._parsed_naca_input
        if parsed is not None:
            return parsed["camber"]
        return self.maximum_camber_input

    @Attribute
    def maximum_camber_position(self):
        if self.sweep_result_position is not None:
            return self.sweep_result_position
        parsed = self._parsed_naca_input
        if parsed is not None:
            return parsed["position"]
        return self.maximum_camber_position_input

    @Attribute
    def thickness_to_chord(self):
        if self.sweep_result_thickness is not None:
            return self.sweep_result_thickness
        parsed = self._parsed_naca_input
        if parsed is not None:
            return parsed["thickness"]
        return self.thickness_to_chord_input

    # ------------------------------------------------------------------ #
    # ACTION: AIRFOIL SWEEP
    # ------------------------------------------------------------------ #

    @action(label="Run Airfoil Sweep")
    def run_sweep(self):
        self.pre_sweep_camber    = self.maximum_camber_input
        self.pre_sweep_position  = self.maximum_camber_position_input
        self.pre_sweep_thickness = self.thickness_to_chord_input

        self.run_airfoil_sweep = True
        best = self.best_airfoil_params

        self.sweep_result_camber    = best["camber"]
        self.sweep_result_position  = best["position"]
        self.sweep_result_thickness = best["thickness"]
        self.alpha = best["alpha"]

        # Overwrite naca_input with the found airfoil so the input field
        # always reflects the active airfoil after a sweep.
        m_d = round(best["camber"]    * 100)
        p_d = round(best["position"]  * 10)
        t_d = round(best["thickness"] * 100)
        self.naca_input = f"{m_d}{p_d}{t_d:02d}"
        print(f"[Sweep] naca_input set to '{self.naca_input}'")

    @Attribute
    def best_airfoil_params(self):
        if not self.run_airfoil_sweep:
            raise RuntimeError("best_airfoil_params called but run_airfoil_sweep=False.")
        if self.ld_required is None:
            raise ValueError("ld_required must be set when run_airfoil_sweep=True.")

        combos = list(product(self.camber_range, self.position_range, self.thickness_range))
        n = len(combos)
        print(f"\nAirfoil sweep: {n} Q3D evaluations")
        print(f"  Required L/D : {self.ld_required}")
        print(f"  Target CL    : {self.target_cl:.4f}")

        results = []
        for i, (m, p, t) in enumerate(combos):
            print(f"  [{i+1}/{n}]  camber={m:.3f}  pos={p:.2f}  t/c={t:.3f}", end="  ")
            safe_p = p if (m > 0.0 and p > 0.0) else 0.4
            try:
                root_af = Airfoil(
                    chord=self.c_root_aero, maximum_camber=m, camber_position=safe_p,
                    thickness_to_chord=t, thickness_factor=self.t_factor_root,
                    export_dat=False, airfoil_name=f"_sweep_root_{i}",
                    position=self._root_position,
                )
                tip_af = Airfoil(
                    chord=self.c_tip, maximum_camber=m, camber_position=safe_p,
                    thickness_to_chord=t, thickness_factor=self.t_factor_tip,
                    export_dat=False, airfoil_name=f"_sweep_tip_{i}",
                    position=self._tip_position,
                )
                af_matrix = matlab.double(
                    np.vstack([root_af.CST_vector, tip_af.CST_vector]).tolist()
                )
                Res, _ = MATLAB_Q3D_ENGINE.run_q3d_cst(
                    self._q3d_planform_matrix, af_matrix,
                    matlab.double([0.0]),
                    matlab.double([self.mach]),
                    matlab.double([self.reynolds]),
                    matlab.double([self.velocity]),
                    matlab.double([self.target_cl]),
                    matlab.double([self.altitude]),
                    matlab.double([self.density]),
                    matlab.logical([True]),
                    matlab.logical([False]),
                    nargout=2,
                )

                if i == 0:
                    print(f"\n  [DEBUG] Res keys: {list(Res.keys())}\n")

                cl    = float(Res["CLwing"])
                cd    = float(Res["CDiwing"])
                cm    = float(Res["CMwing"])
                alpha = float(Res["Alpha"])
                ld    = cl / cd if cd > 1e-10 else 0.0
                feasible = ld >= self.ld_required

                print(f"L/D={ld:.2f}  CL={cl:.4f}  Cm={cm:.4f}  "
                      f"{'FEASIBLE' if feasible else 'FAIL'}")

                results.append({
                    "camber": m, "position": p, "thickness": t,
                    "CL": cl, "CD": cd, "Cm": cm, "L/D": ld,
                    "L/D_margin": ld - self.ld_required,
                    "feasible": feasible, "ld_met": feasible, "alpha": alpha
                })
            except Exception as e:
                import traceback
                print(f"FAILED: {e}")
                traceback.print_exc()

        feasible_results = [r for r in results if r["feasible"]]
        if feasible_results:
            best = max(feasible_results, key=lambda r: r["L/D_margin"])
            print(f"\nBest airfoil: camber={best['camber']:.3f}  "
                  f"pos={best['position']:.2f}  t/c={best['thickness']:.3f}  "
                  f"L/D={best['L/D']:.2f}  margin=+{best['L/D_margin']:.2f}  "
                  f"alpha={best['alpha']:.2f}°")
            return best

        raise ValueError(
            f"Airfoil sweep found no feasible candidate. "
            f"Check ld_required={self.ld_required}."
        )

    # ------------------------------------------------------------------ #
    # ATTACH POSITION
    # ------------------------------------------------------------------ #

    @Attribute
    def attach_x(self):
        if self.is_tail:
            return min(self.wing_ref.attach_x + self.tail_arm,
                       self.fuselage_length - self.c_root_aero)
        return 0.40 * self.fuselage_length

    @Attribute
    def attach_z(self):
        return self.fuselage_wall_radius if (self.is_tail and self.is_vertical_tail) else 0.0

    @Attribute
    def tail_arm(self):
        if not self.is_tail:
            return None
        wing_x  = self.wing_ref.attach_x
        max_arm = self.fuselage_length - wing_x
        return min(0.65 * self.wing_ref._effective_span * 2, max_arm)

    @Attribute
    def fuselage_wall_radius(self):
        if self.is_tail and self.fuselage_cone_radius_fn is not None:
            return self.fuselage_cone_radius_fn(self.attach_x + self.c_root_aero)
        return self.fuselage_radius

    # ------------------------------------------------------------------ #
    # EFFECTIVE AREA AND SPAN
    # ------------------------------------------------------------------ #

    @Attribute
    def _effective_area(self):
        if not self.is_tail:
            if self.effective_area is None:
                raise ValueError("Wing requires effective_area.")
            return self.effective_area

        if self.wing_ref is None:
            raise ValueError("Tail requires wing_ref.")

        S_w = self.wing_ref._effective_area
        b_w = 2 * self.wing_ref._effective_span
        c_w = self.wing_ref.mean_aerodynamic_chord

        if self.is_vertical_tail:
            if None in (self.tail_volume_coefficient_v, self.tail_aspect_ratio_v):
                raise ValueError("VT requires tail_volume_coefficient_v and tail_aspect_ratio_v.")
            return (self.tail_volume_coefficient_v * S_w * b_w) / self.tail_arm
        else:
            if None in (self.tail_volume_coefficient_h, self.tail_aspect_ratio_h):
                raise ValueError("HT requires tail_volume_coefficient_h and tail_aspect_ratio_h.")
            return (self.tail_volume_coefficient_h * S_w * c_w) / self.tail_arm

    @Attribute
    def _effective_span(self):
        if not self.is_tail:
            if self.effective_semi_span is None:
                raise ValueError("Wing requires effective_semi_span.")
            return self.effective_semi_span
        if self.is_vertical_tail:
            return np.sqrt(self.tail_aspect_ratio_v * self._effective_area)
        return np.sqrt(self.tail_aspect_ratio_h * self._effective_area) / 2

    # ------------------------------------------------------------------ #
    # CHORD
    # ------------------------------------------------------------------ #

    @Attribute
    def c_root_aero(self):
        return self._effective_area / (self._effective_span * (1 + self.taper_ratio))

    @Attribute
    def c_tip(self):
        return self.c_root_aero * self.taper_ratio

    @Attribute
    def c_root_geometric(self):
        slope = (self.c_root_aero - self.c_tip) / self._effective_span
        return self.c_root_aero + slope * self.fuselage_wall_radius

    @Attribute
    def _geometric_span(self):
        return self._effective_span + self.fuselage_wall_radius

    # ------------------------------------------------------------------ #
    # DERIVED AERO
    # ------------------------------------------------------------------ #

    @Attribute
    def aspect_ratio(self):
        return (2 * self._effective_span)**2 / self._effective_area

    @Attribute
    def mean_aerodynamic_chord(self):
        tr = self.taper_ratio
        return (2 / 3) * self.c_root_aero * (1 + tr + tr**2) / (1 + tr)

    @Attribute
    def mac_spanwise_position(self):
        tr = self.taper_ratio
        return self._effective_span * (1 + 2 * tr) / (3 * (1 + tr))

    @Attribute
    def mac_x_offset(self):
        return self.mac_spanwise_position * tan(radians(self.sweep_le))

    # ------------------------------------------------------------------ #
    # AERODYNAMIC CENTRE x-position (from nose)
    #
    # Roskam Vol. II §3.2: x_ac = attach_x + LE sweep offset to MAC + 0.25 * MAC
    # The sweep offset brings us to the LE of the MAC spanwise station.
    # ------------------------------------------------------------------ #

    @Attribute
    def x_ac(self) -> float:
        """
        Aerodynamic centre x-position from aircraft nose [m].

        Roskam Vol. II §3.2: AC of a swept tapered wing lies at 25% MAC,
        measured from the leading edge of the MAC.
        x_ac = attach_x + mac_x_offset + 0.25 * MAC
        """
        return self.attach_x + self.mac_x_offset + 0.25 * self.mean_aerodynamic_chord

    # ------------------------------------------------------------------ #
    # STRUCTURAL MASS  (Roskam Vol. I §8.4 / §8.5)
    # ------------------------------------------------------------------ #

    @Attribute
    def calculate_mass(self) -> float:
        """
        Lifting surface structural mass [kg].

        Roskam Vol. I, Table 8.1:
        - Wing : W_w  = 0.036 * S_w^0.758 * AR_w^0.6  * (t/c)^-0.3
        - HT   : W_ht = 0.016 * S_ht^0.873 * AR_ht^0.357
        - VT   : W_vt = 0.073 * S_vt^0.873 * AR_vt^0.357

        S in [m²], AR dimensionless, t/c dimensionless.
        All three equations from Roskam Vol. I Table 8.1 UAV/homebuilt row.
        """
        S  = self._effective_area
        AR = self.aspect_ratio
        tc = self.thickness_to_chord

        if not self.is_tail:
            # Wing — thickness correction included (thicker wing = lighter structurally)
            return 0.036 * (S ** 0.758) * (AR ** 0.6) * (tc ** -0.3)
        elif self.is_vertical_tail:
            return 0.073 * (S ** 0.873) * (AR ** 0.357)
        else:
            return 0.016 * (S ** 0.873) * (AR ** 0.357)

    @Attribute
    def cg_x(self) -> float:
        """
        Structural CG x-position from aircraft nose [m].

        Roskam Vol. I §8.4: wing/tail structural mass centroid at 40% MAC
        from the leading edge of the MAC — consistent with the wingbox
        spanning ~15–60% chord (front_spar to rear_spar midpoint ≈ 38%).
        """
        return self.attach_x + self.mac_x_offset + 0.40 * self.mean_aerodynamic_chord

    # ------------------------------------------------------------------ #
    # POSITIONING HELPERS
    # ------------------------------------------------------------------ #

    def _spanwise_offsets(self, y_sign: float):
        R  = self.fuselage_wall_radius
        return (R * tan(radians(self.sweep_le)),
                y_sign * R,
                R * np.sin(radians(self.dihedral)))

    @Attribute
    def _root_position_wingbox(self):
        base = translate(self.position, "x", self.attach_x, "y", 0.0, "z", self.attach_z)
        return rotate(base, "x", radians(90)) if self.is_vertical_tail else base

    @Attribute
    def _root_position(self):
        if not self.is_vertical_tail:
            dx, dy, dz = self._spanwise_offsets(+1)
            return translate(self.position,
                             "x", self.attach_x + dx, "y", dy, "z", self.attach_z + dz)
        R  = self.fuselage_wall_radius
        dx = R * tan(radians(self.sweep_le))
        base = translate(self.position, "x", self.attach_x + dx, "z", self.attach_z + R)
        return rotate(base, "x", radians(90))

    @Attribute
    def _root_position_mirrored(self):
        if self.is_vertical_tail:
            return self._root_position
        dx, dy, dz = self._spanwise_offsets(-1)
        return translate(self.position,
                         "x", self.attach_x + dx, "y", dy, "z", self.attach_z + dz)

    @Attribute
    def _tip_position(self):
        s = self._effective_span
        if not self.is_vertical_tail:
            return rotate(
                translate(self._root_position,
                          "y", s,
                          "x", s * tan(radians(self.sweep_le)),
                          "z", s * np.sin(radians(self.dihedral))),
                "y", radians(self.twist),
            )
        return rotate(
            translate(self._root_position,
                      "y", s,
                      "x", s * tan(radians(self.sweep_le))),
            "z", radians(self.twist),
        )

    @Attribute
    def _tip_position_mirrored(self):
        if self.is_vertical_tail:
            return None
        s = self._effective_span
        return rotate(
            translate(self._root_position_mirrored,
                      "y", -s,
                      "x",  s * tan(radians(self.sweep_le)),
                      "z",  s * np.sin(radians(self.dihedral))),
            "y", radians(self.twist),
        )

    # ------------------------------------------------------------------ #
    # AIRFOIL PARTS
    # ------------------------------------------------------------------ #

    @Part
    def root_airfoil_wingbox(self):
        return Airfoil(
            chord=self.c_root_geometric,
            maximum_camber=self.maximum_camber,
            camber_position=self.maximum_camber_position,
            thickness_to_chord=self.thickness_to_chord,
            thickness_factor=self.t_factor_root,
            export_dat=True, airfoil_name="root_airfoil_geometric",
            dat_path_override=self.active_dat_path,
            position=self._root_position_wingbox,
            mach=self.mach,
            reynolds=self.reynolds,
            alpha_cruise=self.alpha,
        )

    @Part
    def root_airfoil(self):
        return Airfoil(
            chord=self.c_root_aero,
            maximum_camber=self.maximum_camber,
            camber_position=self.maximum_camber_position,
            thickness_to_chord=self.thickness_to_chord,
            thickness_factor=self.t_factor_root,
            export_dat=True, airfoil_name=None,  # resolved from dat filename (e.g. "naca2412")
            dat_path_override=self.active_dat_path,
            position=self._root_position,
            mach=self.mach,
            reynolds=self.reynolds,
            alpha_cruise=self.alpha,
        )

    @Part
    def root_airfoil_mirrored(self):
        return Airfoil(
            chord=self.c_root_aero,
            maximum_camber=self.maximum_camber,
            camber_position=self.maximum_camber_position,
            thickness_to_chord=self.thickness_to_chord,
            thickness_factor=self.t_factor_root,
            export_dat=True, airfoil_name="root_airfoil_aero_mirrored",
            dat_path_override=self.active_dat_path,
            position=self._root_position_mirrored,
            suppress=self.is_vertical_tail,
            mach=self.mach,
            reynolds=self.reynolds,
            alpha_cruise=self.alpha,
        )

    @Part
    def tip_airfoil(self):
        return Airfoil(
            chord=self.c_tip,
            maximum_camber=self.maximum_camber,
            camber_position=self.maximum_camber_position,
            thickness_to_chord=self.thickness_to_chord,
            thickness_factor=self.t_factor_tip,
            export_dat=True, airfoil_name="tip_airfoil",
            dat_path_override=self.active_dat_path,
            position=self._tip_position,
            mach=self.mach,
            reynolds=self.reynolds,
            alpha_cruise=self.alpha,
        )

    @Part
    def tip_airfoil_mirrored(self):
        return Airfoil(
            chord=self.c_tip,
            maximum_camber=self.maximum_camber,
            camber_position=self.maximum_camber_position,
            thickness_to_chord=self.thickness_to_chord,
            thickness_factor=self.t_factor_tip,
            export_dat=True, airfoil_name="tip_airfoil_mirrored",
            dat_path_override=self.active_dat_path,
            position=self._tip_position_mirrored,
            suppress=self.is_vertical_tail,
            mach=self.mach,
            reynolds=self.reynolds,
            alpha_cruise=self.alpha,
        )

    # ------------------------------------------------------------------ #
    # SOLIDS
    # ------------------------------------------------------------------ #

    @Part
    def solid(self):
        return LoftedSolid(
            profiles=[self.root_airfoil.geometry, self.tip_airfoil.geometry],
            color=self.color_liftingsurface,
            mesh_deflection=self.mesh_deflection,
        )

    @Part
    def solid_mirrored(self):
        return LoftedSolid(
            profiles=[self.root_airfoil_mirrored.geometry, self.tip_airfoil_mirrored.geometry],
            color=self.color_liftingsurface,
            transparency=0.6,
            mesh_deflection=self.mesh_deflection,
            suppress=self.is_vertical_tail,
        )

    # ------------------------------------------------------------------ #
    # FRAME & WINGBOX
    # ------------------------------------------------------------------ #

    @Part
    def frame(self):
        return Frame(pos=self._root_position, hidden=False)

    @Part
    def wingbox(self):
        return Wingbox(
            c_root=self.c_root_geometric, c_tip=self.c_tip,
            semi_span=self._geometric_span,
            sweep_le=self.sweep_le, dihedral=self.dihedral, twist=self.twist,
            front_spar_position=self.front_spar_position,
            rear_spar_position=self.rear_spar_position,
            airfoil_root=self.root_airfoil_wingbox, airfoil_tip=self.tip_airfoil,
            color=self.color_wingbox,
        )

    @Part
    def wingbox_mirrored(self):
        return Wingbox(
            c_root=self.c_root_geometric, c_tip=self.c_tip,
            semi_span=-self._geometric_span,
            sweep_le=-self.sweep_le, dihedral=-self.dihedral, twist=self.twist,
            front_spar_position=self.front_spar_position,
            rear_spar_position=self.rear_spar_position,
            airfoil_root=self.root_airfoil_wingbox, airfoil_tip=self.tip_airfoil_mirrored,
            color=self.color_wingbox,
            suppress=self.is_vertical_tail,
        )

    # ------------------------------------------------------------------ #
    # ISA ATMOSPHERE
    # ------------------------------------------------------------------ #

    @Attribute
    def _isa(self):
        T0, p0, L, R, gamma, g = 288.15, 101_325.0, 0.0065, 287.058, 1.4, 9.80665
        h = self.altitude
        if h <= 11_000:
            T = T0 - L * h
            p = p0 * (T / T0) ** (g / (L * R))
        else:
            T11 = T0 - L * 11_000
            p11 = p0 * (T11 / T0) ** (g / (L * R))
            T   = T11
            p   = p11 * np.exp(-g * (h - 11_000) / (R * T11))
        rho = p / (R * T)
        a   = np.sqrt(gamma * R * T)
        return {"T": T, "p": p, "rho": rho, "a": a}

    @Attribute
    def density(self):
        return self._isa["rho"]

    @Attribute
    def mach(self):
        return self.velocity / self._isa["a"]

    @Attribute
    def reynolds(self):
        T  = self._isa["T"]
        mu = 1.716e-5 * (T / 273.15)**1.5 * (273.15 + 110.4) / (T + 110.4)
        return self.density * self.velocity * self.mean_aerodynamic_chord / mu

    @Attribute
    def target_cl(self):
        return (2.0 * self.weight * self.maximum_load_factor * self.g) / (
            self.density * self.velocity**2 * self._effective_area)

    # ------------------------------------------------------------------ #
    # Q3D MATRICES
    # ------------------------------------------------------------------ #

    @Attribute
    def _q3d_airfoil_matrix(self):
        return matlab.double(
            np.vstack([self.root_airfoil.CST_vector, self.tip_airfoil.CST_vector]).tolist()
        )

    @Attribute
    def _q3d_planform_matrix(self):
        s    = self._effective_span
        x_le = s * tan(radians(self.sweep_le))
        z_tip = s * np.sin(radians(self.dihedral))
        return matlab.double([
            [0.0,   0.0, 0.0,   self.c_root_aero, 0.0       ],
            [x_le,  s,   z_tip, self.c_tip,        self.twist],
        ])

    @Attribute
    def q3d_data(self):
        alpha_or_cl = self.target_cl if self.use_cl else self.alpha
        return MATLAB_Q3D_ENGINE.run_q3d_cst(
            self._q3d_planform_matrix,
            self._q3d_airfoil_matrix,
            matlab.double([0.0]),
            matlab.double([self.mach]),
            matlab.double([self.reynolds]),
            matlab.double([self.velocity]),
            matlab.double([alpha_or_cl]),
            matlab.double([self.altitude]),
            matlab.double([self.density]),
            matlab.logical([self.use_cl]),
            matlab.logical([False]),
            nargout=2,
        )

    # ------------------------------------------------------------------ #
    # UI
    # ------------------------------------------------------------------ #

    @action(label="Plot XFoil polars")
    def plot_cl_alpha(self):
        self.root_airfoil.plot_cl_alpha()


# ---------------------------------------------------------------------- #
# TEST
# ---------------------------------------------------------------------- #

if __name__ == "__main__":
    from parapy.gui import display

    wing = LiftingSurface(
        label="main_wing",
        effective_area=18.0,  effective_semi_span=7.4,
        fuselage_length=10.0, fuselage_radius=0.6,
        is_tail=False, is_vertical_tail=False,
        mesh_deflection=1e-4,
        taper_ratio=0.40, sweep_le=5.0, twist=-2.0, dihedral=5.0,
        run_airfoil_sweep=False,
        ld_required=10.0,
        camber_range=[0.0, 0.02, 0.03, 0.04, 0.05, 0.06],
        position_range=[0.3, 0.35, 0.4, 0.45, 0.5],
        thickness_range=[0.08, 0.10, 0.12, 0.15],
        cm_min=-0.50, cm_max=0.00, t_min=0.10,
        weight=15_000, velocity=60.0, altitude=2_000,
        t_factor_root=1.0, t_factor_tip=1.0,
        front_spar_position=0.15, rear_spar_position=0.60,
        color_wingbox="yellow", color_liftingsurface="orange",
    )

    horizontal_tail = LiftingSurface(
        label="horizontal_tail",
        wing_ref=wing, is_tail=True, is_vertical_tail=False,
        tail_volume_coefficient_h=0.60, tail_volume_coefficient_v=0.04,
        tail_aspect_ratio_h=4.5,       tail_aspect_ratio_v=1.8,
        fuselage_length=10.0, fuselage_radius=0.6,
        mesh_deflection=1e-4,
        taper_ratio=0.40, sweep_le=10.0, twist=0.0, dihedral=0.0,
        run_airfoil_sweep=False,
        maximum_camber_input=0.0, maximum_camber_position_input=0.4,
        thickness_to_chord_input=0.12,
        weight=15_000, velocity=60.0, altitude=2_000,
        t_factor_root=1.0, t_factor_tip=1.0,
        front_spar_position=0.15, rear_spar_position=0.60,
        color_wingbox="red", color_liftingsurface="green",
    )

    vertical_tail = LiftingSurface(
        label="vertical_tail",
        wing_ref=wing, is_tail=True, is_vertical_tail=True,
        tail_volume_coefficient_h=0.60, tail_volume_coefficient_v=0.04,
        tail_aspect_ratio_h=4.5,        tail_aspect_ratio_v=1.8,
        fuselage_length=10.0, fuselage_radius=0.6,
        mesh_deflection=1e-4,
        taper_ratio=0.40, sweep_le=35.0, twist=0.0, dihedral=0.0,
        run_airfoil_sweep=False,
        maximum_camber_input=0.0, maximum_camber_position_input=0.0,
        thickness_to_chord_input=0.12,
        weight=15_000, velocity=60.0, altitude=2_000,
        t_factor_root=1.0, t_factor_tip=1.0,
        front_spar_position=0.15, rear_spar_position=0.60,
        color_wingbox="blue", color_liftingsurface="purple",
    )

    display([wing, horizontal_tail, vertical_tail])