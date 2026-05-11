from itertools import product
from math import radians, tan, cos, sin, sqrt
import numpy as np

from parapy.core import Input, Attribute, Part
from parapy.geom import GeomBase, LoftedSolid, translate, rotate

from Pythonfiles.Components.Liftingsurfaces.Airfoil import Airfoil
from Pythonfiles.Components.Liftingsurfaces.Wingbox import Wingbox
from Pythonfiles.Components.Frame import Frame

import matlab
import matlab.engine
from Pythonfiles.Testfiles.test_aircraft import MATLAB_Q3D_ENGINE


class LiftingSurface(GeomBase):
    """
    Lifting surface for wing, horizontal tail, and vertical tail.

    Airfoil selection
    -----------------
    When run_airfoil_sweep=False (default):
        maximum_camber, maximum_camber_position, thickness_to_chord
        are used directly as given — normal behaviour.

    When run_airfoil_sweep=True:
        The three airfoil parameters above are IGNORED as inputs.
        Instead, best_airfoil_params runs a Q3D sweep over
        camber_range × position_range × thickness_range and resolves
        the three parameters from the feasible candidate with the most
        L/D margin above ld_required.
        All airfoil Parts and the wingbox then pick up the swept values
        automatically via the maximum_camber / maximum_camber_position /
        thickness_to_chord attributes.
    """

    # ------------------------------------------------------------ #
    # FLIGHT CONDITION INPUTS
    # ------------------------------------------------------------ #

    weight:    float = Input(1000)
    velocity:  float = Input(100)
    altitude:  float = Input(10000)
    use_cl:    bool  = Input(True)
    alpha:     float = Input(0.0)      # [deg] only used when use_cl=False

    # ------------------------------------------------------------ #
    # PRIMARY SIZING INPUTS
    # ------------------------------------------------------------ #

    effective_area:  float = Input(None)
    effective_span:  float = Input(None)

    fuselage_length: float = Input()
    fuselage_radius: float = Input()

    fuselage_cone_radius_fn: object = Input(None)

    is_tail:          bool = Input()
    is_vertical_tail: bool = Input()

    mesh_deflection:  float = Input(1e-4)

    color_wingbox:        str = Input()
    color_liftingsurface: str = Input()

    # ------------------------------------------------------------ #
    # GEOMETRY
    # ------------------------------------------------------------ #

    taper_ratio: float = Input()
    sweep_le:    float = Input()
    twist:       float = Input()
    dihedral:    float = Input()

    # ------------------------------------------------------------ #
    # AIRFOIL PARAMETERS
    # These are plain inputs when run_airfoil_sweep=False.
    # When run_airfoil_sweep=True they are overridden by Attributes
    # below that resolve from best_airfoil_params.
    # ------------------------------------------------------------ #

    maximum_camber_input:          float = Input(0.04)
    maximum_camber_position_input: float = Input(0.4)
    thickness_to_chord_input:      float = Input(0.12)

    # ------------------------------------------------------------ #
    # AIRFOIL SWEEP CONTROLS
    # ------------------------------------------------------------ #

    run_airfoil_sweep: bool  = Input(False)

    ld_required:       float = Input(None)   # mission L/D requirement
    camber_range:      list  = Input([0.0, 0.02, 0.04, 0.06])
    position_range:    list  = Input([0.3, 0.4, 0.5])
    thickness_range:   list  = Input([0.10, 0.12, 0.15, 0.18])
    cm_min:            float = Input(-0.10)
    cm_max:            float = Input(0.00)
    t_min:             float = Input(0.10)

    # ------------------------------------------------------------ #
    # AIRFOIL SCALING
    # ------------------------------------------------------------ #

    t_factor_root: float = Input()
    t_factor_tip:  float = Input()

    # ------------------------------------------------------------ #
    # SPARS
    # ------------------------------------------------------------ #

    front_spar_position: float = Input()
    rear_spar_position:  float = Input()

    # ------------------------------------------------------------ #
    # ROSKAM TAIL PARAMETERS
    # ------------------------------------------------------------ #

    tail_volume_coefficient_h: float = Input(None)
    tail_volume_coefficient_v: float = Input(None)
    tail_aspect_ratio_h:       float = Input(None)
    tail_aspect_ratio_v:       float = Input(None)

    wing_ref: "LiftingSurface" = Input(None)

    effective_wing_area:  float = Input(None)
    effective_semi_span:  float = Input(None)

    # ------------------------------------------------------------ #
    # RESOLVED AIRFOIL PARAMETERS
    # These attributes are what all Parts actually read.
    # They either pass through the Input or resolve from the sweep.
    # ------------------------------------------------------------ #

    @Attribute
    def maximum_camber(self):
        return self.best_airfoil_params["camber"] if self.run_airfoil_sweep \
               else self.maximum_camber_input

    @Attribute
    def maximum_camber_position(self):
        return self.best_airfoil_params["position"] if self.run_airfoil_sweep \
               else self.maximum_camber_position_input

    @Attribute
    def thickness_to_chord(self):
        return self.best_airfoil_params["thickness"] if self.run_airfoil_sweep \
               else self.thickness_to_chord_input

    # ------------------------------------------------------------ #
    # AIRFOIL SWEEP
    # ------------------------------------------------------------ #

    @Attribute
    def best_airfoil_params(self):
        """
        Sweeps NACA 4-series (camber × position × thickness) through Q3D
        and returns the parameter dict of the feasible airfoil with the
        most L/D margin above ld_required.

        Only called when run_airfoil_sweep=True.
        ParaPy caches the result — Q3D runs once, not on every access.

        Constraints
        -----------
          L/D  >= ld_required
          CL   >= target_cl        (from 2W/rhoV²S)
          Cm   in [cm_min, cm_max] (pitching moment / stability)
          t/c  >= t_min            (structural minimum)
        """
        if not self.run_airfoil_sweep:
            raise RuntimeError("best_airfoil_params called but run_airfoil_sweep=False.")

        if self.ld_required is None:
            raise ValueError("ld_required must be set when run_airfoil_sweep=True.")

        combos = list(product(self.camber_range, self.position_range, self.thickness_range))
        n      = len(combos)

        print(f"\nAirfoil sweep: {n} Q3D evaluations")
        print(f"  Required L/D : {self.ld_required}")
        print(f"  Target CL    : {self.target_cl:.4f}")
        print(f"  Cm bounds    : [{self.cm_min}, {self.cm_max}]")
        print(f"  t/c minimum  : {self.t_min}\n")

        results = []

        for i, (m, p, t) in enumerate(combos):
            print(f"  [{i+1}/{n}]  camber={m:.3f}  pos={p:.2f}  t/c={t:.3f}", end="  ")

            try:
                # Build lightweight temporary airfoils — no geometry,
                # no dat export, purely to obtain CST vectors.
                root_af = Airfoil(
                    chord              = self.c_root_aero,
                    maximum_camber     = m,
                    camber_position    = p,
                    thickness_to_chord = t,
                    thickness_factor   = self.t_factor_root,
                    export_dat         = False,
                    airfoil_name       = f"_sweep_root_{i}",
                    position           = self._root_position,
                )
                tip_af = Airfoil(
                    chord              = self.c_tip,
                    maximum_camber     = m,
                    camber_position    = p,
                    thickness_to_chord = t,
                    thickness_factor   = self.t_factor_tip,
                    export_dat         = False,
                    airfoil_name       = f"_sweep_tip_{i}",
                    position           = self._tip_position,
                )

                # CST matrix: rows = [root, tip], cols = 12 coefficients
                af_matrix = matlab.double(
                    np.vstack([root_af.CST_vector, tip_af.CST_vector]).tolist()
                )

                # Q3D call with candidate airfoil shape
                Res, _ = MATLAB_Q3D_ENGINE.run_q3d_cst(
                    self._q3d_planform_matrix,
                    af_matrix,
                    matlab.double([0.0]),
                    matlab.double([self.mach]),
                    matlab.double([self.reynolds]),
                    matlab.double([self.velocity]),
                    matlab.double([self.target_cl]),
                    matlab.double([self.altitude]),
                    matlab.double([self.density]),
                    matlab.logical([True]),          # always CL-prescribed in sweep
                    nargout=2
                )
                #print(Res)
                cl  = float(Res["CLwing"])
                cd  = float(Res["CDiwing"])
                cm  = float(Res["CMwing"])
                ld  = cl / cd if cd > 0 else 0.0

                # Individual constraint checks
                ld_met    = ld  >= self.ld_required
                cl_met    = cl  >= self.target_cl
                cm_stable = self.cm_min <= cm <= self.cm_max
                t_ok      = t   >= self.t_min
                feasible  = ld_met and cl_met and cm_stable and t_ok

                print(f"L/D={ld:.2f}  CL={cl:.4f}  Cm={cm:.4f}  "
                      f"{'FEASIBLE' if feasible else 'FAIL'}")

                results.append({
                    "camber":     m,
                    "position":   p,
                    "thickness":  t,
                    "CL":         cl,
                    "CD":         cd,
                    "Cm":         cm,
                    "L/D":        ld,
                    "L/D_margin": ld - self.ld_required,
                    "feasible":   feasible,
                    "ld_met":     ld_met,
                    "cl_met":     cl_met,
                    "cm_stable":  cm_stable,
                })

            except Exception as e:
                print(f"FAILED: {e}")

        # ---- pick best feasible ------------------------------------- #
        feasible_results = [r for r in results if r["feasible"]]

        if feasible_results:
            best = max(feasible_results, key=lambda r: r["L/D_margin"])
            print(f"\nBest airfoil found:")
            print(f"  camber={best['camber']:.3f}  "
                  f"pos={best['position']:.2f}  "
                  f"t/c={best['thickness']:.3f}")
            print(f"  L/D={best['L/D']:.2f}  "
                  f"margin=+{best['L/D_margin']:.2f}  "
                  f"CL={best['CL']:.4f}  Cm={best['Cm']:.4f}")
            return best

        # ---- no feasible — diagnose which constraint blocked most --- #
        print("\nNo feasible airfoil found. Constraint breakdown:")
        print(f"  L/D  failed : {sum(1 for r in results if not r['ld_met'])}/{n}")
        print(f"  CL   failed : {sum(1 for r in results if not r['cl_met'])}/{n}")
        print(f"  Cm   failed : {sum(1 for r in results if not r['cm_stable'])}/{n}")
        raise ValueError(
            f"Airfoil sweep found no feasible candidate. "
            f"Check ld_required={self.ld_required}, "
            f"target_cl={self.target_cl:.4f}, "
            f"cm bounds=[{self.cm_min}, {self.cm_max}]."
        )

    # ------------------------------------------------------------ #
    # ATTACH POSITION  (unchanged)
    # ------------------------------------------------------------ #

    @Attribute
    def attach_x(self):
        if self.is_tail:
            raw = self.wing_ref.attach_x + self.tail_arm
            return min(raw, self.fuselage_length - self.c_root_aero)
        return 0.40 * self.fuselage_length

    @Attribute
    def attach_z(self):
        if self.is_tail and self.is_vertical_tail:
            return self.fuselage_wall_radius
        return 0.0

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

    # ------------------------------------------------------------ #
    # EFFECTIVE AREA AND SPAN  (unchanged)
    # ------------------------------------------------------------ #

    @Attribute
    def _effective_area(self):
        if not self.is_tail:
            if self.effective_area is None:
                raise ValueError("Wing requires effective_area input.")
            return self.effective_area
        if self.wing_ref is None:
            raise ValueError("Tail requires wing_ref.")
        S_w = self.wing_ref._effective_area
        b_w = 2 * self.wing_ref._effective_span
        c_w = self.wing_ref.mean_aerodynamic_chord
        if self.is_vertical_tail:
            if self.tail_volume_coefficient_v is None or self.tail_aspect_ratio_v is None:
                raise ValueError("VT requires tail_volume_coefficient_v and tail_aspect_ratio_v.")
            return (self.tail_volume_coefficient_v * S_w * b_w) / self.tail_arm
        else:
            if self.tail_volume_coefficient_h is None or self.tail_aspect_ratio_h is None:
                raise ValueError("HT requires tail_volume_coefficient_h and tail_aspect_ratio_h.")
            return (self.tail_volume_coefficient_h * S_w * c_w) / self.tail_arm

    @Attribute
    def _effective_span(self):
        if not self.is_tail:
            if self.effective_semi_span is None:
                raise ValueError("Wing requires effective_span input.")
            return self.effective_semi_span
        if self.is_vertical_tail:
            return np.sqrt(self.tail_aspect_ratio_v * self._effective_area)
        else:
            return np.sqrt(self.tail_aspect_ratio_h * self._effective_area) / 2

    # ------------------------------------------------------------ #
    # CHORD  (unchanged)
    # ------------------------------------------------------------ #

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

    # ------------------------------------------------------------ #
    # DERIVED AERO PROPERTIES  (unchanged)
    # ------------------------------------------------------------ #

    @Attribute
    def aspect_ratio(self):
        return (2 * self._effective_span) ** 2 / self._effective_area

    @Attribute
    def mean_aerodynamic_chord(self):
        tr = self.taper_ratio
        return (2 / 3) * self.c_root_aero * (1 + tr + tr ** 2) / (1 + tr)

    @Attribute
    def mac_spanwise_position(self):
        tr = self.taper_ratio
        return self._effective_span * (1 + 2 * tr) / (3 * (1 + tr))

    @Attribute
    def mac_x_offset(self):
        return self.mac_spanwise_position * tan(radians(self.sweep_le))

    # ------------------------------------------------------------ #
    # POSITIONING HELPERS  (unchanged)
    # ------------------------------------------------------------ #

    def _spanwise_offsets(self, y_sign: float):
        R  = self.fuselage_wall_radius
        dx = R * tan(radians(self.sweep_le))
        dy = y_sign * R
        dz = R * sin(radians(self.dihedral))
        return dx, dy, dz

    @Attribute
    def _root_position_wingbox(self):
        if not self.is_vertical_tail:
            return translate(self.position, "x", self.attach_x, "y", 0.0, "z", self.attach_z)
        base = translate(self.position, "x", self.attach_x, "z", self.attach_z)
        return rotate(base, "x", radians(90))

    @Attribute
    def _root_position(self):
        if not self.is_vertical_tail:
            dx, dy, dz = self._spanwise_offsets(+1)
            return translate(self.position, "x", self.attach_x + dx, "y", dy, "z", self.attach_z + dz)
        R  = self.fuselage_wall_radius
        dx = R * tan(radians(self.sweep_le))
        dz = R
        base = translate(self.position, "x", self.attach_x + dx, "z", self.attach_z + dz)
        return rotate(base, "x", radians(90))

    @Attribute
    def _root_position_mirrored(self):
        if self.is_vertical_tail:
            return self._root_position
        dx, dy, dz = self._spanwise_offsets(-1)
        return translate(self.position, "x", self.attach_x + dx, "y", dy, "z", self.attach_z + dz)

    @Attribute
    def _tip_position(self):
        if not self.is_vertical_tail:
            return rotate(
                translate(
                    self._root_position,
                    "y",  self._effective_span,
                    "x",  self._effective_span * tan(radians(self.sweep_le)),
                    "z",  self._effective_span * np.sin(radians(self.dihedral)),
                ),
                "y", radians(self.twist),
            )
        return rotate(
            translate(
                self._root_position,
                "y", self._effective_span,
                "x", self._effective_span * tan(radians(self.sweep_le)),
            ),
            "z", radians(self.twist),
        )

    @Attribute
    def _tip_position_mirrored(self):
        if self.is_vertical_tail:
            return None
        return rotate(
            translate(
                self._root_position_mirrored,
                "y", -self._effective_span,
                "x",  self._effective_span * tan(radians(self.sweep_le)),
                "z",  self._effective_span * np.sin(radians(self.dihedral)),
            ),
            "y", radians(self.twist),
        )

    # ------------------------------------------------------------ #
    # AIRFOIL PARTS  (unchanged — now read resolved Attributes)
    # ------------------------------------------------------------ #

    @Part
    def root_airfoil_wingbox(self):
        return Airfoil(
            chord              = self.c_root_geometric,
            maximum_camber     = self.maximum_camber,       # resolved
            camber_position    = self.maximum_camber_position,
            thickness_to_chord = self.thickness_to_chord,
            export_dat         = True,
            airfoil_name       = "root_airfoil_geometric",
            position           = self._root_position_wingbox,
        )

    @Part
    def root_airfoil(self):
        return Airfoil(
            chord              = self.c_root_aero,
            maximum_camber     = self.maximum_camber,       # resolved
            camber_position    = self.maximum_camber_position,
            thickness_to_chord = self.thickness_to_chord,
            export_dat         = True,
            airfoil_name       = "root_airfoil_aero",
            position           = self._root_position,
        )

    @Part
    def root_airfoil_mirrored(self):
        return Airfoil(
            chord              = self.c_root_aero,
            maximum_camber     = self.maximum_camber,
            camber_position    = self.maximum_camber_position,
            thickness_to_chord = self.thickness_to_chord,
            export_dat         = True,
            airfoil_name       = "root_airfoil_aero_mirrored",
            position           = self._root_position_mirrored,
            suppress           = self.is_vertical_tail,
        )

    @Part
    def tip_airfoil(self):
        return Airfoil(
            chord              = self.c_tip,
            maximum_camber     = self.maximum_camber,       # resolved
            camber_position    = self.maximum_camber_position,
            thickness_to_chord = self.thickness_to_chord,
            export_dat         = True,
            airfoil_name       = "tip_airfoil",
            position           = self._tip_position,
        )

    @Part
    def tip_airfoil_mirrored(self):
        return Airfoil(
            chord              = self.c_tip,
            maximum_camber     = self.maximum_camber,
            camber_position    = self.maximum_camber_position,
            thickness_to_chord = self.thickness_to_chord,
            export_dat         = True,
            airfoil_name       = "tip_airfoil_mirrored",
            position           = self._tip_position_mirrored,
            suppress           = self.is_vertical_tail,
        )

    # ------------------------------------------------------------ #
    # SOLIDS  (unchanged)
    # ------------------------------------------------------------ #

    @Part
    def solid(self):
        return LoftedSolid(
            profiles        = [self.root_airfoil.geometry, self.tip_airfoil.geometry],
            color           = self.color_liftingsurface,
            mesh_deflection = self.mesh_deflection,
        )

    @Part
    def solid_mirrored(self):
        return LoftedSolid(
            profiles        = [self.root_airfoil_mirrored.geometry, self.tip_airfoil_mirrored.geometry],
            color           = self.color_liftingsurface,
            transparency    = 0.6,
            mesh_deflection = self.mesh_deflection,
            suppress        = self.is_vertical_tail,
        )

    # ------------------------------------------------------------ #
    # FRAME  (unchanged)
    # ------------------------------------------------------------ #

    @Part
    def frame(self):
        return Frame(pos=self._root_position, hidden=False)

    # ------------------------------------------------------------ #
    # WINGBOX  (unchanged — picks up resolved thickness_to_chord
    #           indirectly via airfoil_root / airfoil_tip Parts)
    # ------------------------------------------------------------ #

    @Part
    def wingbox(self):
        return Wingbox(
            c_root               = self.c_root_geometric,
            c_tip                = self.c_tip,
            semi_span            = self._geometric_span,
            sweep_le             = self.sweep_le,
            dihedral             = self.dihedral,
            twist                = self.twist,
            front_spar_position  = self.front_spar_position,
            rear_spar_position   = self.rear_spar_position,
            airfoil_root         = self.root_airfoil_wingbox,
            airfoil_tip          = self.tip_airfoil,
            color                = self.color_wingbox,
        )

    @Part
    def wingbox_mirrored(self):
        return Wingbox(
            c_root               = self.c_root_geometric,
            c_tip                = self.c_tip,
            semi_span            = -self._geometric_span,
            sweep_le             = -self.sweep_le,
            dihedral             = -self.dihedral,
            twist                = self.twist,
            front_spar_position  = self.front_spar_position,
            rear_spar_position   = self.rear_spar_position,
            airfoil_root         = self.root_airfoil_wingbox,
            airfoil_tip          = self.tip_airfoil_mirrored,
            color                = self.color_wingbox,
            suppress             = self.is_vertical_tail,
        )

    # ------------------------------------------------------------ #
    # ISA ATMOSPHERE
    # ------------------------------------------------------------ #

    @Attribute
    def _isa(self):
        T0    = 288.15
        p0    = 101_325.0
        L     = 0.0065
        R     = 287.058
        gamma = 1.4
        g     = 9.80665
        h     = self.altitude

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
        return (2.0 * self.weight) / (
            self.density * self.velocity**2 * self._effective_area
        )

    # ------------------------------------------------------------ #
    # Q3D MATRICES
    # ------------------------------------------------------------ #

    @Attribute
    def _q3d_airfoil_matrix(self):
        cst_root = self.root_airfoil.CST_vector
        cst_tip  = self.tip_airfoil.CST_vector
        return matlab.double(np.vstack([cst_root, cst_tip]).tolist())

    @Attribute
    def _q3d_planform_matrix(self):
        y_tip    = self._effective_span
        x_le_tip = y_tip * tan(radians(self.sweep_le))
        z_tip    = y_tip * np.sin(radians(self.dihedral))
        return matlab.double([
            [0.0,      0.0,   0.0,   self.c_root_aero, 0.0       ],
            [x_le_tip, y_tip, z_tip, self.c_tip,       self.twist],
        ])

    # ------------------------------------------------------------ #
    # Q3D CALL
    # ------------------------------------------------------------ #

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
            nargout=2
        )


# ---------------------------------------------------------------------- #
# TEST
# ---------------------------------------------------------------------- #

if __name__ == "__main__":

    from parapy.gui import display

    wing = LiftingSurface(
        label="main_wing",

        # --- sizing --- #
        effective_area       = 18.0,
        effective_semi_span  = 7.4,

        fuselage_length      = 10.0,
        fuselage_radius      = 0.6,

        is_tail              = False,
        is_vertical_tail     = False,

        mesh_deflection      = 1e-4,

        # --- geometry --- #
        taper_ratio          = 0.40,
        sweep_le             = 5.0,
        twist                = -2.0,
        dihedral             = 5.0,

        # --- airfoil (fixed, no sweep) --- #
        run_airfoil_sweep            = False,
        maximum_camber_input         = 0.04,
        maximum_camber_position_input= 0.4,
        thickness_to_chord_input     = 0.15,

        # --- flight conditions (needed for Q3D / sweep) --- #
        weight               = 15_000,    # [N]
        velocity             = 60.0,      # [m/s]
        altitude             = 2_000,     # [m]

        t_factor_root        = 1.0,
        t_factor_tip         = 1.0,

        front_spar_position  = 0.15,
        rear_spar_position   = 0.60,

        color_wingbox        = "yellow",
        color_liftingsurface = "orange",
    )

    # --- wing with airfoil sweep enabled --- #
    wing_swept = LiftingSurface(
        label="main_wing_swept",

        effective_area       = 18.0,
        effective_semi_span  = 7.4,

        fuselage_length      = 10.0,
        fuselage_radius      = 0.6,

        is_tail              = False,
        is_vertical_tail     = False,

        mesh_deflection      = 1e-4,

        taper_ratio          = 0.40,
        sweep_le             = 5.0,
        twist                = -2.0,
        dihedral             = 5.0,

        # --- airfoil sweep --- #
        run_airfoil_sweep            = True,
        ld_required                  = 10.0,
        camber_range                 = [0.0, 0.02, 0.04, 0.06],
        position_range               = [0.3, 0.4, 0.5],
        thickness_range              = [0.10, 0.12, 0.15, 0.18],
        cm_min                       = -0.30,
        cm_max                       = 0.00,
        t_min                        = 0.00,
        # these are ignored when run_airfoil_sweep=True
        # but ParaPy still needs a value for the Input
        maximum_camber_input         = 0.04,
        maximum_camber_position_input= 0.4,
        thickness_to_chord_input     = 0.12,

        weight               = 15_000,
        velocity             = 60.0,
        altitude             = 2_000,

        t_factor_root        = 1.0,
        t_factor_tip         = 1.0,

        front_spar_position  = 0.15,
        rear_spar_position   = 0.60,

        color_wingbox        = "yellow",
        color_liftingsurface = "orange",
    )

    horizontal_tail = LiftingSurface(
        label="horizontal_tail",

        wing_ref             = wing,
        is_tail              = True,
        is_vertical_tail     = False,

        tail_volume_coefficient_h = 0.60,
        tail_volume_coefficient_v = 0.04,
        tail_aspect_ratio_h       = 4.5,
        tail_aspect_ratio_v       = 1.8,

        fuselage_length      = 10.0,
        fuselage_radius      = 0.6,

        mesh_deflection      = 1e-4,

        taper_ratio          = 0.40,
        sweep_le             = 10.0,
        twist                = 0.0,
        dihedral             = 0.0,

        run_airfoil_sweep            = False,
        maximum_camber_input         = 0.0,
        maximum_camber_position_input= 0.4,
        thickness_to_chord_input     = 0.12,

        weight               = 15_000,
        velocity             = 60.0,
        altitude             = 2_000,

        t_factor_root        = 1.0,
        t_factor_tip         = 1.0,

        front_spar_position  = 0.15,
        rear_spar_position   = 0.60,

        color_wingbox        = "red",
        color_liftingsurface = "green",
    )

    vertical_tail = LiftingSurface(
        label="vertical_tail",

        wing_ref             = wing,
        is_tail              = True,
        is_vertical_tail     = True,

        tail_volume_coefficient_h = 0.60,
        tail_volume_coefficient_v = 0.04,
        tail_aspect_ratio_h       = 4.5,
        tail_aspect_ratio_v       = 1.8,

        fuselage_length      = 10.0,
        fuselage_radius      = 0.6,

        mesh_deflection      = 1e-4,

        taper_ratio          = 0.40,
        sweep_le             = 35.0,
        twist                = 0.0,
        dihedral             = 0.0,

        run_airfoil_sweep            = False,
        maximum_camber_input         = 0.0,
        maximum_camber_position_input= 0.0,
        thickness_to_chord_input     = 0.12,

        weight               = 15_000,
        velocity             = 60.0,
        altitude             = 2_000,

        t_factor_root        = 1.0,
        t_factor_tip         = 1.0,

        front_spar_position  = 0.15,
        rear_spar_position   = 0.60,

        color_wingbox        = "blue",
        color_liftingsurface = "purple",
    )

    display([wing, wing_swept, horizontal_tail, vertical_tail])