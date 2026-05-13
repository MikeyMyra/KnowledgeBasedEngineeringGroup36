import numpy as np
import matplotlib.pyplot as plt
import os
import sys
import json
from ISA_calculator import *

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

class WP_WS_Diagram:
    
    def __init__(self, aspect_ratio, CL_max_clean, CL_max_land, CL_max_TO, cruise_speed, CD0, prop_eff, engine_type,
                 oswald_factor, n_max, x_max=1000, y_max=1, plot=True, save_path=None):

        self.plot = plot
        self.save_path = save_path
        self.x_max = x_max
        self.y_max = y_max
        # self.load_data()

        self.engine_type = engine_type
        self.engine_bypass_ratio = 0.0 if self.engine_type == 'Jet' else 0.4
        
        self.CL_max_clean = [CL_max_clean]
        self.CL_max_land  = [CL_max_land]
        self.CL_max_TO    = [CL_max_TO]
        self.A_design     = [aspect_ratio]
        
        self.V_cruise = cruise_speed
        self.V_stall = 35
        self.climb_rate = 2
        self.climb_gradient = self.climb_rate / self.V_stall
        
        self.e = oswald_factor
        self.CD0 = CD0
        self.n_max = n_max
        
        self.f = 0.97 * 0.985
        
        self.TOP = 120 #self.aircraft['mission_profile']['take-off_parameter']  # Takeoff Parameter (depends on aircraft class)
        self.runway_length = 1000 #self.aircraft['performance']['landing']['required_runway_length_m']
        self.engine_bypass_ratio = 5
        self.ground_friction_coeff = 0.4
        self.n_p = prop_eff #self.aircraft['constants']['propulsion']['propeller_eff']
        self.prop_setting = 1
        
        self.h = [0] #self.pre_h if isinstance(self.pre_h, list) else [self.pre_h]
        self.rho = []
        self.rho0 = 1.225
        # self.rho = self.rho if isinstance(self.pre_rho, list) else [self.pre_rho]
        self.set_densities()
        if self.plot or self.save_path:
            self.plot_wing_loading_constraints()
    
    # def load_data(self):
    #
    #     # Load main aircraft data
    #     with open("__data__/aircraft_data.json", "r") as f:
    #         self.aircraft = json.load(f)
    #
    #     self.pre_CL_MAX_clean = self.aircraft['aerodynamics']['CL_max']
    #     self.pre_CL_MAX_land = self.aircraft['aerodynamics']['CL_land']
    #     self.pre_CL_MAX_TO = self.aircraft['aerodynamics']['CL_TO']
    #     self.pre_rho = self.aircraft['environment']['density_kg/m3']
    #     self.pre_h = self.aircraft['environment']['altitude_m']
    #     self.pre_A_design = self.aircraft['aerodynamics']['aspect_ratio'] / 2
    
    def set_densities(self):
        for h in self.h:
            rho = ISA_calculator(h)[2]
            self.rho.append(rho)

    def wing_loading(self, V, CL_max, rho=1.225):
        """Calculate Wing Loading for given velocity and maximum lift coefficient in cruise"""
        return 0.5 * rho * V ** 2 * CL_max

    def take_off_loading(self, CL_TO_list=None, rho=1.225):
        """
        Calculate W/P vs W/S curves for takeoff performance.
        Roskam Part I, eq 3.8 (propeller) and eq 3.6 (jet)
        TOP: takeoff parameter, ~200 for military, ~300 for civil
        """


        if CL_TO_list is None:
            CL_TO_list = [self.CL_max_TO]

        W_S_range = np.linspace(1, self.x_max, 1000)
        results = []

        if self.engine_type in ('Turboprop', 'Piston'):
            for CL_TO in CL_TO_list:
                effective_CL = CL_TO / (1.1 ** 2)
                W_P_range = (self.TOP / W_S_range) * effective_CL * (rho / self.rho0)
                results.append((W_P_range, W_S_range, CL_TO))

        elif self.engine_type == 'Jet':
            for CL_TO in CL_TO_list:
                effective_CL = CL_TO / (1.1 ** 2)
                # Roskam: T/W = W_S / (effective_CL * TOP * (rho/rho0))
                # T_W_range = W_S_range / (effective_CL * self.TOP * (rho / self.rho0))
                k_2 = 0.75 * (5 + self.engine_bypass_ratio) / (4 + self.engine_bypass_ratio)
                T_W_range = (1.44 * W_S_range) / (9.81 * k_2 * self.runway_length * rho * effective_CL) + \
                (0.72 * self.CD0) / (k_2 * effective_CL) + self.ground_friction_coeff / k_2
                results.append((T_W_range, W_S_range, CL_TO))
        return results

    def landing_loading(self, CL_max, rho=1.225):
        """
        Calculate Wing Loading for landing phase.
        Roskam Part I, eq 3.11
        landing_distance in meters, f accounts for approach speed factor
        f = 1.0 for military, f = 1.667 for civil (FAR 25)
        """
        return (CL_max * rho * (self.runway_length / 0.5915)) / (2 * self.f)

    def climb_loading(self, A_list=None, rho=1.225):
        """
        Calculate W/P or T/W vs W/S curves for cruise climb performance.
        Roskam Part I, eq 3.18 (propeller) and eq 3.15 (jet)
        Maximizes climb speed at cruise condition.
        """
        if A_list is None:
            A_list = [self.aspect_ratio]

        W_S_range = np.linspace(1, self.x_max, 1000)
        results = []

        if self.engine_type in ('Turboprop', 'Piston'):
            for A in A_list:
                # W/P = n_p / (climb_speed * (D/W - climb_gradient))
                # D/W computed from drag polar at cruise
                CD = self.CD0 + (W_S_range / (0.5 * rho * self.V_cruise ** 2)) ** 2 / (np.pi * A * self.e)
                CL = W_S_range / (0.5 * rho * self.V_cruise ** 2)
                LD = CL / CD
                W_P_range = self.n_p / (self.V_cruise * (1 / LD - self.climb_gradient) ** -1) \
                    if False else \
                    self.n_p * (self.prop_setting * (rho / self.rho0) ** 0.75) / \
                    (((self.CD0 * 0.5 * rho * self.V_cruise ** 3) / W_S_range) +
                     (W_S_range / (np.pi * A * self.e * 0.5 * rho * self.V_cruise)))
                results.append((W_P_range, W_S_range, A))

        elif self.engine_type == 'Jet':
            for A in A_list:
                # Roskam eq 3.15: T/W = CD0 * q / (W/S) + (W/S) / (pi * A * e * q)
                q = 0.5 * rho * self.V_cruise ** 2
                T_W_range = (self.CD0 * q / W_S_range) + \
                            (W_S_range / (np.pi * A * self.e * q)) + \
                            self.climb_gradient
                results.append((T_W_range, W_S_range, A))

        return results

    def climb_rate_loading(self, A_list=None, rho=1.225):
        """
        Calculate W/P or T/W vs W/S for climb rate requirement.
        Roskam Part I, eq 3.22 (propeller) and eq 3.19 (jet)
        climb_rate in m/s
        """
        if A_list is None:
            A_list = [self.aspect_ratio]

        W_S_range = np.linspace(1, self.x_max, 1000)
        results = []

        if self.engine_type in ('Turboprop', 'Piston'):
            for A in A_list:
                # Roskam eq 3.22
                W_P_range = self.n_p / (
                        self.climb_rate +
                        (np.sqrt(W_S_range) * np.sqrt(2 / rho)) /
                        (1.345 * ((A * self.e) ** 0.75) / (self.CD0 ** 0.25))
                )
                results.append((W_P_range, W_S_range, A))

        elif self.engine_type == 'Jet':
            for A in A_list:
                # Roskam eq 3.19: T/W = climb_rate/V_climb + 2*sqrt(CD0 / (pi*A*e))
                # V for best climb rate: V = sqrt(2*W/S / rho * sqrt(3/(pi*A*e*CD0)))  (Roskam eq 3.20)
                V_climb = np.sqrt((2 / rho) * W_S_range * np.sqrt(3 / (np.pi * A * self.e * self.CD0)))
                # clamp to avoid divide by zero
                V_climb = np.maximum(V_climb, 1.0)
                T_W_range = (self.climb_rate / V_climb) + \
                            2 * np.sqrt(self.CD0 / (np.pi * A * self.e))
                results.append((T_W_range, W_S_range, A))

        return results

    def climb_gradient_loading(self, A_list=None, rho=1.225):
        """
        Calculate W/P or T/W vs W/S for climb gradient requirement.
        Roskam Part I, eq 3.25 (propeller) and eq 3.23 (jet)
        climb_gradient dimensionless (e.g. 0.024 for 2.4%)
        """
        if A_list is None:
            A_list = [self.aspect_ratio]

        W_S_range = np.linspace(1, self.x_max, 1000)
        results = []

        if self.engine_type in ('Turboprop', 'Piston'):
            for A in A_list:
                CD = self.drag_polar(A=A)
                CL_land = self.CL_max_clean[-1]
                # Roskam eq 3.25
                W_P_range = self.n_p / (
                        np.sqrt(W_S_range) *
                        (self.climb_gradient + (CD / CL_land)) *
                        np.sqrt(2 / (rho * CL_land))
                )
                results.append((W_P_range, W_S_range, A))

        elif self.engine_type == 'Jet':
            for A in A_list:
                CD = self.drag_polar(A=A)
                CL_land = self.CL_max_clean[-1]
                # Roskam eq 3.23: T/W = climb_gradient + CD/CL
                T_W_range = np.full_like(W_S_range,
                                         self.climb_gradient + (CD / CL_land))
                results.append((T_W_range, W_S_range, A))

        return results
    
    def maneuver_loading(self, rho=1.225):
        """
        Structural / maneuver constraint.
        At load factor n, stall speed increases: V_stall_n = V_stall * sqrt(n)
        Wing loading must satisfy: W/S <= 0.5 * rho * V_stall^2 * CL_max / n
        Or expressed as a vertical line on the diagram.
        """
        W_S_limit = 0.5 * rho * self.V_cruise**2 * self.CL_max_clean[0] / self.n_max
        return W_S_limit

    def find_design_point(self, rho=1.225):
        """
        Finds the optimal design point (W/S, T/W or W/P) as the intersection of:
        - the most restrictive vertical constraint (leftmost vertical line)
        - the most restrictive curve constraint evaluated at that W/S
        Returns (W_S_design, y_design)
        """
        W_S_range = np.linspace(1, self.x_max, 10000)

        # --- Step 1: find the leftmost vertical constraint ---
        vertical_limits = []

        # Stall limits
        for CL in self.CL_max_land:
            WL_stall = self.wing_loading(self.V_stall, CL, rho=rho)
            vertical_limits.append(WL_stall)

        # Cruise limits
        for CL in self.CL_max_clean:
            WL_cruise = self.wing_loading(self.V_cruise, CL, rho=rho)
            vertical_limits.append(WL_cruise)

        # Landing limits
        for CL in self.CL_max_land:
            WL_land = self.landing_loading(CL, rho=rho)
            vertical_limits.append(WL_land)
        
        # Maneuver limit
        WL_maneuver = self.maneuver_loading(rho=rho)
        vertical_limits.append(WL_maneuver)

        W_S_design = min(vertical_limits)  # most restrictive = leftmost

        # --- Step 2: evaluate all curve constraints at W_S_design ---
        y_values = []

        # Takeoff curves
        for W_P_or_T_W, W_S_curve, _ in self.take_off_loading(CL_TO_list=self.CL_max_TO, rho=rho):
            y_at_design = np.interp(W_S_design, W_S_curve, W_P_or_T_W)
            y_values.append(y_at_design)

        # Climb curves
        for W_P_or_T_W, W_S_curve, _ in self.climb_loading(A_list=self.A_design, rho=rho):
            y_at_design = np.interp(W_S_design, W_S_curve, W_P_or_T_W)
            y_values.append(y_at_design)

        # Climb rate curves
        for W_P_or_T_W, W_S_curve, _ in self.climb_rate_loading(A_list=self.A_design, rho=rho):
            y_at_design = np.interp(W_S_design, W_S_curve, W_P_or_T_W)
            y_values.append(y_at_design)

        # Climb gradient curves
        for W_P_or_T_W, W_S_curve, _ in self.climb_gradient_loading(A_list=self.A_design, rho=rho):
            y_at_design = np.interp(W_S_design, W_S_curve, W_P_or_T_W)
            y_values.append(y_at_design)

        # --- Step 3: most restrictive curve at that W/S ---
        if self.engine_type == 'Jet':
            y_design = max(y_values)  # for jet: T/W must be ABOVE all curves → take max
        else:
            y_design = min(y_values)  # for prop: W/P must be BELOW all curves → take min

        return W_S_design, y_design

    def plot_wing_loading_constraints(self, CL_TO_list=None):
        """Plot Wing Loading Constraints for Stall, Cruise, Takeoff, etc. across up to 3 altitudes as subplots."""

        num_altitudes = min(len(self.rho), 3)
        fig, axs = plt.subplots(1, num_altitudes, figsize=(6 * num_altitudes, 6), sharey=True)
        axs = np.atleast_1d(axs)

        is_jet = self.engine_type == 'Jet'
        y_label = "Thrust-to-Weight T/W (-)" if is_jet else "Weight-to-Power W/P (N/W)"

        for idh, rho in enumerate(self.rho[:num_altitudes]):
            ax = axs[idh]

            # Stall constraint
            stall_lines = []
            for idx, CL in enumerate(self.CL_max_land):
                WL_stall = self.wing_loading(self.V_stall, CL, rho=rho)
                stall_lines.append(WL_stall)
                color = f"C{idx % 10}"
                ax.axvline(x=WL_stall, color=color, linestyle='--',
                           label=f"Stall CL={CL:.2f} @ {round(WL_stall, 2)}")
            WL_stall_limit = min(stall_lines)
            ax.axvspan(WL_stall_limit, self.x_max, color='red', alpha=0.15)

            # Cruise constraint
            cruise_lines = []
            for idx, CL in enumerate(self.CL_max_clean):
                WL_cruise = self.wing_loading(self.V_cruise, CL, rho=rho)
                cruise_lines.append(WL_cruise)
                color = f"C{idx % 10 + 1}"
                ax.axvline(x=WL_cruise, color=color, linestyle='--',
                           label=f"Cruise CL={CL:.2f} @ {round(WL_cruise, 2)}")
            WL_cruise_limit = min(cruise_lines)
            ax.axvspan(WL_cruise_limit, self.x_max, color='red', alpha=0.15)

            # Landing constraint
            for idx, CL in enumerate(self.CL_max_land):
                WL_land = self.landing_loading(CL, rho=rho)
                color = f"C{idx % 10 + 2}"
                ax.axvline(x=WL_land, color=color, linestyle='--',
                           label=f"Landing CL={CL:.2f} @ {round(WL_land, 2)}")
            ax.axvspan(WL_land, self.x_max, color='red', alpha=0.15)

            # Takeoff constraint
            takeoff_curves = self.take_off_loading(CL_TO_list=self.CL_max_TO, rho=rho)
            for idx, (W_P_or_T_W, W_S_range, CL_TO) in enumerate(takeoff_curves):
                color = f"C{idx % 10 + 3}"
                ax.plot(W_S_range, W_P_or_T_W, label=f"Takeoff CL_TO={CL_TO:.2f}", color=color)
            if is_jet:
                ax.fill_between(W_S_range, 0, W_P_or_T_W, color='red', alpha=0.15)  # below curve
            else:
                ax.fill_between(W_S_range, W_P_or_T_W, self.y_max, color='red', alpha=0.15)  # above curve

            # Climb constraint
            climb_curves = self.climb_loading(A_list=self.A_design, rho=rho)
            for idx, (W_P_or_T_W, W_S_range, A) in enumerate(climb_curves):
                color = f"C{idx % 10 + 4}"
                ax.plot(W_S_range, W_P_or_T_W, label=f"Climb A={A:.0f}", color=color)
            if is_jet:
                ax.fill_between(W_S_range, 0, W_P_or_T_W, color='red', alpha=0.15)
            else:
                ax.fill_between(W_S_range, W_P_or_T_W, self.y_max, color='red', alpha=0.15)

            # Climb rate constraint
            climb_rate_curves = self.climb_rate_loading(A_list=self.A_design, rho=rho)
            for idx, (W_P_or_T_W, W_S_range, A) in enumerate(climb_rate_curves):
                color = f"C{idx % 10 + 5}"
                ax.plot(W_S_range, W_P_or_T_W, label=f"Climb Rate A={A:.0f}", color=color)
            if is_jet:
                ax.fill_between(W_S_range, 0, W_P_or_T_W, color='red', alpha=0.15)
            else:
                ax.fill_between(W_S_range, W_P_or_T_W, self.y_max, color='red', alpha=0.15)

            # Climb gradient constraint
            climb_gradient_curves = self.climb_gradient_loading(A_list=self.A_design, rho=rho)
            for idx, (W_P_or_T_W, W_S_range, A) in enumerate(climb_gradient_curves):
                color = f"C{idx % 10 + 6}"
                ax.plot(W_S_range, W_P_or_T_W, label=f"Climb Gradient A={A:.0f}", color=color)
            if is_jet:
                ax.fill_between(W_S_range, 0, W_P_or_T_W, color='red', alpha=0.15)
            else:
                ax.fill_between(W_S_range, W_P_or_T_W, self.y_max, color='red', alpha=0.15)
            
            # Maneuver constraint
            WL_maneuver = self.maneuver_loading(rho=rho)

            ax.axvline(
                x=WL_maneuver,
                color='purple',
                linestyle='-.',
                label=f"Maneuver n={self.n_max} @ {round(WL_maneuver, 2)}"
            )

            ax.axvspan(
                WL_maneuver,
                self.x_max,
                color='red',
                alpha=0.15
            )

            # Axis setup
            ax.set_xlim(0, self.x_max)
            ax.set_ylim(0, self.y_max)
            ax.set_xlabel("Wing Loading W/S (N/m²)")
            if idh == 0:
                ax.set_ylabel(y_label)
            ax.set_title(f"Altitude: {self.h[idh]} m")
            ax.grid(True)
            ax.legend(fontsize='small', loc='upper right')

        plt.suptitle("Wing Loading vs " + ("Thrust-to-Weight" if is_jet else "Weight-to-Power") +
                     " Across Altitudes", fontsize=16)
        plt.tight_layout(rect=[0, 0, 1, 0.95])

        # Design point
        W_S_opt, y_opt = self.find_design_point(rho=rho)
        ax.scatter(W_S_opt, y_opt, color='black', zorder=5, s=80,
                   label=f"Design point W/S={W_S_opt:.0f}, y={y_opt:.3f}")
        ax.annotate(f"({W_S_opt:.0f}, {y_opt:.3f})",
                    xy=(W_S_opt, y_opt),
                    xytext=(W_S_opt + self.x_max * 0.03, y_opt + self.y_max * 0.03),
                    fontsize=8,
                    arrowprops=dict(arrowstyle='->', color='black'))

        if self.save_path:
            plt.savefig(self.save_path, dpi=150, bbox_inches='tight')
        if self.plot:
            plt.show()
        else:
            plt.close(fig)

    def drag_polar(self, A=0):
        return self.CD0 + (self.CL_max_clean[len(self.CL_max_clean) - 1] ** 2) / (np.pi * A * self.e)  # Drag Coefficient

# Example Usage
if __name__ == "__main__":
    wpws = WP_WS_Diagram(

        plot=True,
        x_max=1000,
        y_max=1

    )
    wpws.plot_wing_loading_constraints()