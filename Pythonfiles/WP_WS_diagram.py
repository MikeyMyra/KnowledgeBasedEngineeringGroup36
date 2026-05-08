import numpy as np
import matplotlib.pyplot as plt
import os
import sys
import json

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

class WP_WS_Diagram:
    
    def __init__(self, x_max=1000, y_max=1, plot=True):
        
        self.plot = plot
        self.x_max = x_max
        self.y_max = y_max
        self.load_data()
        
        self.CL_max_clean = [1.4] #self.pre_CL_MAX_clean if isinstance(self.pre_CL_MAX_clean, list) else [self.pre_CL_MAX_clean]
        self.CL_max_land  = [2.4] #self.pre_CL_MAX_land if isinstance(self.pre_CL_MAX_land, list) else [self.pre_CL_MAX_land]
        self.CL_max_TO    = [1.8] #self.pre_CL_MAX_TO if isinstance(self.pre_CL_MAX_TO, list) else [self.pre_CL_MAX_TO]
        self.A_design     = [6] #self.pre_A_design if isinstance(self.pre_A_design, list) else [self.pre_A_design]
        
        self.V_cruise = 70 #self.aircraft['performance']['cruise']['velocity_m/s']
        self.V_stall = 28 #self.aircraft['performance']['cruise']['stall_speed_m/s']
        self.climb_rate = 2 #self.aircraft['requirements']['rate_of_climb_m/s']
        self.climb_gradient = self.climb_rate / self.V_stall
        
        self.e = 0.8 #self.aircraft['aerodynamics']['oswald_efficiency']
        self.CD0 = 0.02 #self.aircraft['aerodynamics']['CD0']
        
        self.f = 0.94 #self.aircraft['weights']['fuel_fraction']
        
        self.TOP = 120 #self.aircraft['mission_profile']['take-off_parameter']  # Takeoff Parameter (depends on aircraft class)
        self.landing_distance = 1000 #self.aircraft['performance']['landing']['required_runway_length_m']
        self.n_p = 0.85 #self.aircraft['constants']['propulsion']['propeller_eff']
        self.prop_setting = 1
        
        self.h = [0] #self.pre_h if isinstance(self.pre_h, list) else [self.pre_h]
        self.rho = []
        self.rho0 = 1.225
        self.rho = self.pre_rho if isinstance(self.pre_rho, list) else [self.pre_rho]
        #self.set_densities()
        #self.plot_wing_loading_constraints()
    
    def load_data(self):
        
        # Load main aircraft data
        with open("__data__/aircraft_data.json", "r") as f:
            self.aircraft = json.load(f)
        
        self.pre_CL_MAX_clean = self.aircraft['aerodynamics']['CL_max']
        self.pre_CL_MAX_land = self.aircraft['aerodynamics']['CL_land']
        self.pre_CL_MAX_TO = self.aircraft['aerodynamics']['CL_TO']
        self.pre_rho = self.aircraft['environment']['density_kg/m3']
        self.pre_h = self.aircraft['environment']['altitude_m']
        self.pre_A_design = self.aircraft['aerodynamics']['aspect_ratio'] / 2
    
    def set_densities(self):
        for h in self.h:
            isa = ISA_Calculator(self.h)
            self.rho.append(isa.results[h]["Density [kg/m3]"])
    
    def wing_loading(self, V, CL_max, rho=1.225):
        """Calculate Wing Loading for given velocity and maximum lift coefficient in cruise"""
        return 0.5 * rho* V**2 * CL_max
    
    def take_off_loading(self, CL_TO_list=None, rho=1.225):
        """
        Calculate W/P vs W/S curves for an array of CL_TO values.
        """
        if CL_TO_list is None:
            CL_TO_list = [self.CL_max_TO / (1.1 ** 2)]
        
        W_S_range = np.linspace(1, self.x_max, 1000)
        results = []
        
        for CL_TO in CL_TO_list:
            effective_CL = CL_TO / (1.1 ** 2)
            W_P_range = (self.TOP / W_S_range) * effective_CL * (rho/ self.rho0)
            results.append((W_P_range, W_S_range, CL_TO))
        
        return results
    
    def landing_loading(self, CL_max, rho=1.225):
        """Calculate Wing Loading for given velocity and maximum lift coefficient in landing phase"""
        return (CL_max * rho* (self.landing_distance / 0.5915)) / (2 * self.f)
    
    def climb_loading(self, A_list=None, rho=1.225):
        """
        Calculate W/P vs W/S curves for climb performance.
        """
        if A_list is None:
            A_list = [1]
        
        W_S_range = np.linspace(1, self.x_max, 1000)
        results = []
        
        for A in A_list:
            W_P_range = self.prop_setting * (self.n_p * (((rho / self.rho0) ** 0.75) * ((((self.CD0 * 0.5 * rho * (self.V_cruise ** 3)) / (W_S_range)) + ((W_S_range) / (np.pi * A * self.e * 0.5 * rho * self.V_cruise))) ** (-1))))
            results.append((W_P_range, W_S_range, A))
        
        return results
    
    def climb_rate_loading(self, A_list=None, rho=1.225):
        """
        Calculate W/P vs W/S curves for climb rate performance.
        """
        if A_list is None:
            A_list = [1]
        
        W_S_range = np.linspace(1, self.x_max, 1000)
        results = []
        
        for A in A_list:
            W_P_range = ((self.n_p) / (self.climb_rate + ((np.sqrt(W_S_range) * np.sqrt(2 / rho)) / (1.345 * (((A * self.e) ** 0.75) / ((self.CD0) ** 0.25))))))
            results.append((W_P_range, W_S_range, A))
        
        return results

    def climb_gradient_loading(self, A_list=None, rho=1.225):
        """
        Calculate W/P vs W/S curves for climb gradient performance.
        """
        if A_list is None:
            A_list = [1]
        
        W_S_range = np.linspace(1, self.x_max, 1000)
        results = []
        
        for A in A_list:
            CD = self.drag_polar(A=A)
            W_P_range = ((self.n_p) / (np.sqrt(W_S_range) * (self.climb_gradient + (CD / self.CL_max_clean[len(self.CL_max_clean) - 1])) * (np.sqrt((2) / (rho * self.CL_max_clean[len(self.CL_max_clean) - 1])))))
            results.append((W_P_range, W_S_range, A))
        
        return results
    
    def plot_wing_loading_constraints(self, CL_TO_list=None):
        """Plot Wing Loading Constraints for Stall, Cruise, Takeoff, etc. across up to 4 altitudes as subplots."""
        
        num_altitudes = min(len(self.rho), 3)
        fig, axs = plt.subplots(1, num_altitudes, figsize=(6 * num_altitudes, 6), sharey=True)
        axs = np.atleast_1d(axs)  # Ensure axs is iterable even for 1 subplot

        for idh, rho in enumerate(self.rho[:num_altitudes]):  # Limit to 4 plots
            ax = axs[idh]

            # Stall constraint
            stall_lines = []
            for idx, CL in enumerate(self.CL_max_land):
                WL_stall = self.wing_loading(self.V_stall, CL, rho=rho)
                stall_lines.append(WL_stall)
                color = f"C{idx % 10}"
                ax.axvline(x=WL_stall, color=color, linestyle='--', label=f"Stall CL={CL:.2f} @ {round(WL_stall, 2)}")
            WL_stall_limit = min(stall_lines)
            ax.axvspan(WL_stall_limit, self.x_max, color='red', alpha=0.15)

            # Cruise constraint
            cruise_lines = []
            for idx, CL in enumerate(self.CL_max_clean):
                WL_cruise = self.wing_loading(self.V_cruise, CL, rho=rho)
                cruise_lines.append(WL_cruise)
                color = f"C{idx % 10 + 1}"
                ax.axvline(x=WL_cruise, color=color, linestyle='--', label=f"Cruise CL={CL:.2f} @ {round(WL_cruise, 2)}")
            WL_cruise_limit = min(cruise_lines)
            ax.axvspan(WL_cruise_limit, self.x_max, color='red', alpha=0.15)

            # Landing constraint
            for idx, CL in enumerate(self.CL_max_land):
                WL_land = self.landing_loading(CL, rho=rho)
                color = f"C{idx % 10 + 2}"
                ax.axvline(x=WL_land, color=color, linestyle='--', label=f"Landing CL={CL:.2f} @ {round(WL_land, 2)}")
            ax.axvspan(WL_land, self.x_max, color='red', alpha=0.15)

            # Take-off constraint
            takeoff_curves = self.take_off_loading(CL_TO_list=self.CL_max_TO, rho=rho)
            for idx, (W_P_range, W_S_range, CL_TO) in enumerate(takeoff_curves):
                label = f"Takeoff CL_TO={CL_TO:.2f}"
                color = f"C{idx % 10 + 3}"
                ax.plot(W_S_range, W_P_range, label=label, color=color)
            ax.fill_between(W_S_range, W_P_range, self.y_max, color='red', alpha=0.15)

            # Climb constraint
            climb_aspect_curves = self.climb_loading(A_list=self.A_design, rho=rho)
            for idx, (W_P_range, W_S_range, A) in enumerate(climb_aspect_curves):
                label = f"Climb A ={A:.0f}"
                color = f"C{idx % 10 + 4}"
                ax.plot(W_S_range, W_P_range, label=label, color=color)
            ax.fill_between(W_S_range, W_P_range, self.y_max, color='red', alpha=0.15)

            # Climb rate constraint
            climb_rate_aspect_curves = self.climb_rate_loading(A_list=self.A_design, rho=rho)
            for idx, (W_P_range, W_S_range, A) in enumerate(climb_rate_aspect_curves):
                label = f"Climb Rate A ={A:.0f}"
                color = f"C{idx % 10 + 5}"
                ax.plot(W_S_range, W_P_range, label=label, color=color)
            ax.fill_between(W_S_range, W_P_range, self.y_max, color='red', alpha=0.15)

            # Climb gradient constraint
            climb_gradient_aspect_curves = self.climb_gradient_loading(A_list=self.A_design, rho=rho)
            for idx, (W_P_range, W_S_range, A) in enumerate(climb_gradient_aspect_curves):
                label = f"Climb Gradient A ={A:.0f}"
                color = f"C{idx % 10 + 6}"
                ax.plot(W_S_range, W_P_range, label=label, color=color)
            ax.fill_between(W_S_range, W_P_range, self.y_max, color='red', alpha=0.15)

            # Axis setup
            ax.set_xlim(0, self.x_max)
            ax.set_ylim(0, self.y_max)
            ax.set_xlabel("Wing Loading W/S (N/m²)")
            if idh == 0:
                ax.set_ylabel("Weight-to-Power W/P (N/W)")
            ax.set_title(f"Altitude: {self.h[idh]} m")
            ax.grid(True)
            ax.legend(fontsize='small', loc='upper right')
            
            ax.axhline(y=self.aircraft['performance']['power_loading_N/W'], color='black', linestyle='-', label=f"Power Loading W/P={CL:.2f}")
            ax.axvline(x=self.aircraft['aerodynamics']['wing_loading_N/m2'], color='black', linestyle='-', label=f"Wing Loading W/S={CL:.2f}")

        plt.suptitle("Wing Loading vs Weight-to-Power Across Altitudes", fontsize=16)
        plt.tight_layout(rect=[0, 0, 1, 0.95])
        if self.plot:
            plt.show()

    def drag_polar(self, A=0):
        return (self.CD0 + (self.CL_max_clean[len(self.CL_max_clean) - 1] ** 2) / (np.pi * A * self.e))  # Drag Coefficient

# Example Usage
if __name__ == "__main__":
    wpws = WP_WS_Diagram(
        
        plot=True,
        x_max=150,
        y_max=0.3
        
    )
    wpws.plot_wing_loading_constraints()

