from parapy.core import Base, Input, Attribute
from ISA_calculator import ISA_calculator, ISA_altitude_from_density
# from Components.Wing import Wing
import numpy as np
import metric_imperial_conversions as m2i
from Pythonfiles.main import engine_type
from Pythonfiles.WP_WS_diagram import WP_WS_Diagram


class Mission(Base):
    #---------Inputs---------------
    mission_altitude: float = Input()
    mission_range: float = Input()
    mission_endurance: float = Input()
    payload_weight: float = Input()
    specific_fuel: float = Input()
    maximum_mach: float = Input()
    prop_efficiency: float = Input()
    cruise_speed: float = Input()
    loiter_speed: float = Input()
    mission_objective: str = Input()
    oswald_factor: float = Input(0.8)
    reserve_time: float = Input(0.5)






    # climb_speed: float = Input()
    # wing_loading: float = Input()



    # MTOW: float = Input()
    # aspect_ratio: float = Input()
    # thrust_loading: float = Input()
    # cd_0: float = Input()
    # cd_0_loiter: float = Input().

    def engine_selection(self):
        a = ISA_calculator(self.mission_altitude)[3]
        if self.cruise_speed / a > 0.4:
            self.engine_type = "Jet"
        elif self.cruise_speed / a <= 0.4:
            if self.mission_objective == "High Endurance":
                self.engine_type = "Turboprop"
            elif self.mission_objective == "Low cost" or "Low weight":
                self.engine_type = "Piston"
        return self.engine_type

    def take_off_weight_estimate(self, tol=0.01, max_iter=100):
        """
        Estimate takeoff weight, iterated until W0 converges.
        Inputs in SI, internal calculation in imperial (Raymer).
        """
        cruise_range = self.mission_range / 2
        Wp = m2i.kilograms_to_pounds(self.payload_weight)
        Rc = m2i.kilometers_to_nautical_miles(cruise_range)
        T_loit = self.mission_endurance
        Vc = m2i.meter_per_second_to_knots(self.cruise_speed)
        Vl = m2i.meter_per_second_to_knots(self.loiter_speed)
        prop_efficiency = 0.8
        self.engine_selection()

        if self.engine_type == "Jet":
            empty_weight_guess = 600
            LDc = 13  # cruise L/D higher for jets
            LDl = 11  # loiter L/D lower for jets
            SFC_cruise = 0.8
            SFC_loiter = 0.7
            A = 1.67
            C = -0.16
        elif self.engine_type == "Turboprop":
            empty_weight_guess = 400
            LDc = 23  # cruise L/D
            LDl = 28  # loiter L/D higher for props (near best L/D speed)
            SFC_cruise = 0.4
            SFC_loiter = 0.5
            A = 2.75
            C = -0.18
        elif self.engine_type == "Piston":
            empty_weight_guess = 100
            LDc = 13
            LDl = 15  # loiter L/D higher for props
            SFC_cruise = 0.4
            SFC_loiter = 0.5
            A = 0.97
            C = -0.06

        # Fixed mission segment fractions
        W10 = 0.97
        W21 = 0.985
        W65 = 0.995

        # Fuel-dependent fractions (computed once, don't change with W0 guess)
        if self.engine_type == "Jet":
            W32 = W54 =  np.exp((-Rc * (SFC_cruise/3600)) / (Vc * LDc))
            W43 = np.exp(-T_loit * SFC_loiter / LDl)
        elif self.engine_type == "Turboprop" or "Piston":
            W32 = W54 = np.exp((-Rc * SFC_cruise) / (prop_efficiency * LDc))
            W43 = np.exp(-T_loit * Vl * (SFC_loiter / 3600) / (prop_efficiency * LDl))

        wf_w0 = 1.06 * (1 - (W10 * W21 * W32 * W43 * W54 * W65))

        # Iteration on W0_guess
        W0_guess = m2i.kilograms_to_pounds(empty_weight_guess)

        for i in range(max_iter):
            W0_We = A * (W0_guess ** C)
            W0_lbs = Wp / (1 - wf_w0 - W0_We)
            W0_kg = m2i.pounds_to_kilograms(W0_lbs)

            if abs(W0_lbs - W0_guess) / W0_guess < tol:
                print(f"Converged in {i + 1} iterations")
                break

            W0_guess = 0.5 * W0_guess + 0.5 * W0_lbs # feed W0 back as the new guess
        else:
            print(f"Warning: did not converge after {max_iter} iterations")

        We = W0_kg * (1 - wf_w0) - m2i.pounds_to_kilograms(Wp)
        Wf = W0_kg * wf_w0
        print(f"W0_kg: {W0_kg}, We: {We}, Wf: {Wf}")

        return W0_kg, We, Wf

    def calc_aspect_ratio(self):
        if self.engine_type == "Jet":
            a, C = 4.737, -0.979
            self.aspect_ratio = a * self.maximum_mach ** C
        elif self.engine_type == "Turboprop":
            self.aspect_ratio = 9.2
        elif self.engine_type == "Piston":
            self.aspect_ratio = 7.6
        return self.aspect_ratio

    # def estimate_wing_parameters(self):
    #     if self.engine_type == "Jet":
    #         return [1.2, 1.5, 1.7, 0.015] # CL_max_clean, CL_max_TO, CL_max_land, CD0
    #     elif self.engine_type == "Turboprop":
    #         return [1.4, 1.7, 2.0, 0.02]
    #     elif self.engine_type == "Piston":
    #         return [1.3, 1.5, 1.8, 0.02]

    def estimate_wing_parameters(self):
        if self.engine_type == "Jet":
            self.CL_max_clean, self.CL_max_TO, self.CL_max_land = 1.2, 1.5, 1.7
            self.CD0 = 0.015
        elif self.engine_type == "Turboprop":
            self.CL_max_clean, self.CL_max_TO, self.CL_max_land = 1.4, 1.7, 2.0
            self.CD0 = 0.02
        elif self.engine_type == "Piston":
            self.CL_max_clean, self.CL_max_TO, self.CL_max_land = 1.3, 1.5, 1.8
            self.CD0 = 0.02
        return self.CL_max_clean, self.CL_max_TO, self.CL_max_land, self.CD0


    # def calc_thrust_to_weight(self):
    #     a = ISA_calculator(self.mission_altitude)[3]
    #     max_velocity = self.maximum_mach * a
    #     fuel_fraction_to_cruise = 0.97 * 0.985
    #     if self.engine_type == "Jet":
    #         TO_thrust_over_cruise_thrust = 1 / 0.4
    #         a, C = 0.488, 0.728
    #         cruise_thrust_to_weight = a * self.maximum_mach ** C
    #         self.thrust_to_weight = cruise_thrust_to_weight * fuel_fraction_to_cruise * TO_thrust_over_cruise_thrust
    #     elif self.engine_type == "Turboprop":
    #         TO_thrust_over_cruise_thrust = 1 / 0.6
    #         a, C = 0.013, 0.50
    #         cruise_thrust_to_weight = a * max_velocity ** C
    #         self.thrust_to_weight = cruise_thrust_to_weight * fuel_fraction_to_cruise * TO_thrust_over_cruise_thrust
    #     elif self.engine_type == "Piston":
    #         TO_thrust_over_cruise_thrust = 1 / 0.75
    #         a, C = 0.025, 0.22
    #         cruise_thrust_to_weight = a * max_velocity ** C
    #         self.thrust_to_weight = cruise_thrust_to_weight * fuel_fraction_to_cruise * TO_thrust_over_cruise_thrust
    #     return self.thrust_to_weight

    def thrust_and_wing_loading(self):
        CL_clean, CL_TO, CL_land, CD0 = self.estimate_wing_parameters()

        tw_ws = WP_WS_Diagram(plot=False,
                                aspect_ratio=self.calc_aspect_ratio(),
                                CL_max_TO=CL_TO,
                                CL_max_land=CL_land,
                                CL_max_clean=CL_clean,
                                cruise_speed=self.cruise_speed,
                                CD0=CD0,
                                prop_eff=self.prop_efficiency,
                                engine_type=self.engine_type,
                                oswald_factor=self.oswald_factor,
                                x_max=2000,
                                y_max=1
                                )
        self.density = ISA_calculator(self.mission_altitude)[2]
        W_S_design, y_design = tw_ws.find_design_point(rho=self.density)

        self.wing_loading = W_S_design
        self.thrust_to_weight = y_design

        return W_S_design, y_design


    @Attribute
    def fuel_weight_sizing(self) -> float:
        """Typical mission profile:
        taxi
        takeoff
        climb to altitude
        cruise
        loiter
        cruise back
        reserve fuel
        descend
        landing
        taxi"""
        specific_fuel = self.specific_fuel
        cruise_range = self.mission_range / 2
        speed_of_sound = ISA_calculator(self.mission_altitude)[3]
        loiter_time = self.mission_endurance
        self.MTOW = self.take_off_weight_estimate()[0]
        self.wing_loading = self.thrust_and_wing_loading()[0]
        if self.engine_type == "Jet":
            self.thrust_over_weight = self.thrust_and_wing_loading()[1]
        elif self.engine_type == "Turboprop" or self.engine_type == "Piston":
            self.power_loading = self.thrust_and_wing_loading()[1]
        self.calc_aspect_ratio()
        self.CD0 = self.estimate_wing_parameters()[3]


        #start-up, taxi
        w1_w0 = 0.98
        #climb
        optimal_cruise_speed = self.cruise_speed
        if self.engine_type == "Jet":
            for i in range(10): # Iterates the cruise speed until the old cruise speed and new cruise speed match up
                old_cruise_speed = optimal_cruise_speed
                M = optimal_cruise_speed / speed_of_sound
                w2_w1 = 1.0065 - 0.0325 * M
                optimal_cruise_speed = self.find_optimal_cruise_speed_sizing(w1_w0 * w2_w1)
                if abs(optimal_cruise_speed - old_cruise_speed) < 0.1:
                    self.cruise_speed = optimal_cruise_speed
                    break
        elif self.engine_type == "Turboprop" or self.engine_type == "Piston":
            M = self.cruise_speed / speed_of_sound
            w2_w1 = 1.0065 - 0.0325 * M

        #cruise to loiter
        cruise_wing_loading = self.wing_loading * w1_w0 * w2_w1
        cruise_lift_to_drag = self.calculate_lift_to_drag(cruise_wing_loading, self.CD0, self.cruise_speed,
                                                          self.aspect_ratio, self.mission_altitude, self.oswald_factor)
        if self.engine_type == "Jet":
            w3_w2 = np.exp((-cruise_range * specific_fuel) / (self.cruise_speed * cruise_lift_to_drag))
        elif self.engine_type == "Turboprop" or self.engine_type == "Piston":
            w3_w2 = np.exp((-cruise_range * specific_fuel) / (self.prop_efficiency * cruise_lift_to_drag))

        #loiter
        # loiter_speed = self.find_optimal_loiter_speed_sizing(self, w1_w0 * w2_w1 * w3_w2)
        loiter_wing_loading = self.wing_loading * w1_w0 * w2_w1 * w3_w2
        self.loiter_speed = self.find_optimal_loiter_speed_sizing(loiter_wing_loading)
        loiter_lift_to_drag = self.calculate_lift_to_drag(loiter_wing_loading,
                                                          self.CD0,
                                                          self.loiter_speed,
                                                          self.aspect_ratio,
                                                          self.mission_altitude,
                                                          self.oswald_factor)

        if self.engine_type == "Jet":
            w4_w3 = np.exp(-loiter_time * (self.specific_fuel / 3600) / loiter_lift_to_drag)
        elif self.engine_type == "Turboprop" or self.engine_type == "Piston":
            w4_w3 = np.exp(-loiter_time * self.loiter_speed * (self.specific_fuel / 3600) / (self.prop_efficiency *
                                                                                loiter_lift_to_drag))
        #cruise back
        cruise_back_wing_loading = self.wing_loading * w1_w0 * w2_w1 * w3_w2 * w4_w3
        cruise_back_lift_to_drag = self.calculate_lift_to_drag(cruise_back_wing_loading, self.CD0,
                                                               self.cruise_speed, self.aspect_ratio,
                                                               self.mission_altitude, self.oswald_factor)
        if self.engine_type == "Jet":
            w5_w4 = np.exp((-cruise_range * specific_fuel) / (self.cruise_speed * cruise_back_lift_to_drag))
        elif self.engine_type == "Turboprop" or self.engine_type == "Piston":
            w5_w4 = np.exp((-cruise_range * specific_fuel) / (self.prop_efficiency * cruise_back_lift_to_drag))

        #reserve
        reserve_wing_loading = self.wing_loading * w1_w0 * w2_w1 * w3_w2 * w4_w3 * w5_w4
        reserve_lift_to_drag = self.calculate_lift_to_drag(reserve_wing_loading,
                                                           self.CD0,
                                                           self.loiter_speed,
                                                           self.aspect_ratio,
                                                           self.mission_altitude,
                                                           self.oswald_factor)
        if self.engine_type == "Jet":
            w6_w5 = np.exp(-self.reserve_time * (self.specific_fuel / 3600) / reserve_lift_to_drag)
        elif self.engine_type == "Turboprop" or self.engine_type == "Piston":
            w6_w5 = np.exp(-self.reserve_time * self.loiter_speed * (self.specific_fuel / 3600) /
                           (self.prop_efficiency * reserve_lift_to_drag))

        #landing
        w7_w6 = 0.99

        #taxi
        w8_w7 = 0.992
        #final weight
        print(f'taxi: {w1_w0}\n', f'climb: {w2_w1}\n',
              f'cruise: {w3_w2}\n', f'Loiter: {w4_w3}\n',
              f'cruise back: {w5_w4}\n', f'reserve: {w6_w5}\n',
              f'landing: {w7_w6}\n', f'taxi: {w8_w7}\n')
        wf_w0 = w1_w0 * w2_w1 * w3_w2 * w4_w3 * w5_w4 * w6_w5 * w7_w6 * w8_w7
        print(wf_w0)
        empty_weight = self.MTOW * w1_w0 * w2_w1 * w3_w2 * w4_w3 * w5_w4 * w6_w5 * w7_w6 * w8_w7
        fuel_weight = self.MTOW - empty_weight - self.payload_weight
        return fuel_weight*1.01


    def calculate_lift_to_drag(self, wing_loading: float, Cd_0: float, speed: float,
                               aspect_ratio: float, altitude: float, oswald_factor: float) -> float:
        """Calculates the lift to drag of an aircraft using the formula given by Raymer ch. 6"""
        density = ISA_calculator(altitude)[2]
        q = 0.5 * density * speed ** 2
        return 1 / ((q * Cd_0) / wing_loading + wing_loading * (1 / (q * np.pi * aspect_ratio * oswald_factor)))

    def find_optimal_cruise_speed_sizing(self, weight_fraction):
        """Finds the speed corresponding to the maximum fuel fraction"""
        V_range = np.arange(1, 400, 10)
        res = []
        for item in V_range:
            specific_fuel = self.specific_fuel
            cruise_range = self.mission_range / 2
            cruise_wing_loading = self.wing_loading * weight_fraction
            cruise_lift_to_drag = self.calculate_lift_to_drag(cruise_wing_loading, self.CD0, item, self.aspect_ratio,
                                                              self.mission_altitude, self.oswald_factor)
            w3_w2 = np.exp((-cruise_range * specific_fuel) / (item * cruise_lift_to_drag))
            res.append(w3_w2)
        optimal_cruise = V_range[res.index(max(res))]
        return float(optimal_cruise)

    def find_optimal_loiter_speed_sizing(self, weight_fraction) -> float:
        """Calculates the optimal loiter speed by calculating the fuel fraction for a range of speeds
        then taking the maximum fuel fraction"""
        v_range = np.arange(1, 400, 10)
        res = []
        for item in v_range:
            loiter_time = self.mission_endurance
            loiter_wing_loading = self.wing_loading * weight_fraction
            loiter_lift_to_drag = self.calculate_lift_to_drag(loiter_wing_loading,
                                                              self.CD0,
                                                              item,
                                                              self.aspect_ratio,
                                                              self.mission_altitude,
                                                              self.oswald_factor)
            if self.engine_type == "Jet":
                w4_w3 = np.exp(-loiter_time * (self.specific_fuel / 3600) / loiter_lift_to_drag)
                res.append(w4_w3)
            elif self.engine_type == "Turboprop" or self.engine_type == "Piston":
                w4_w3 = np.exp(-loiter_time * item * (self.specific_fuel / 3600) / (self.prop_efficiency *
                                                                                    loiter_lift_to_drag))
                res.append(w4_w3)
        optimal_loiter = v_range[res.index(max(res))]
        return float(optimal_loiter)



    # @Attribute
    # def fuel_weight_final(self) -> float:
    #     """Your existing fuel_weight logic, parameterized by L/D values."""
    #
    #     specific_fuel = self.specific_fuel
    #     cruise_range = self.mission_range / 2
    #     speed_of_sound = ISA_calculator(self.mission_altitude)[3]
    #     loiter_time = self.mission_endurance
    #
    #     # start-up, taxi
    #     w1_w0 = 0.98
    #
    #     # climb
    #     M = self.cruise_speed / speed_of_sound
    #     w2_w1 = 1.0065 - 0.0325 * M
    #
    #     # cruise to loiter
    #     cruise_range = self.mission_range / 2
    #     cruise_lift_to_drag = Wing.lift_to_drag_cruise
    #
    #     if self.engine_type == "Jet":
    #         w3_w2 = np.exp((-cruise_range * specific_fuel) /
    #                        (self.cruise_speed * cruise_lift_to_drag))
    #     else:  # Propeller-like
    #         w3_w2 = np.exp((-cruise_range * specific_fuel) /
    #                        (self.prop_efficiency * cruise_lift_to_drag))
    #
    #     # loiter
    #     loiter_lift_to_drag = Wing.lift_to_drag_loiter
    #
    #     if self.engine_type == "Jet":
    #         w4_w3 = np.exp(-loiter_time * (self.specific_fuel / 3600) /
    #                        loiter_lift_to_drag)
    #     else:  # Propeller-like
    #         w4_w3 = np.exp(-loiter_time * self.loiter_speed * (self.specific_fuel / 3600) /
    #                        (self.prop_efficiency * loiter_lift_to_drag))
    #
    #     # cruise back – use same cruise L/D
    #     if self.engine_type == "Jet":
    #         w5_w4 = np.exp((-cruise_range * specific_fuel) /
    #                        (self.cruise_speed * cruise_lift_to_drag))
    #     else:
    #         w5_w4 = np.exp((-cruise_range * specific_fuel) /
    #                        (self.prop_efficiency * cruise_lift_to_drag))
    #
    #     # reserve – use reserve L/D (often same as loiter)
    #     reserve_lift_to_drag = Wing.lift_to_drag_loiter
    #
    #     if self.engine_type == "Jet":
    #         w6_w5 = np.exp(-self.reserve_time * (self.specific_fuel / 3600) /
    #                        reserve_lift_to_drag)
    #     else:
    #         w6_w5 = np.exp(-self.reserve_time * self.loiter_speed * (self.specific_fuel / 3600) /
    #                        (self.prop_efficiency * reserve_lift_to_drag))
    #
    #     # landing
    #     w7_w6 = 0.99
    #     # taxi
    #     w8_w7 = 0.992
    #
    #     final_weight = (self.MTOW_sizing * w1_w0 * w2_w1 * w3_w2 *
    #                     w4_w3 * w5_w4 * w6_w5 * w7_w6 * w8_w7)
    #     fuel_weight = self.MTOW_sizing - final_weight - self.payload_weight
    #     return fuel_weight * 1.01


if __name__ == "__main__":

    # --- Test 1: Turboprop MALE-class drone ---
    mission_tp = Mission(
        mission_altitude=6000,
        mission_range=10,
        mission_endurance=8,
        payload_weight=100,
        specific_fuel=0.5,
        maximum_mach=0.5,
        prop_efficiency=0.8,
        cruise_speed=80,
        loiter_speed=57,
        mission_objective='High Endurance',
        oswald_factor=0.8,
        reserve_time=0.5,
    )
    mission_tp.engine_selection()        # sets self.engine_type
    mission_tp.estimate_wing_parameters() # sets CL_max_TO, CL_max_land, CL_max_clean, CD0
    print(f"Engine type:    {mission_tp.engine_type}")
    # print(f"CL_max_clean:   {mission_tp.CL_max_clean}")
    # print(f"CL_max_TO:      {mission_tp.CL_max_TO}")
    # print(f"CL_max_land:    {mission_tp.CL_max_land}")
    # print(f"CD0:            {mission_tp.CD0}")
    fuel_weight = mission_tp.fuel_weight_sizing
    print(f"MTOW:        {mission_tp.MTOW}")
    print(f"fuel weight: {fuel_weight}")


    # # --- Test 2: Jet drone ---
    # mission_jet = Mission(
    #     mission_altitude=8000,
    #     mission_range=1000,
    #     mission_endurance=5,
    #     payload_weight=150,
    #     specific_fuel=0.8,
    #     maximum_mach=0.8,
    #     prop_efficiency=1.0,            # unused for jet, placeholder
    #     cruise_speed=220,
    #     loiter_speed=150,
    #     mission_objective='Long range',
    #     oswald_factor=0.8,
    #     reserve_time=0.5,
    # )
    # mission_jet.engine_selection()
    # mission_jet.estimate_wing_parameters()
    # print(f"\nEngine type:    {mission_jet.engine_type}")
    # # print(f"CL_max_clean:   {mission_jet.CL_max_clean}")
    # # print(f"CL_max_TO:      {mission_jet.CL_max_TO}")
    # # print(f"CL_max_land:    {mission_jet.CL_max_land}")
    # # print(f"CD0:            {mission_jet.CD0}")
    # fuel_weight = mission_jet.fuel_weight_sizing
    # print(f"MTOW:        {mission_jet.MTOW}")
    # print(f"fuel weight: {fuel_weight}")
    #
    # # --- Test 3: Piston small drone ---
    # mission_piston = Mission(
    #     mission_altitude=3000,
    #     mission_range=10,
    #     mission_endurance=10,
    #     payload_weight=50,
    #     specific_fuel=0.4,
    #     maximum_mach=0.3,
    #     prop_efficiency=0.8,
    #     cruise_speed=60,
    #     loiter_speed=30,
    #     mission_objective='Low cost',
    #     oswald_factor=0.8,
    #     reserve_time=0.5,
    # )
    # mission_piston.engine_selection()
    # mission_piston.estimate_wing_parameters()
    # print(f"\nEngine type:    {mission_piston.engine_type}")
    # # print(f"CL_max_clean:   {mission_piston.CL_max_clean}")
    # # print(f"CL_max_TO:      {mission_piston.CL_max_TO}")
    # # print(f"CL_max_land:    {mission_piston.CL_max_land}")
    # # print(f"CD0:            {mission_piston.CD0}")
    # fuel_weight = mission_piston.fuel_weight_sizing
    # print(f"MTOW:        {mission_piston.MTOW}")
    # print(f"fuel weight: {fuel_weight}")


