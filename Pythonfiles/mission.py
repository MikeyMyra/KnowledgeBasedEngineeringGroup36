from parapy.core import Base, Input, Attribute
from ISA_calculator import ISA_calculator, ISA_altitude_from_density
# from Components.Wing import Wing
import numpy as np
import metric_imperial_conversions as m2i

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








    # climb_speed: float = Input()
    # wing_loading: float = Input()
    # oswald_factor: float = Input()

    # reserve_time: float = Input()
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
            self.engine_type = "propeller"

    def take_off_weight_estimate(self, empty_weight_guess, tol=0.01, max_iter=100):
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

        if self.engine_type == "Jet":
            LDc = 13  # cruise L/D higher for jets
            LDl = 11  # loiter L/D lower for jets
            SFC_cruise = 0.8
            SFC_loiter = 0.7
            A = 1.67
            C = -0.16
        elif self.engine_type == "Propeller":
            if self.mission_objective == "Endurance"
                LDc = 23  # cruise L/D
                LDl = 28  # loiter L/D higher for props (near best L/D speed)
                SFC_cruise = 0.4
                SFC_loiter = 0.5
                A = 2.75
                C = -0.18
            elif self.mission_objective == "Low cost":
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
        elif self.engine_type == "Propeller":
            W32 = W54 = np.exp((-Rc * SFC_cruise) / (prop_efficiency * LDc))
            W43 = np.exp(-T_loit * Vl * (SFC_loiter / 3600) / (prop_efficiency *LDl))

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

        return W0_kg, We, Wf


    # def calculate_empty_weight_fraction(self) -> float:
    #     """returns the empty weight fraction for a given aspect ratio, thrust_loading, wing_loading, maximum_mach,
    #      fixed payload weight,fuel weight and empty weight."""
    #     if self.engine_type == "Propeller":
    #         a = -0.25
    #         b = 1.18
    #         C1 = -0.2
    #         C2 = 0.08
    #         C3 = 0.05
    #         C4 = -0.05
    #         C5 = 0.20
    #     elif self.engine_type == "Jet":
    #         a = 0
    #         b = 4.28
    #         C1 = -0.1
    #         C2 = 0.1
    #         C3 = 0.2
    #         C4 = -0.24
    #         C5 = 0.11
    #     empty_weight_fraction = a + b * \
    #                             self.MTOW ** C1 * \
    #                             self.aspect_ratio ** C2 * \
    #                             self.thrust_loading ** C3 * \
    #                             self.wing_loading ** C4 * \
    #                             self.maximum_mach ** C5
    #     return empty_weight_fraction

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
        elif self.engine_type == "Propeller":
            M = optimal_cruise_speed / speed_of_sound
            w2_w1 = 1.0065 - 0.0325 * M

        #cruise to loiter
        cruise_wing_loading = self.wing_loading * w1_w0 * w2_w1
        cruise_lift_to_drag = self.calculate_lift_to_drag(self, cruise_wing_loading, self.cd_0, self.cruise_speed,
                                                          self.aspect_ratio, self.mission_altitude, self.oswald_factor)
        if self.engine_type == "Jet":
            w3_w2 = np.exp((-cruise_range * specific_fuel) / (optimal_cruise_speed * cruise_lift_to_drag))
        elif self.engine_type == "Propeller":
            w3_w2 = np.exp((-cruise_range * specific_fuel) / (self.prop_efficiency * cruise_lift_to_drag))

        #loiter
        loiter_speed = self.find_optimal_loiter_speed_sizing(self, w1_w0 * w2_w1 * w3_w2)
        loiter_wing_loading = self.wing_loading * w1_w0 * w2_w1 * w3_w2
        loiter_lift_to_drag = self.calculate_lift_to_drag(loiter_wing_loading,
                                                     self.cd_0_loiter,
                                                     loiter_speed,
                                                     self.aspect_ratio,
                                                     self.mission_altitude,
                                                     self.oswald_factor)

        if self.engine_type == "Jet":
            w4_w3 = np.exp(-loiter_time * (self.specific_fuel / 3600) / loiter_lift_to_drag)
        elif self.engine_type == "Propeller":
            w4_w3 = np.exp(-loiter_time * loiter_speed * (self.specific_fuel / 3600) / (self.prop_efficiency *
                                                                                loiter_lift_to_drag))
        #cruise back
        cruise_back_wing_loading = self.wing_loading * w1_w0 * w2_w1 * w3_w2 * w4_w3
        cruise_back_lift_to_drag = self.calculate_lift_to_drag(self, cruise_back_wing_loading, self.cd_0,
                                                               self.cruise_speed, self.aspect_ratio,
                                                               self.mission_altitude, self.oswald_factor)
        if self.engine_type == "Jet":
            w5_w4 = np.exp((-cruise_range * specific_fuel) / (self.cruise_speed * cruise_back_lift_to_drag))
        elif self.engine_type == "Propeller":
            w5_w4 = np.exp((-cruise_range * specific_fuel) / (self.prop_efficiency * cruise_back_lift_to_drag))

        #reserve
        reserve_wing_loading = self.wing_loading * w1_w0 * w2_w1 * w3_w2 * w4_w3 * w5_w4
        reserve_lift_to_drag = self.calculate_lift_to_drag(reserve_wing_loading,
                                                          self.cd_0_loiter,
                                                          loiter_speed,
                                                          self.aspect_ratio,
                                                          self.mission_altitude,
                                                          self.oswald_factor)
        if self.engine_type == "Jet":
            w6_w5 = np.exp(-self.reserve_time * (self.specific_fuel / 3600) / reserve_lift_to_drag)
        elif self.engine_type == "Propeller":
            w6_w5 = np.exp(-self.reserve_time * loiter_speed * (self.specific_fuel / 3600) / (self.prop_efficiency *
                                                                                              reserve_lift_to_drag))

        #landing
        w7_w6 = 0.99

        #taxi
        w8_w7 = 0.992
        #final weight
        print(f'taxi: {w1_w0}\n', f'climb: {w2_w1}\n',
              f'cruise(300NM): {w3_w2}\n', f'Loiter: {w4_w3}\n',
              f'cruise back: {w5_w4}\n', f'reserve: {w6_w5}\n',
              f'landing: {w7_w6}\n', f'taxi: {w8_w7}\n')
        final_weight = self.MTOW * w1_w0 * w2_w1 * w3_w2 * w4_w3 * w5_w4 * w6_w5 * w7_w6 * w8_w7
        fuel_weight = self.MTOW - final_weight - self.payload_weight
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
            cruise_lift_to_drag = self.calculate_lift_to_drag(cruise_wing_loading, self.Cd_0, item, self.aspect_ratio,
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
                                                         self.cd_0_loiter,
                                                         item,
                                                         self.aspect_ratio,
                                                         self.mission_altitude,
                                                         self.oswald_factor)
            if self.engine_type == "Jet":
                w4_w3 = np.exp(-loiter_time * (self.specific_fuel / 3600) / loiter_lift_to_drag)
                res.append(w4_w3)
            elif self.engine_type == "Propeller":
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
    # --- Test case: Turboprop MALE-class drone ---
    mission = Mission(
        mission_altitude=6000,        # m, ~20,000 ft typical MALE altitude
        mission_range=20,               # km, total range (split in half for cruise)
        mission_endurance=100,          # hours loiter
        payload_weight=100,           # kg
        specific_fuel=0.5,            # lb/(hp·hr) for turboprop, Raymer table 3.3
        engine_type="Turboprop",
        maximum_mach=0.5,             # typical turboprop cruise
        prop_efficiency=0.8,          # typical propeller efficiency
        cruise_speed_guess=90,        # m/s ~175 knots
        loiter_speed_guess=57,        # m/s ~110 knots
    )

    empty_weight_guess = 1300 # kg, initial guess

    W0, We, Wf = mission.take_off_weight_estimate(empty_weight_guess)

    print("=== Takeoff Weight Estimate ===")
    print(f"  MTOW  (W0): {W0:.1f} kg")
    print(f"  Empty (We): {We:.1f} kg")
    print(f"  Fuel  (Wf): {Wf:.1f} kg")
    print(f"  Fuel fraction: {Wf/W0:.3f}")
    print(f"  Empty fraction: {We/W0:.3f}")
    print(f"  Payload fraction: {mission.payload_weight/W0:.3f}")
    print(f"  Fractions sum to: {(Wf + We + mission.payload_weight)/W0:.3f}  (should be ~1.0)")

    # --- Sanity check: fractions should sum to 1 ---
    assert abs((Wf + We + mission.payload_weight) / W0 - 1.0) < 0.01, \
        "Weight fractions do not sum to 1 — check unit conversions"

    # --- Second test: Jet drone ---
    mission_jet = Mission(
        mission_altitude=8000,        # m
        mission_range=800,            # km
        mission_endurance=2,          # hours, jets loiter less
        payload_weight=150,           # kg
        specific_fuel=0.8,            # lb/(lbf·hr) for jet
        engine_type="Jet",
        maximum_mach=0.8,
        prop_efficiency=1.0,          # unused for jet, placeholder
        cruise_speed_guess=220,       # m/s ~430 knots
        loiter_speed_guess=150,       # m/s ~290 knots
    )

    W0_j, We_j, Wf_j = mission_jet.take_off_weight_estimate(empty_weight_guess)

    print("\n=== Jet Drone Takeoff Weight Estimate ===")
    print(f"  MTOW  (W0): {W0_j:.1f} kg")
    print(f"  Empty (We): {We_j:.1f} kg")
    print(f"  Fuel  (Wf): {Wf_j:.1f} kg")
    print(f"  Fuel fraction: {Wf_j/W0_j:.3f}")
    print(f"  Empty fraction: {We_j/W0_j:.3f}")


