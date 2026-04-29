from parapy.core import Base, Input
from ISA_calculator import ISA_calculator, ISA_altitude_from_density
import numpy as np

class Mission(Base):
    #---------Inputs---------------
    mission_altitude: float = Input()
    mission_range: float = Input()
    mission_endurance: float = Input()
    cruise_speed: float = Input()
    climb_speed: float = Input()
    engine_type: str = Input()
    specific_fuel: float = Input()
    wing_loading: float = Input()
    oswald_factor: float = Input()
    prop_efficiency: float = Input()
    reserve_time: float = Input()
    MTOW: float = Input()
    payload_weight: float = Input()



    def calculate_empty_weight_fraction(self) -> float:
        """returns the empty weight fraction for a given aspect ratio, thrust_loading, wing_loading, maximum_mach
        crew weight, fixed payload weight, dropped payload weight, fuel weight and empty weight."""
        if self.engine_type == "Propeller":
            a = -0.25
            b = 1.18
            C1 = -0.2
            C2 = 0.08
            C3 = 0.05
            C4 = -0.05
            C5 = 0.20
        elif self.engine_type == "Jet":
            a = 0
            b = 4.28
            C1 = -0.1
            C2 = 0.1
            C3 = 0.2
            C4 = -0.24
            C5 = 0.11
        empty_weight_fraction = a + b * \
                                self.take_off_weight ** C1 * \
                                self.aspect_ratio ** C2 * \
                                self.thrust_loading ** C3 * \
                                self.wing_loading ** C4 * \
                                self.maximum_mach ** C5
        return empty_weight_fraction

    def calculate_fuel_weight(self) -> float:
        """Typical mission profile:
        taxi
        takeoff
        climb to altitude
        cruise
        loiter
        cruise back
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
                optimal_cruise_speed = self.find_optimal_cruise_speed_jet(w1_w0 * w2_w1)
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
        loiter_speed = self.find_optimal_loiter_speed(self, w1_w0 * w2_w1 * w3_w2)
        loiter_wing_loading = self.wing_loading * w1_w0 * w2_w1 * w3_w2
        loiter_lift_to_drag = self.calculate_lift_to_drag(loiter_wing_loading,
                                                     self.cd_0_loiter,
                                                     loiter_speed,
                                                     self.aspect_ratio,
                                                     self.mission_altitude,
                                                     self.oswald_factor)

        if self.engine_type == "Jet":
            w4_w3 = np.exp(-loiter_time * (self.specific_fuel / 3600) / loiter_lift_to_drag)
            res.append(w4_w3)
        elif self.engine_type == "Propeller":
            w4_w3 = np.exp(-loiter_time * item * (self.specific_fuel / 3600) / (self.prop_efficiency *
                                                                                loiter_lift_to_drag))
        #cruise back
        cruise_back_wing_loading = self.wing_loading * w1_w0 * w2_w1 * w3_w2 * w4_w3
        cruise_back_lift_to_drag = self.calculate_lift_to_drag(self, cruise_back_wing_loading, self.cd_0,
                                                               self.cruise_speed, self.aspect_ratio,
                                                               self.mission_altitude, self.oswald_factor)
        if self.engine_type == "Jet":
            w5_w4 = np.exp((-cruise_range * specific_fuel) / (optimal_cruise_speed * cruise_back_lift_to_drag))
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

    def find_optimal_cruise_speed_jet(self, weight_fraction):
        """Finds the speed corresponding to the maximum fuel fraction"""
        V_range = np.arange(1, 1000, 10)
        res = []
        for item in V_range:
            specific_fuel = self.specific_fuel
            cruise_range = self.mission_range / 2
            cruise_wing_loading = self.wing_loading * weight_fraction
            cruise_lift_to_drag = self.calculate_lift_to_drag(cruise_wing_loading, self.Cd_0, item, self.aspect_ratio,
                                                              self.mission_altitude, self.oswald_factor)
            w3_w2 = np.exp(
                (-cruise_range * specific_fuel) / (item * cruise_lift_to_drag))
            res.append(w3_w2)
        optimal_cruise = V_range[res.index(max(res))]
        return float(optimal_cruise)

    def find_optimal_loiter_speed(self, weight_fraction) -> float:
        """Calculates the optimal loiter speed by calculating the fuel fraction for a range of speeds
        then taking the maximum fuel fraction"""
        v_range = np.arange(1, 1000, 10)
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


