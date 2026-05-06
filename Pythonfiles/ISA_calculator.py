import numpy as np

"""ISA CALCULATOR"""

def ISA_calculator(h):
    """Calculate the temperature, pressure, density, speed of sound and density altitude at a given altitude.	
    Troposphere is up to 11000m (appr. 36000 ft) after that it is tropopause, where conditions are constant."""
    # h: altitude [m]
    # returns: temperature [K], pressure [Pa], density [kg/m^3]
    # Constants
    T0 = 288.15 # ISA SL temperature [K]
    p0 = 101325 # ISA SL pressure [Pa]
    rho0 = 1.225 # ISA SL density [kg/m^3]
    g = 9.81 # gravity [m/s^2]
    R = 287.05 # specific gas constant [J/kgK]
    L = 0.0065 # temperature lapse rate [K/m]
    h_tropopause = 11000 # Tropopause altitude [m]
    T_tropopause = T0 - L * h_tropopause  # Temperature at the tropopause [K]
    p_tropopause = p0 * (1 - L * h_tropopause / T0)**(g / (R * L))  # Pressure at the tropopause [Pa]
    if h <= h_tropopause:
        # Troposphere
        T = T0 - L * h
        p = p0 * (1 - L * h / T0)**(g / (R * L))
        rho = rho0 * (1 - L * h / T0)**((g / (R * L)) - 1)
    else:
        # Stratosphere (isothermal layer)
        T = T_tropopause  # Constant temperature
        p = p_tropopause * np.exp(-g * (h - h_tropopause) / (R * T))
        rho = p / (R * T)

    a = np.sqrt(1.4 * R * T) # speed of sound [m/s]
    DA = p / (R * T) # density altitude [m]
    return T, p, rho, a, DA

import numpy as np

def ISA_altitude_from_density(rho):
    """Calculate the altitude given the air density (rho) using the ISA model.
    
    Args:
        rho: Air density [kg/m^3]
    
    Returns:
        h: Altitude [m]
    """
    # Constants
    T0 = 288.15  # ISA SL temperature [K]
    rho0 = 1.225  # ISA SL density [kg/m^3]
    g = 9.81  # gravity [m/s^2]
    R = 287.05  # specific gas constant [J/kgK]
    L = 0.0065  # temperature lapse rate [K/m]
    
    # Exponent for the density equation
    exponent = 1 / ((g / (R * L)) - 1)
    
    # Calculate altitude
    h = (T0 / L) * (1 - (rho / rho0)**exponent)
    return h