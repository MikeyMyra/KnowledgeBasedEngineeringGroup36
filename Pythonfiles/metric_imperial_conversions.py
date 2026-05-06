# all conversion functions from metric to imperial and vice versa are defined here


def miles_to_kilometers(miles):
    return miles * 1.60934


def kilometers_to_miles(kilometers):
    return kilometers / 1.60934


def pounds_to_kilograms(pounds):
    return pounds * 0.453592


def kilograms_to_pounds(kilograms):
    return kilograms / 0.453592


def fahrenheit_to_kelvin(fahrenheit):
    return (fahrenheit - 32) * 5 / 9 + 273.15


def kelvin_to_fahrenheit(kelvin):
    return (kelvin - 273.15) * 9 / 5 + 32


def feet_to_meters(feet):
    return feet * 0.3048


def meters_to_feet(meters):
    return meters / 0.3048


def square_feet_to_square_meters(square_feet):
    return square_feet * 0.092903


def square_meters_to_square_feet(square_meters):
    return square_meters / 0.092903


def cubic_feet_to_cubic_meters(cubic_feet):
    return cubic_feet * 0.0283168


def cubic_meters_to_cubic_feet(cubic_meters):
    return cubic_meters / 0.0283168


def pounds_to_newtons(pounds):
    return pounds * 4.44822


def newtons_to_pounds(newtons):
    return newtons / 4.44822


def inches_to_centimeters(inches):
    return inches * 2.54


def centimeters_to_inches(centimeters):
    return centimeters / 2.54


def gallons_to_liters(gallons):
    return gallons * 3.78541


def liters_to_gallons(liters):
    return liters / 3.78541


def ounces_to_grams(ounces):
    return ounces * 28.3495


def grams_to_ounces(grams):
    return grams / 28.3495


def knots_to_mps(knots):
    return knots * 0.514444


def mps_to_knots(mps):
    return mps / 0.514444


def kg_per_cubic_meter_to_pounds_per_cubic_feet(kg_per_cubic_meter):
    return kg_per_cubic_meter * 0.062427961


def pounds_per_cubic_feet_to_kg_per_cubic_meter(pounds_per_cubic_feet):
    return pounds_per_cubic_feet / 0.062427961


def pound_per_hour_per_poundforce_to_kg_per_hour_per_kN(pound_per_hour_per_poundforce):
    return pound_per_hour_per_poundforce * 0.000125998


def kg_per_hour_per_kN_to_pound_per_hour_per_poundforce(kg_per_hour_per_kN):
    return kg_per_hour_per_kN / 0.000125998


# Convert meters to miles
def meters_to_miles(meters):
    return meters / 1609.34


# Convert miles to meters
def miles_to_meters(miles):
    return miles * 1609.34


# Convert meters to nautical miles
def meters_to_nautical_miles(meters):
    return meters / 1852


# Convert nautical miles to meters
def nautical_miles_to_meters(nautical_miles):
    return nautical_miles * 1852


# Convert nautical miles to kilometers
def nautical_miles_to_kilometers(nautical_miles):
    return nautical_miles * 1.852


# Convert kilometers to nautical miles
def kilometers_to_nautical_miles(kilometers):
    return kilometers / 1.852


# Convert bar to psi
def bar_to_psi(bar):
    return bar * 14.5038


# Convert psi to bar
def psi_to_bar(psi):
    return psi / 14.5038


# Convert T/W metric into T/W imperial
def t_w_metric_to_t_w_imperial(w_s_metric):
    return w_s_metric / 48.8243


# Convert T/W imperial into T/W metric
def w_s_imperial_to_w_s_metric(w_s_imperial):
    return w_s_imperial * 48.8243


# Convert kts to m/s
def knots_to_meter_per_second(kts):
    return kts / 1.94384


# Convert m/s to kts
def meter_per_second_to_knots(meter_per_second):
    return 1.94384 * meter_per_second


# Convert N/m^2 to psf
def N_per_m2_to_psf(N_per_m2):
    return N_per_m2 * 0.020885434273039


# Convert kg/m^3 to slugs/ft^3
def kg_per_m3_to_slugs_per_ft3(kg_per_m3):
    return kg_per_m3 * 0.0019403203
