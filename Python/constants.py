from typing import Final

import math

# simulation constants
g = -9.81         # gravitational acceleration (m/s^2)
L = .3079/2       # length of the pendulum (m) length of paint stick is 30.79cm
m = .011          # mass of the pendulum (kg) Mass of pendulum W/o drill holl is 11g. remeasure w/ drill hole
M = .052          # mass of the cart (kg) Mass of cart, dowel, encoder, screws, and nuts: 0.052kg
W = .381          # cart width (m)
H = .5            # cart height (m)
wr = .05          # wheel radius
mr = .1           # mass radius
d = 1             # dampening force
b = 0.02          # pendulum pivot/bearing friction (N*m*s/rad)
cd_cart = 0.05    # cart aerodynamic drag coeff, quadratic in v (N*s^2/m^2)
cd_pend = 0.005   # pendulum aerodynamic drag coeff, quadratic in w (N*m*s^2/rad^2)

# hardware constants
AS5600_RAW_TO_RADIANS = (math.pi * 2.0) / 4096
AS5600_RAW_TO_DEGREES = 360.0 / 4096