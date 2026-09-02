from typing import Final
import math

# simulation constants
g = -9.81           # gravitational acceleration (m/s^2)
L = .3079/2          # length of the pendulum (m)
m = .011             # mass of the pendulum (kg)
M = .052             # mass of the cart (kg)
W = .0381            # cart width (m)
H = .05              # cart height (m)
wr = .015            # wheel radius (m)
mr = .015            # mass radius (m)
d = 0.005            # damping coeff
b = 0.001            # pendulum pivot/bearing friction (N*m*s/rad)
cd_cart = 0.0001     # cart aerodynamic drag coeff, quadratic in v (N*s^2/m^2)
cd_pend = 0.0001     # pendulum aerodynamic drag coeff, quadratic in w (N*m*s^2/rad^2)

# motor / drive constants
r_belt = 0.006                              # belt/pulley drive radius (m)
rpm_max = 1000                               # motor rated speed (RPM)
tau_rated = 0.15 * 9.80665 * 0.01           # kgf*cm -> N*m
u_max = tau_rated / r_belt                   # rated continuous force ceiling (N)
v_max = (2 * math.pi * rpm_max / 60) * r_belt  # max cart speed at rated RPM (m/s)

# rail / soft-limit constants
x_min = -0.5      # left end of rail (m)
x_max = 0.5       # right end of rail (m)
x_margin = 0.05   # distance from rail end where soft wall starts (m)
k_wall = 400      # soft-wall spring stiffness (N/m)
c_wall = 20       # soft-wall damping (N*s/m)

# hardware constants
AS5600_RAW_TO_RADIANS = (math.pi * 2.0) / 4096
AS5600_RAW_TO_DEGREES = 360.0 / 4096