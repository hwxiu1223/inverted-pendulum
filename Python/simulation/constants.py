from typing import Final
import math

# simulation constants
g = -9.81            # gravitational acceleration (m/s^2)
L = .132             # length of the pendulum (m)
m = .015             # mass of the pendulum (kg)
M = .100             # mass of the cart (kg)
W = .0381            # cart width (m)
H = .05              # cart height (m)
wr = .015            # wheel radius (m)
mr = .015            # mass radius (m)
d = 5e-4             # damping coeff
b = 1e-6             # pendulum pivot/bearing friction (N*m*s/rad)
cd_cart = 1e-4       # cart aerodynamic drag coeff, quadratic in v (N*s^2/m^2)
cd_pend = 1e-6       # pendulum aerodynamic drag coeff, quadratic in w (N*m*s^2/rad^2)

# motor / drive constants
r_belt = 0.006                              # belt/pulley drive radius (m)
rpm_max = 1000                               # motor rated speed (RPM)
tau_rated = 0.15 * 9.80665 * 0.01           # kgf*cm -> N*m (0.15 kgf*cm = 0.01471 N*m)
u_max = 2.55                                 # rated continuous force ceiling (N) ~= 2.45 N (tau_rated / r_belt)
v_max = (2 * math.pi * rpm_max / 60) * r_belt  # max cart speed at rated RPM (m/s) ~= 0.628 m/s

# rail / soft-limit constants
x_min = -0.48     # available space on rail to left (m)
x_max = 0.48      # available space on rail to right (m)
x_margin = 0.08   # distance from the end where the soft wall starts pushing back (m)
k_wall = 10       # soft-wall spring stiffness (N/m)
c_wall = 2        # soft-wall damping (N*s/m)

# sensor & process noise, used by the lqg design
sigma_x = 5e-4       # cart position encoder noise, 1-sigma (m)   ~0.5 mm
sigma_theta = 1e-2   # pendulum angle encoder noise, 1-sigma (rad) ~0.57 deg

# process noise intensity: how much we distrust the model's prediction
# of the (unmeasured) velocity states between samples
q_dx = 1e-4       # process noise on cart velocity state
q_dtheta = 1e-3   # process noise on pendulum angular velocity state

# ---------------------------------------------------------------------------
# hardware encoder calibration
# ---------------------------------------------------------------------------
# AS5600 magnetic encoder on the pendulum pivot: 12-bit, 0-4095 raw counts
# per revolution.
AS5600_COUNTS_PER_REV = 4096
AS5600_RAW_TO_RADIANS = (2 * math.pi) / AS5600_COUNTS_PER_REV
AS5600_RAW_TO_DEGREES = 360.0 / AS5600_COUNTS_PER_REV

# Raw AS5600 reading when the pendulum hangs straight down at rest, which is
# theta = 0 in the sim/controller convention (theta = pi is the upright
# balance point). TODO: calibrate on your hardware -- let the pendulum hang
# freely, call cart.read_encoders(), and set this to the mag_val you get.
AS5600_ZERO_RAW = 0

# Rotary (cart position) encoder: quadrature counts per revolution of the
# pulley/belt shaft. TODO: verify against your actual encoder's spec.
ROTARY_COUNTS_PER_REV = 4096

# Raw rotary count corresponding to x = 0 (rail center). center_cart()
# already homes to this value, so it doubles as the position zero-reference.
ROTARY_CENTER = 30000

# Meters of cart travel per encoder count, derived from the belt/pulley
# radius. TODO: verify against your actual pulley/encoder mounting -- this
# assumes one shaft revolution = one belt revolution (2*pi*r_belt of travel).
METERS_PER_COUNT = (2 * math.pi * r_belt) / ROTARY_COUNTS_PER_REV

# ---------------------------------------------------------------------------
# motor command mapping
# ---------------------------------------------------------------------------
# hybrid_controller_lqg outputs a force command u in [-u_max, u_max] (N).
# The Arduino motor driver takes an integer command in [-255, 255].
MOTOR_MAX = 255

# +u (positive force) maps to a *negative* motor command on this hardware.
# Flip this to +1 if your wiring/motor direction is the other way -- an easy
# way to check is: command a small positive motor value and confirm the cart
# moves in the +x direction (toward x_max).
MOTOR_SIGN = -1