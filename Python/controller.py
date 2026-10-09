import math
import numpy as np
from scipy.linalg import solve_continuous_are

# ---------------- actuator (MEASURE THESE with motor_id.py) ----------------
TAU = 0.032          # s, cart speed time constant
K_V = 0.00305         # (m/s) per PWM count above the deadband
DEADBAND = 24        # PWM at which the cart starts moving from rest
MOTOR_MAX = 255
MOTOR_SIGN = -1      # negative PWM -> cart count increases (+x), per your logs
V_MAX = K_V * (MOTOR_MAX - DEADBAND)
V_EPS = 0.04         # m/s; below this the PWM ramps linearly through zero

# ---------------- pendulum ----------------
G = 9.81
L = 0.147                   # half-length, consistent with J = 4/3 m L^2
W2 = 3 * G / (4 * L)        # ~50 (rad/s)^2 ; your free swing measures ~53
C = 3 / (4 * L)             # coupling from cart accel to pendulum accel
BJ = 0.37                   # pivot damping b/J (1/s)

# ---------------- rail ----------------
X_LIMIT = 0.40              # m, start pushing back beyond this

# ---------------- mode switching (with hysteresis) ----------------
ENTER_BAL = 0.35            # rad from upright to catch
ENTER_RATE = 5.0            # rad/s max pendulum rate to catch
EXIT_BAL = 0.70             # rad from upright to give up

# ---------------- swing-up ----------------
K_E = 12.0                  # energy pumping gain (m/s^2 per unit energy per rad/s)
A_MAX = 8.0                 # m/s^2 accel limit during swing-up
K_XC, K_VC = 6.0, 3.0       # cart centering during swing-up
E_TARGET = 1.2              # normalized energy, upright = 1.0 (overshoot a bit; model is approximate)


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def linear_model():
    """State s = [x, xd, phi, phid], phi = th - pi. Input = v_cmd (m/s)."""
    A = np.array([
        [0, 1, 0, 0],
        [0, -1 / TAU, 0, 0],
        [0, 0, 0, 1],
        [0, -C / TAU, W2, -BJ],
    ])
    B = np.array([[0], [1 / TAU], [0], [C / TAU]])
    return A, B


def design_lqr(Q=np.diag([30.0, 1.0, 60.0, 1.0]), R=np.array([[1.0]])):
    A, B = linear_model()
    P = solve_continuous_are(A, B, Q, R)
    K = np.linalg.solve(R, B.T @ P).flatten()
    return K


def speed_to_pwm(v):
    top = DEADBAND + V_EPS / K_V
    if abs(v) < V_EPS:
        mag = abs(v) / V_EPS * top
    else:
        mag = DEADBAND + abs(v) / K_V
    mag = min(mag, MOTOR_MAX)
    return int(round(MOTOR_SIGN * math.copysign(mag, v)))


class StateFilter:
    def __init__(self, cutoff_hz=25.0):
        self.tc = 1.0 / (2 * math.pi * cutoff_hz)
        self.prev = None
        self.xd = 0.0
        self.thd = 0.0

    def reset(self):
        self.prev = None
        self.xd = self.thd = 0.0

    def update(self, x, th, dt):
        if self.prev is None or dt <= 0:
            self.prev = (x, th)
            return 0.0, 0.0
        px, pth = self.prev
        raw_xd = (x - px) / dt
        raw_thd = wrap(th - pth) / dt
        a = dt / (self.tc + dt)
        self.xd += a * (raw_xd - self.xd)
        self.thd += a * (raw_thd - self.thd)
        self.prev = (x, th)
        return self.xd, self.thd


class Controller:
    def __init__(self, K=None):
        self.K = design_lqr() if K is None else K
        self.mode = "swing"

    def reset(self):
        self.mode = "swing"

    def swing_accel(self, x, xd, th, thd):
        E = thd * thd / (2 * W2) - math.cos(th)
        if abs(wrap(th)) < 0.05 and abs(thd) < 0.2:
            a = A_MAX                              # kick from rest
        else:
            a = K_E * (E - E_TARGET) * thd * math.cos(th)
        a -= K_XC * x + K_VC * xd                  # keep it on the rail
        return max(-A_MAX, min(A_MAX, a))

    def step(self, x, xd, th, thd):
        phi = wrap(th - math.pi)

        if self.mode == "balance" and abs(phi) > EXIT_BAL:
            self.mode = "swing"
        elif self.mode == "swing" and abs(phi) < ENTER_BAL and abs(thd) < ENTER_RATE:
            self.mode = "balance"

        if self.mode == "balance":
            v = -float(self.K @ np.array([x, xd, phi, thd]))
        else:
            v = xd + TAU * self.swing_accel(x, xd, th, thd)

        # track soft limits
        if x > X_LIMIT and v > 0:
            v = min(v, -2.0 * (x - X_LIMIT))
        elif x < -X_LIMIT and v < 0:
            v = max(v, -2.0 * (x + X_LIMIT))

        v = max(-V_MAX, min(V_MAX, v))
        return v, speed_to_pwm(v)