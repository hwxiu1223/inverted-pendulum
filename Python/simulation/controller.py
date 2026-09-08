import control
import numpy as np
import Python.simulation.constants as constants
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from Python.simulation.dynamics import cartpend

g, L, m, M, W, H, wr, mr, d, b, cd_cart, cd_pend, u_max, v_max, x_min, x_max, x_margin, k_wall, c_wall = (
    constants.g, constants.L, constants.m, constants.M, constants.W, constants.H,
    constants.wr, constants.mr, constants.d, constants.b, constants.cd_cart, constants.cd_pend,
    constants.u_max, constants.v_max, constants.x_min, constants.x_max, constants.x_margin,
    constants.k_wall, constants.c_wall
)

A_r = np.array([
    [-d / M,             -m * g / M,                   -b / (M * L)                 ],
    [0,                   0,                            1                           ],
    [-d / (M * L),       -(M + m) * g / (M * L),       -(M + m) * b / (m * M * L**2)]
])

B_r = np.array([
    [1 / M      ],
    [0          ],
    [1 / (M * L)]
])

Q_r = np.array([
    [1, 0, 0],
    [0, 10, 0],
    [0, 0, 100]
])

R = np.array([[0.001]])
K3, S, E = control.lqr(A_r, B_r, Q_r, R)
K = np.hstack([[[0]], K3])

def track_limit_force(x, dx):
    """Soft virtual wall: as the cart's body edge nears either end of the
    rail, ramp in a spring-damper restoring force so it decelerates and
    reverses before actually hitting the end of travel."""
    u_wall = 0
    half_W = W / 2

    if (x + half_W) > (x_max - x_margin):
        penetration = (x + half_W) - (x_max - x_margin)
        u_wall = -k_wall * penetration - c_wall * max(dx, 0)
    elif (x - half_W) < (x_min + x_margin):
        penetration = (x - half_W) - (x_min + x_margin)
        u_wall = -k_wall * penetration - c_wall * min(dx, 0)

    return u_wall


def hybrid_controller(t, y, K):
    x, dx, theta, dtheta = y

    theta_wrapped = (theta % (2 * np.pi)) - np.pi

    # --- 1. LQR CATCH & BALANCE ---
    if abs(theta_wrapped) < 0.6:
        theta_target = theta - theta_wrapped
        target = np.array([x, 0, theta_target, 0])
        u = -K @ (y - target)

    # --- 2. ENERGY SWING-UP ---
    else:
        if t < 0.1:
            u = u_max
        else:
            g_mag = abs(g)
            E = 0.5 * m * L**2 * dtheta**2 - m * g_mag * L * np.cos(theta)
            E_target = m * g_mag * L
            ke = 5.0
            u = ke * (E - E_target) * dtheta * np.cos(theta)

    # --- Motor force ceiling (droop when reinforcing current motion) ---
    if dx != 0 and np.sign(u) == np.sign(dx):
        u_max_eff = u_max * max(0, 1 - abs(dx) / v_max)
    else:
        u_max_eff = u_max
    u = max(min(u, u_max_eff), -u_max_eff)

    # --- Soft track-limit repulsion ---
    u_wall = track_limit_force(x, dx)
    u = u + u_wall

    # Final hard clamp: never exceed the motor's absolute max
    u = max(min(u, u_max), -u_max)

    dy = cartpend(y, u)
    return dy, u


tspan = np.arange(0, 30.1, 0.1)
y0 = [0, 0, 0, 0]

sol = solve_ivp(
    lambda t, y: hybrid_controller(t, y, K)[0],  # only dy goes to the solver
    [tspan[0], tspan[-1]],
    y0,
    t_eval=tspan
)

t = sol.t
y = sol.y.T  # transpose so each row is a timestep, matching MATLAB's y(k,:) shape  