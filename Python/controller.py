import control
import numpy as np
import constants
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from dynamics import cartpend

g, L, m, M, W, H, wr, mr, d, b, cd_cart, cd_pend = (
    constants.g, constants.L, constants.m, constants.M, constants.W, constants.H,
    constants.wr, constants.mr, constants.d, constants.b, constants.cd_cart, constants.cd_pend
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

def hybrid_controller(t, y, K):
    x, dx, theta, dtheta = y

    theta_wrapped = (theta + np.pi) % (2 * np.pi) - np.pi

    if abs(theta - np.pi) < 0.6:
        target = np.array([x, 0, np.pi, 0])
        u = -K @ (y - target)
    else:
        if abs(theta_wrapped) < 0.05 and abs(dtheta) < 0.05:
            u = 6
        else:
            g_mag = abs(g)
            E = 0.5 * m * L**2 * dtheta**2 - m * g_mag * L * np.cos(theta)
            E_target = m * g_mag * L
            ke = 0.4
            u = ke * (E - E_target) * dtheta * np.cos(theta) - 0.2 * dx

        u = max(min(u, 20), -20)

    dy = cartpend(y, u)
    return dy, u


tspan = np.arange(0, 30.5, 0.5)
y0 = [0, 0, 0, 0]

sol = solve_ivp(
    lambda t, y: hybrid_controller(t, y, K)[0],  # only dy goes to the solver
    [tspan[0], tspan[-1]],
    y0,
    t_eval=tspan
)

t = sol.t
y = sol.y.T  # transpose so each row is a timestep, matching MATLAB's y(k,:) shape  