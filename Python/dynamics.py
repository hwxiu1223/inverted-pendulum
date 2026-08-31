import constants
import numpy as np

g, L, m, M, W, H, wr, mr, d, b, cd_cart, cd_pend = (
    constants.g, constants.L, constants.m, constants.M, constants.W, constants.H,
    constants.wr, constants.mr, constants.d, constants.b, constants.cd_cart, constants.cd_pend
)

def cartpend(y, u):
    x, dx, theta, dtheta = y

    Sy = np.sin(theta)
    Cy = np.cos(theta)
    D = m * L * L * (M + m * (1 - Cy**2))  # damping denominator (N)

    F_cart = m * L * dtheta**2 * Sy - d * dx - cd_cart * dx * abs(dx)

    # Pivot-side damping "torque": linear bearing friction + nonlinear
    # aerodynamic drag on the swinging pendulum (scales as w|w|)
    F_pivot = b * dtheta + cd_pend * dtheta * abs(dtheta)

    dy = np.zeros(4)
    dy[0] = dx
    dy[1] = (1 / D) * (-m**2 * L**2 * g * Cy * Sy + m * L**2 * F_cart + m * L * Cy * F_pivot) + m * L * L * (1 / D) * u
    dy[2] = dtheta
    dy[3] = (1 / D) * ((m + M) * m * g * L * Sy - m * L * Cy * F_cart - (m + M) * F_pivot) - m * L * Cy * (1 / D) * u

    return dy