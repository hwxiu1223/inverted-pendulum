import numpy as np

from simulation import constants

g, L, m, M, W, H, wr, mr, d, b, cd_cart, cd_pend = (
    constants.g, constants.L, constants.m, constants.M, constants.W, constants.H,
    constants.wr, constants.mr, constants.d, constants.b, constants.cd_cart, constants.cd_pend
)


def cartpend(y, u):
    x, dx, theta, dtheta = y

    Sy = np.sin(theta)
    Cy = np.cos(theta)
    J = (4 / 3) * m * L**2            # adjusted rotational inertia term for uniform rod
    D = (M + m) * J - m**2 * L**2 * Cy**2  # damping denominator

    # frictional/drag forces on cart and pivot
    F_cart = m * L * dtheta**2 * Sy - d * dx - cd_cart * dx * abs(dx)
    F_pivot = b * dtheta + cd_pend * dtheta * abs(dtheta)

    dy = np.zeros(4)
    dy[0] = dx
    dy[1] = (1 / D) * (-m**2 * L**2 * g * Cy * Sy + J * F_cart + m * L * Cy * F_pivot) + J * (1 / D) * u
    dy[2] = dtheta
    dy[3] = (1 / D) * ((m + M) * m * g * L * Sy - m * L * Cy * F_cart - (m + M) * F_pivot) - m * L * Cy * (1 / D) * u

    return dy   