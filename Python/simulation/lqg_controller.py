import numpy as np

from simulation import constants
from simulation.dynamics import cartpend


def wrap_to_pi(theta):
    """Equivalent of MATLAB's wrapToPi: wraps angle into [-pi, pi]."""
    return (theta + np.pi) % (2 * np.pi) - np.pi


def track_limit_force(x, dx):
    """Adjusted force as cart enters marginal distance from wall."""
    u_wall = 0.0
    margin = constants.x_margin

    if x > (constants.x_max - margin):
        penetration = x - (constants.x_max - margin)
        u_wall = -constants.k_wall * penetration - constants.c_wall * max(dx, 0)
    elif x < (constants.x_min + margin):
        penetration = x - (constants.x_min + margin)
        u_wall = -constants.k_wall * penetration - constants.c_wall * min(dx, 0)

    return u_wall


def hybrid_controller_lqg(t, xhat, y_meas, K, L_gain, A, B, C, dt):
    """hybrid_controller_lqg: modified swing-up and lqg balancing controller
    driven by Kalman filter estimate.

    xhat    - current state estimate
    y_meas  - sensor measurement
    returns (u, xhat_next) - control force and updated state estimate
    """

    # pull estimated angle and wrap it into [-pi, pi]
    theta_hat = xhat[2]
    theta_wrapped = wrap_to_pi(theta_hat)
    theta_target = np.sign(theta_wrapped) * np.pi
    theta_target_global = theta_hat - theta_wrapped + theta_target

    # lqr controller with new measured values
    if abs(theta_wrapped) > (np.pi - 0.5):
        target = np.array([0.0, 0.0, theta_target_global, 0.0])
        u = float(-K @ (xhat - target))
    else:
        # swing-up controller

        # adjusted inertia term for uniform rod
        J = (4 / 3) * constants.m * constants.L**2

        # initial kick
        if abs(theta_wrapped) < 0.05 and abs(xhat[3]) < 0.1:
            u = constants.u_max
        else:
            # swing-up law
            g_mag = abs(constants.g)
            E = 0.5 * J * xhat[3]**2 - constants.m * g_mag * constants.L * np.cos(theta_hat)
            E_target = constants.m * g_mag * constants.L
            ke = 24.0
            kv = 0.2
            u = ke * (E - E_target) * xhat[3] * np.cos(theta_hat) - kv * xhat[1]

    # motor force ceiling
    if xhat[1] != 0 and np.sign(u) == np.sign(xhat[1]):
        # commanding force that reinforces current motion, applies droop
        u_max_eff = constants.u_max * max(0, 1 - abs(xhat[1]) / constants.v_max)
    else:
        # commanding braking, allows full torque
        u_max_eff = constants.u_max
    u = max(min(u, u_max_eff), -u_max_eff)

    # track limits
    u_wall = track_limit_force(xhat[0], xhat[1])
    u = u + u_wall

    # final hard clamp: never command more than the motor's absolute max
    u = max(min(u, constants.u_max), -constants.u_max)

    # kalman filter update using a simple forward-Euler step of length dt
    dxhat_nonlin = cartpend(xhat, u)
    xhat_next = xhat + dt * dxhat_nonlin + dt * (L_gain @ (y_meas - C @ xhat))

    return u, xhat_next