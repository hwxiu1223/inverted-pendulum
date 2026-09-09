import control
import matplotlib.pyplot as plt
import numpy as np

from simulation import constants
from simulation.dynamics import cartpend
from simulation.lqg_controller import hybrid_controller_lqg

g = constants.g
L = constants.L
m = constants.m
M = constants.M
d = constants.d
b = constants.b
J = (4 / 3) * m * L**2
Delta = (M + m) * J - m**2 * L**2

# matrices created from equations of motion
# A acts as matrix for the system dynamics (state 'y')
# B acts as matrix mapping the input to the state ('u')
A = np.array([
    [0, 1, 0, 0],
    [0, -J * d / Delta,        -m**2 * g * L**2 / Delta,   -m * L * b / Delta],
    [0, 0, 0, 1],
    [0, -m * L * d / Delta,    -(M + m) * m * g * L / Delta, -(M + m) * b / Delta],
])
B = np.array([[0], [J / Delta], [0], [m * L / Delta]])

# lqg
# sensors on cart position and pendulum angle
C = np.array([
    [1, 0, 0, 0],
    [0, 0, 1, 0],
])

# lqr gain

# Q matrix represents the allowed deviation of a state variable from
# its desired target
# eg. Q[0,0] = 200 means the simulation will prioritize fixing the
# position to its desired value over cart velocity (Q[1,1]) and
# pendulum velocity (Q[3,3]), but less than the angle (Q[2,2])

# R value represents the penalty associated with the usage of the
# controller (lower value -> faster correction; may need tuning)

# K runs everything through the lqr cost function and returns an
# optimized value that will be given to the input for control
Q = np.array([
    [200, 0, 0, 0],
    [0, 0, 0, 0],
    [0, 0, 1000, 0],
    [0, 0, 0, 0],
])
R = np.array([[0.01]])
K, _, _ = control.lqr(A, B, Q, R)
K = np.asarray(K).flatten()

# kalman filter gain
# Qn represents any disturbances or unmodeled dynamics affecting the
# plant states (true physical states)
Qn = np.diag([0, constants.q_dx, 0, constants.q_dtheta])

# Rn represents any noise or inaccuracies affecting the sensors
Rn = np.diag([constants.sigma_x**2, constants.sigma_theta**2])

# process noise enters through matrix G
G = np.eye(4)

# lqe acts as modified lqr running through new Kalman filter
L_gain, _, _ = control.lqe(A, G, C, Qn, Rn)

# checking that filter's error converges
print("Estimator poles (hopefully all negative real part):")
print(np.linalg.eigvals(A - L_gain @ C))

# simulation settings (using fixed step)
dt = 0.002       # estimator update / control period (s)
Tfinal = 30
N = round(Tfinal / dt) + 1     # total number of points for time grid
t_hist = np.arange(N) * dt     # build fixed time grid

y_true = np.zeros((N, 4))      # true plant state
xhat_hist = np.zeros((N, 4))   # Kalman filter estimate
y_true[0, :] = [0, 0, 0, 0]        # initial plant state
xhat_hist[0, :] = [0, 0, 0, 0]      # estimator starts with no information

rng = np.random.default_rng(1)  # reproducible sensor noise draw; fixes the random seed
sensor_sigma = np.array([constants.sigma_x, constants.sigma_theta])


def rk4_step(f, t, y, dt):
    """samples derivative f at four points across the interval and combines
    them in a 1-2-2-1 weighted average"""
    k1 = f(t, y)
    k2 = f(t + dt / 2, y + dt / 2 * k1)
    k3 = f(t + dt / 2, y + dt / 2 * k2)
    k4 = f(t + dt, y + dt * k3)
    return y + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)


# main loop
for k in range(N - 1):
    # pull out this step's true state and estimated state
    x_k = y_true[k, :]
    xhat_k = xhat_hist[k, :]

    # simulate noisy sensor reading
    y_meas = C @ x_k + sensor_sigma * rng.standard_normal(2)

    # controller computing u from estimate and advancing estimator by
    # one step, and returning the force needed
    u, xhat_next = hybrid_controller_lqg(t_hist[k], xhat_k, y_meas, K, L_gain, A, B, C, dt)

    # advances true nonlinear plant state one step
    x_next = rk4_step(lambda tt, yy: cartpend(yy, u), t_hist[k], x_k, dt)

    # log trajectories
    y_true[k + 1, :] = x_next
    xhat_hist[k + 1, :] = xhat_next

# plot results (cart position and pendulum angle, true vs. estimated)
fig, axes = plt.subplots(2, 1, sharex=True, figsize=(8, 6))
axes[0].plot(t_hist, y_true[:, 0], label="true x")
axes[0].plot(t_hist, xhat_hist[:, 0], "--", label="estimated x")
axes[0].set_ylabel("cart position (m)")
axes[0].legend()

axes[1].plot(t_hist, y_true[:, 2], label="true theta")
axes[1].plot(t_hist, xhat_hist[:, 2], "--", label="estimated theta")
axes[1].set_ylabel("pendulum angle (rad)")
axes[1].set_xlabel("time (s)")
axes[1].legend()

fig.suptitle("Inverted Pendulum Simulation (LQG)")
plt.tight_layout()
plt.show()