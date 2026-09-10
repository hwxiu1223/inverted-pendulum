import os
import sys
import time

import control
import keyboard
import numpy as np
import serial

from config import COM_PORT
from cart_control import CartControl
from simulation import constants
from simulation.lqg_controller import hybrid_controller_lqg

enc_count_file = os.path.join(os.path.dirname(__file__), "enc_counts.txt")
log_file = os.path.join(os.path.dirname(__file__), "log.txt")

ser = serial.Serial(COM_PORT, 115200, timeout=1)
cart = CartControl(ser, enc_count_file)

# Line-buffered so entries hit disk immediately rather than sitting in a
# buffer if the process exits abnormally (e.g. Ctrl+C instead of 'space').
log = open(log_file, "a", buffering=1)

enc_count = cart.handshake()
print("Loaded saved encoder count:", enc_count)

# ---------------------------------------------------------------------------
# Build the LQG model + gains once at startup (same model as lqg_sim.py).
# ---------------------------------------------------------------------------
g, L, m, M, d, b = constants.g, constants.L, constants.m, constants.M, constants.d, constants.b
J = (4 / 3) * m * L**2
Delta = (M + m) * J - m**2 * L**2

A = np.array([
    [0, 1, 0, 0],
    [0, -J * d / Delta,        -m**2 * g * L**2 / Delta,     -m * L * b / Delta],
    [0, 0, 0, 1],
    [0, -m * L * d / Delta,    -(M + m) * m * g * L / Delta, -(M + m) * b / Delta],
])
B = np.array([[0], [J / Delta], [0], [m * L / Delta]])

# sensors: cart position + pendulum angle (same two encoders read below)
C = np.array([
    [1, 0, 0, 0],
    [0, 0, 1, 0],
])

Q = np.array([
    [200, 0, 0, 0],
    [0, 0, 0, 0],
    [0, 0, 1000, 0],
    [0, 0, 0, 0],
])
R = np.array([[0.01]])
K, _, _ = control.lqr(A, B, Q, R)
K = np.asarray(K).flatten()

Qn = np.diag([0, constants.q_dx, 0, constants.q_dtheta])
Rn = np.diag([constants.sigma_x**2, constants.sigma_theta**2])
G = np.eye(4)
L_gain, _, _ = control.lqe(A, G, C, Qn, Rn)

# ---------------------------------------------------------------------------
# Runtime state for the control loop. xhat is the Kalman filter's running
# estimate of [x, dx, theta, dtheta]; it's only meaningful while control is
# active, so it gets reset every time control is (re)started.
# ---------------------------------------------------------------------------
xhat = np.zeros(4)
t_sim = 0.0
last_step_time = None

# Upper bound on dt fed into the forward-Euler estimator update. The offline
# sim (and therefore the LQE gain L_gain) is tuned around a fixed dt of
# ~0.002-0.02s; a stall (dropped serial frame, OS scheduling hiccup, etc.)
# can otherwise produce a dt 100-1000x larger, which blows up xhat in one
# step and shows up as a jitter/kick in the motor command.
MAX_DT = 0.05

key_state = {}
control_active = False


def key_just_pressed(key):
    is_down = keyboard.is_pressed(key)
    was_down = key_state.get(key, False)
    key_state[key] = is_down
    return is_down and not was_down


def start_control():
    """Reset the estimator and clock so a stale xhat/timestamp from a
    previous run never leaks into a fresh start."""
    global xhat, t_sim, last_step_time
    xhat = np.zeros(4)
    t_sim = 0.0
    last_step_time = time.time()
    print("Control started")


def stop_control():
    cart.set_motor(0)
    print("Control stopped")


def control_step():
    """One iteration of the real-time LQG loop: read the encoders, run the
    estimator + controller, and command the motor."""
    global xhat, t_sim, last_step_time

    x, theta, mag_val, rotary_val = cart.read_state()
    now = time.time()

    if x is None:
        msg = "!! Unable to get encoder data, stopping motor !!"
        print(msg)
        log.write(f"{now:.3f} {msg}\n")
        cart.set_motor(0)
        # Refresh the timestamp even on a failed read so a stale
        # last_step_time doesn't compound into a huge dt on the next
        # successful step.
        last_step_time = now
        return

    dt = (now - last_step_time) if last_step_time else 0.0
    last_step_time = now

    if dt <= 0:
        return

    # Guard against stalls (serial hiccup, OS scheduling delay, etc.)
    # producing a dt far larger than the estimator was tuned for, which
    # would otherwise blow up xhat in a single forward-Euler step.
    if dt > MAX_DT:
        msg = f"!! dt={dt:.3f}s exceeds MAX_DT={MAX_DT}s, clamping !!"
        print(msg)
        log.write(f"{now:.3f} {msg}\n")
        dt = MAX_DT

    y_meas = np.array([x, theta])
    u, xhat = hybrid_controller_lqg(t_sim, xhat, y_meas, K, L_gain, A, B, C, dt)
    t_sim += dt

    motor_val = cart.force_to_motor(u)
    motor_val = cart.enforce_limits(motor_val, rotary_val)
    cart.set_motor(motor_val)
    print("u val", u, ", Motor: ", motor_val, ", PEND: ", mag_val, ", CART: ", rotary_val)
    log.write(
        f"{now:.3f} u val {u}, Motor: {motor_val}, PEND: {mag_val}, CART: {rotary_val}\n"
    )


while True:
    if key_just_pressed('p'):
        control_active = not control_active
        if control_active:
            start_control()
        else:
            stop_control()

    if control_active:
        control_step()
    else:
        cart.set_motor(0)

    if key_just_pressed('s'):
            cart.save_enc_val()
    
    if key_just_pressed('m'):
        cart.center_cart()
    
    if key_just_pressed('r'):
        cart.set_motor(0)
        cart.reset_enc_val()
    
    if key_just_pressed('space'):
        cart.set_motor(0)
        print("Cart stopped, saving encoder count...")
        time.sleep(1)
        saved = cart.save_enc_val()
        print("Stopped and saved encoder count:", saved)

        cart.reset_arduino()
        time.sleep(1)

        log.close()
        sys.exit()
        break