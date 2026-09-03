import os
import time
import keyboard
import serial
import sys
from config import COM_PORT
from cart_control import CartControl
from matlab_bridge import MatlabLQG

MATLAB_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "MATLAB"))
print("Looking for MATLAB files in:", MATLAB_DIR)

enc_count_file = os.path.join(os.path.dirname(__file__), "enc_counts.txt")

ser = serial.Serial(COM_PORT, 115200, timeout=1)
cart = CartControl(ser, enc_count_file)
matlab_lqg = MatlabLQG(MATLAB_DIR)

enc_count = cart.handshake()
print("Loaded saved encoder count:", enc_count)

key_state = {}
control_active = False

# Kalman filter state carried between control-loop ticks
xhat = [0.0, 0.0, 0.0, 0.0]
t_sim = 0.0
last_step_time = None


def key_just_pressed(key):
    is_down = keyboard.is_pressed(key)
    was_down = key_state.get(key, False)
    key_state[key] = is_down
    return is_down and not was_down


try:
    while True:
        if key_just_pressed('p'):
            control_active = not control_active
            if control_active:
                matlab_lqg.connect()  # no-op after the first call
                xhat = [0.0, 0.0, 0.0, 0.0]
                t_sim = 0.0
                last_step_time = time.perf_counter()
                print("starting control")
            else:
                cart.set_motor(0)
                print("stopping control")

        if control_active:
            x_meas, theta_meas = cart.read_state()
            if x_meas is not None:
                now = time.perf_counter()
                dt = (now - last_step_time) if last_step_time else 0.002
                last_step_time = now
                t_sim += dt

                u, xhat = matlab_lqg.step(t_sim, xhat, x_meas, theta_meas, dt)
                cart.send_force(u, matlab_lqg.params['u_max'])

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

            matlab_lqg.disconnect()
            sys.exit()
            break
finally:
    matlab_lqg.disconnect()