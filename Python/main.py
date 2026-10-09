import os
import sys
import time

import keyboard
import serial

from config import COM_PORT
from cart_control import CartControl
import Python.controller as cv

enc_count_file = os.path.join(os.path.dirname(__file__), "enc_counts.txt")
log_file = os.path.join(os.path.dirname(__file__), "log_v2.csv")

ser = serial.Serial(COM_PORT, 115200, timeout=1)
cart = CartControl(ser, enc_count_file)
print("Loaded saved encoder count:", cart.handshake())

ctl = cv.Controller()
filt = cv.StateFilter(cutoff_hz=25.0)
print("LQR K =", ctl.K)

log = open(log_file, "a")          # block-buffered: no disk hit every loop
log.write("t,dt,x,xd,theta,thetad,mode,v_cmd,pwm,mag,rot\n")

key_state = {}
control_active = False
last_t = None
last_mode = None


def key_just_pressed(key):
    is_down = keyboard.is_pressed(key)
    was_down = key_state.get(key, False)
    key_state[key] = is_down
    return is_down and not was_down


def control_step():
    global last_t, last_mode
    x, th, mag, rot = cart.read_state()
    now = time.perf_counter()
    if x is None:
        cart.set_motor(0)
        filt.reset()               # don't difference across a dropout
        last_t = None
        return
    dt = (now - last_t) if last_t is not None else 0.0
    last_t = now
    if dt > 0.03:                  # big stall: velocities would be garbage
        filt.reset()
        dt = 0.0

    xd, thd = filt.update(x, th, dt)
    v, pwm = ctl.step(x, xd, th, thd)
    pwm = cart.enforce_limits(pwm, rot)
    cart.set_motor(pwm)

    if ctl.mode != last_mode:
        print("mode ->", ctl.mode)
        last_mode = ctl.mode
    log.write(f"{now:.4f},{dt:.4f},{x:.4f},{xd:.3f},{th:.4f},{thd:.3f},"
              f"{ctl.mode},{v:.3f},{pwm},{mag},{rot}\n")


loop_i = 0
try:
    while True:
        loop_i += 1
        if control_active:
            control_step()
        else:
            time.sleep(0.002)

        if loop_i % 5:             # poll keyboard every 5th loop only
            continue

        if key_just_pressed('p'):
            control_active = not control_active
            cart.set_motor(0)
            ctl.reset()
            filt.reset()
            last_t = None
            print("Control", "started" if control_active else "stopped")
        if not control_active:
            cart.set_motor(0)
        if key_just_pressed('s'):
            cart.save_enc_val()
        if key_just_pressed('m'):
            cart.center_cart()
        if key_just_pressed('r'):
            cart.set_motor(0)
            cart.reset_enc_val()
        if key_just_pressed('space'):
            break
finally:
    cart.set_motor(0)
    time.sleep(0.5)
    print("Saved encoder count:", cart.save_enc_val())
    cart.reset_arduino()
    log.close()
    sys.exit()