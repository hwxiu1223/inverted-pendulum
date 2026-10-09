"""
motor_id.py -- measure DEADBAND, K_V and TAU for controller_v2.py.
Pendulum can be removed or left hanging; it barely matters.
Run from the same folder as main.py.
"""
import os
import time

import numpy as np
import serial

from config import COM_PORT
from cart_control import CartControl
from simulation import constants

M_PER_COUNT = constants.METERS_PER_COUNT

ser = serial.Serial(COM_PORT, 115200, timeout=1)
cart = CartControl(ser, os.path.join(os.path.dirname(__file__), "enc_counts.txt"))
cart.handshake()


def read_counts():
    _, r = cart.read_encoders()
    return r


def find_deadband(direction):
    """Slowly ramp PWM until the cart actually starts moving."""
    cart.center_cart()
    time.sleep(0.5)
    start = read_counts()
    for pwm in range(20, 140, 2):
        cart.set_motor(direction * pwm)
        time.sleep(0.08)
        r = read_counts()
        if r is not None and abs(r - start) > 30:
            cart.set_motor(0)
            return pwm
    cart.set_motor(0)
    return None


def step_response(pwm, dur=0.35):
    cart.center_cart()
    time.sleep(0.5)
    data = []
    t0 = time.time()
    cart.set_motor(pwm)
    while time.time() - t0 < dur:
        r = read_counts()
        if r is None:
            continue
        data.append((time.time() - t0, r * M_PER_COUNT))
        if r < 6000 or r > 54000:
            break
    cart.set_motor(0)
    d = np.array(data)
    t, x = d[:, 0], d[:, 1]
    v = np.gradient(x, t)
    v = np.convolve(v, np.ones(5) / 5, mode="same")        # light smoothing
    late = t > 0.6 * t[-1]
    v_ss = float(np.median(v[late]))
    reach = np.nonzero(np.abs(v) >= 0.632 * abs(v_ss))[0]
    tau = float(t[reach[0]]) if len(reach) else float("nan")
    return v_ss, tau


if __name__ == "__main__":
    try:
        db_pos = find_deadband(+1)
        db_neg = find_deadband(-1)
        print(f"static deadband: +PWM {db_pos}, -PWM {db_neg}")

        rows = []
        for mag in (90, 120, 150, 180, 210, 240, 255):
            for sgn in (-1, +1):
                v_ss, tau = step_response(sgn * mag)
                rows.append((sgn * mag, v_ss, tau))
                print(f"PWM {sgn*mag:+4d}: v_ss {v_ss:+.3f} m/s   tau {tau*1000:.0f} ms")

        pwm = np.array([abs(r[0]) for r in rows], float)
        vss = np.array([abs(r[1]) for r in rows], float)
        k_v, b = np.polyfit(pwm, vss, 1)          # |v| = k_v*|pwm| + b
        print("\n--- put these in controller_v2.py ---")
        print(f"K_V      = {k_v:.5f}   # m/s per PWM")
        print(f"DEADBAND = {-b / k_v:.0f}      # kinetic (fit intercept); "
              f"static was {db_pos}/{db_neg}")
        print(f"TAU      = {np.nanmedian([r[2] for r in rows]):.3f}")
        sign_ok = all((r[0] < 0) == (r[1] > 0) for r in rows)
        print(f"MOTOR_SIGN = -1 is {'correct' if sign_ok else 'WRONG, use +1'}")
    finally:
        cart.set_motor(0)