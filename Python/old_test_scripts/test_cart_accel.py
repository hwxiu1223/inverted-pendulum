"""
test_cart_acceleration.py

Standalone hardware calibration script. Commands a fixed motor PWM for a
short pulse, records cart position vs. time from the rotary encoder, and
fits the initial position trace to estimate real acceleration -> real force.

This exists to answer one question: does the real rig actually deliver the
force the simulation assumes (u_max = 2.55 N at PWM = 255, motion starting
around PWM = DEADBAND = 50), or is the real motor/belt/friction system
delivering meaningfully less? That mismatch is the leading suspect for why
the swing-up controller stalls out at a small bounded oscillation on
hardware while working fine in simulation.

Usage:
    python test_cart_acceleration.py                       # default sweep
    python test_cart_acceleration.py --pwms 60 100 160 255  # specific PWMs
    python test_cart_acceleration.py --find-deadband        # just find the
                                                              real deadband
    python test_cart_acceleration.py --plot                 # also plot

Safety:
    - Cart is re-centered before every trial.
    - Every trial is aborted immediately if the cart nears the rail ends.
    - Motor is always set to 0 on exit, including on Ctrl+C or an error.
"""

import argparse
import csv
import os
import sys
import time

import numpy as np
import serial

from config import COM_PORT
from cart_control import CartControl
from simulation import constants

# Encoder counts from ROTARY_CENTER at which we bail out of a trial early,
# regardless of requested duration. Keeps trials well inside the rail even
# for pulses that turn out to accelerate harder than expected.
SAFETY_HALF_RANGE_COUNTS = 15000  # ~0.255 m each side of center


def collect_pulse(cart, pwm, duration_s):
    """Command a fixed PWM for duration_s while sampling the rotary encoder
    as fast as the serial round-trip allows. Returns (t, x) arrays, t in
    seconds from pulse start, x in meters relative to the position at
    pulse start. Stops the motor and returns early if the cart nears the
    rail limits.
    """
    _, rot0 = cart.read_encoders()
    if rot0 is None:
        raise RuntimeError("Could not read encoders before starting pulse")

    t_hist = []
    x_hist = []

    t0 = time.time()
    cart.set_motor(pwm)
    try:
        while True:
            now = time.time()
            elapsed = now - t0
            if elapsed > duration_s:
                break

            _, rot = cart.read_encoders()
            if rot is None:
                continue

            if abs(rot - constants.ROTARY_CENTER) > SAFETY_HALF_RANGE_COUNTS:
                print(f"  !! Safety limit hit at rotary={rot}, stopping pulse early")
                break

            t_hist.append(elapsed)
            x_hist.append((rot - rot0) * constants.METERS_PER_COUNT)
    finally:
        cart.set_motor(0)

    return np.array(t_hist), np.array(x_hist)


def fit_acceleration(t, x, frac=0.6):
    """Fit x(t) = x0 + v0*t + 0.5*a*t^2 by least squares over the first
    `frac` of the collected window (the constant-force / pre-deadband-of-
    velocity-limit phase), and return (x0, v0, a). Using a quadratic fit
    over a window is much less noise-sensitive than double-differencing
    the raw encoder trace.
    """
    if len(t) < 5:
        return None

    n = max(5, int(len(t) * frac))
    tt = t[:n]
    xx = x[:n]

    # Design matrix for [x0, v0, 0.5*a]
    A = np.stack([np.ones_like(tt), tt, tt**2], axis=1)
    coeffs, *_ = np.linalg.lstsq(A, xx, rcond=None)
    x0, v0, half_a = coeffs
    return x0, v0, 2 * half_a


def motor_val_to_force(motor_val, deadband=None, u_max=None):
    """Inverse of CartControl.force_to_motor's *assumed* mapping, for
    comparison against what we actually measure."""
    deadband = constants.DEADBAND if deadband is None else deadband
    u_max = constants.u_max if u_max is None else u_max
    mag = abs(motor_val)
    if mag < deadband:
        return 0.0
    norm = (mag - deadband) / (constants.MOTOR_MAX - deadband)
    return norm * u_max


def run_sweep(cart, pwms, duration_s, mass_total, out_csv):
    results = []
    for i, pwm in enumerate(pwms):
        sign = 1 if pwm >= 0 else -1
        print(f"[{i+1}/{len(pwms)}] Centering before trial (pwm={pwm})...")
        cart.center_cart()
        time.sleep(0.3)

        print(f"  Commanding motor={pwm} for {duration_s:.2f}s...")
        t, x = collect_pulse(cart, pwm, duration_s)

        if len(t) < 5:
            print("  !! Not enough samples collected, skipping")
            results.append(dict(pwm=pwm, n_samples=len(t), a_measured=None,
                                 f_measured=None, f_assumed=None))
            continue

        fit = fit_acceleration(t, x)
        if fit is None:
            print("  !! Fit failed, skipping")
            continue

        x0, v0, a = fit
        f_measured = mass_total * a
        f_assumed = motor_val_to_force(pwm)
        # sign convention: MOTOR_SIGN flips the commanded polarity vs u;
        # report magnitudes since we only care about authority here.
        print(f"  samples={len(t):4d}  a={a:+.4f} m/s^2  "
              f"F_measured={f_measured:+.3f} N  F_assumed={f_assumed:+.3f} N")

        results.append(dict(
            pwm=pwm, n_samples=len(t), duration_s=duration_s,
            a_measured=a, f_measured=f_measured, f_assumed=f_assumed,
        ))

    with open(out_csv, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=[
            "pwm", "n_samples", "duration_s", "a_measured", "f_measured", "f_assumed"])
        writer.writeheader()
        for row in results:
            writer.writerow(row)
    print(f"\nSaved results to {out_csv}")

    return results


def find_deadband(cart, pwm_lo=10, pwm_hi=120, step=5, probe_s=0.3,
                   move_threshold_m=0.001):
    """Sweep PWM upward from pwm_lo and report the first value at which the
    cart actually moves more than move_threshold_m within probe_s. Compares
    against the assumed constants.DEADBAND."""
    print(f"Searching for real deadband between {pwm_lo} and {pwm_hi} "
          f"(assumed DEADBAND={constants.DEADBAND})...")
    found = None
    for pwm in range(pwm_lo, pwm_hi + 1, step):
        cart.center_cart()
        time.sleep(0.2)
        t, x = collect_pulse(cart, pwm, probe_s)
        cart.set_motor(0)
        moved = len(x) > 0 and abs(x[-1]) > move_threshold_m
        print(f"  pwm={pwm:4d}  moved={moved}  "
              f"net_x={(x[-1] if len(x) else 0)*1000:+.2f} mm")
        if moved and found is None:
            found = pwm
            break

    if found is None:
        print(f"No motion detected up to pwm={pwm_hi}. Either the real "
              f"deadband is higher than tested, or something else is "
              f"wrong (wiring, driver enable, etc).")
    else:
        print(f"\nReal deadband appears to be ~{found} "
              f"(assumed constants.DEADBAND = {constants.DEADBAND})")
    return found


def maybe_plot(results):
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not available, skipping plot")
        return

    pwms = [r["pwm"] for r in results if r.get("f_measured") is not None]
    f_meas = [r["f_measured"] for r in results if r.get("f_measured") is not None]
    f_assum = [r["f_assumed"] for r in results if r.get("f_measured") is not None]

    if not pwms:
        return

    plt.figure(figsize=(7, 5))
    plt.plot(pwms, f_assum, "o--", label="Force assumed by force_to_motor()")
    plt.plot(pwms, f_meas, "o-", label="Force measured on hardware")
    plt.xlabel("Motor PWM command")
    plt.ylabel("Force (N)")
    plt.title("Assumed vs. measured motor force")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pwms", type=int, nargs="+", default=None,
                         help="PWM values to test (e.g. --pwms 60 100 160 255). "
                              "Default: a sweep from just above DEADBAND to MOTOR_MAX.")
    parser.add_argument("--duration", type=float, default=0.25,
                         help="Pulse duration per trial, seconds (default 0.25)")
    parser.add_argument("--find-deadband", action="store_true",
                         help="Instead of the force sweep, search for the real "
                              "minimum PWM that produces motion")
    parser.add_argument("--plot", action="store_true",
                         help="Show a matplotlib plot of assumed vs measured force")
    parser.add_argument("--out", type=str, default="cart_accel_results.csv",
                         help="CSV output path for the force sweep")
    args = parser.parse_args()

    enc_count_file = os.path.join(os.path.dirname(__file__), "enc_counts.txt")

    ser = serial.Serial(COM_PORT, 115200, timeout=1)
    cart = CartControl(ser, enc_count_file)

    try:
        enc_count = cart.handshake()
        print("Loaded saved encoder count:", enc_count)
        print("Centering cart before starting...")
        cart.center_cart()

        if args.find_deadband:
            find_deadband(cart)
        else:
            pwms = args.pwms
            if pwms is None:
                lo = constants.DEADBAND + 10
                hi = constants.MOTOR_MAX
                pwms = list(np.linspace(lo, hi, 6).astype(int))
            print(f"Testing PWM values: {pwms}")

            # Total moving mass as a first-order approximation. The real
            # coupling with the pendulum is more complex, but for a short,
            # sharp pulse starting from rest with the pendulum hanging
            # near-stationary, cart+pendulum mass is a reasonable estimate.
            mass_total = constants.M + constants.m

            results = run_sweep(cart, pwms, args.duration, mass_total, args.out)

            if args.plot:
                maybe_plot(results)

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        cart.set_motor(0)
        cart.save_enc_val()
        cart.reset_arduino()    
        ser.close()
        print("Motor stopped, serial port closed.")


if __name__ == "__main__":
    main()