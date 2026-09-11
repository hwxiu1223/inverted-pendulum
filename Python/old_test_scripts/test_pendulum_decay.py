"""
test_pendulum_decay.py

Standalone hardware calibration script. With the cart motor held at 0
(free, undriven), displace the pendulum by hand and let it swing freely.
This script records the raw pendulum encoder trace, finds the peak-to-peak
amplitude envelope, and reports how long it takes the oscillation to decay
to half its starting amplitude -- the real damping the swing-up controller
is fighting, as opposed to the idealized d/b/cd_pend values in
simulation/constants.py.

Usage:
    python test_pendulum_decay.py
    python test_pendulum_decay.py --duration 25 --plot
    python test_pendulum_decay.py --out decay_trial1.csv

Procedure:
    1. Run the script. It centers the cart, then waits for you to press
       Enter once.
    2. After you press Enter, it watches the pendulum encoder live. Lift
       the pendulum by hand to a moderate angle (small enough to stay well
       clear of +-180 deg / the wrap boundary -- 20-60 deg is plenty).
       You can hold it there as long as you like.
    3. The instant you let go and it starts swinging back down, the
       script auto-detects the release (peak-then-reversal in the live
       trace) and starts recording from that exact moment -- no second
       Enter press needed. Recording runs for --duration seconds from the
       detected release point while the motor stays at 0.
    4. The script finds each swing's peak angle, prints the amplitude
       envelope, and reports the time to decay to half the starting
       amplitude (both by direct interpolation between the two straddling
       peaks, and by an exponential fit across all peaks as a cross-check).

Safety:
    - The motor is commanded to 0 for the entire recording and never set
      to anything else -- this script never drives the cart.
    - Ctrl+C stops cleanly and still saves/reports whatever was collected.
"""

import argparse
import csv
import os
import time

import numpy as np
import serial

from config import COM_PORT
from cart_control import CartControl
from simulation import constants
from simulation.lqg_controller import wrap_to_pi


def read_theta(cart):
    """Single blocking read of the pendulum angle, wrapped to [-pi, pi].
    Returns None if the encoder read came back empty (caller should
    retry)."""
    mag_val, _ = cart.read_encoders()
    if mag_val is None:
        return None
    return wrap_to_pi((mag_val - constants.PENDULUM_ZERO_RAW) * constants.PENDULUM_RAW_TO_RADIANS)


def record_free_swing(cart, duration_s, t_offset=0.0):
    """Sample the pendulum encoder as fast as the serial round-trip
    allows until elapsed time (measured from t_offset) reaches
    duration_s. Motor is never commanded (left at 0 the whole time).
    t_offset lets a caller pick up the clock where a previous recording
    phase (e.g. wait_for_release) left off, so the two segments can be
    concatenated into one continuous trace. Returns (t, theta) arrays,
    t in seconds from the *original* t=0 (t_offset included), theta in
    radians (wrapped to [-pi, pi])."""
    t_hist = []
    theta_hist = []

    t0 = time.time()
    while True:
        now = time.time()
        elapsed = t_offset + (now - t0)
        if elapsed > duration_s:
            break

        theta = read_theta(cart)
        if theta is None:
            continue

        t_hist.append(elapsed)
        theta_hist.append(theta)

    return np.array(t_hist), np.array(theta_hist)


def wait_for_release(cart, threshold_rad, drop_rad, max_wait_s=60.0):
    """Watch the live pendulum trace while the user lifts it by hand and
    holds it, and auto-detect the moment it's released and starts
    swinging back toward center.

    Logic: keep sampling theta and tracking the running peak |theta|
    seen so far. Once that peak has exceeded threshold_rad (a real,
    deliberate displacement -- not encoder noise near zero) and the
    current |theta| has since fallen drop_rad below that peak (the
    pendulum is unambiguously moving back toward center, i.e. it's been
    let go), the peak sample is taken as the release point.

    Returns (t, theta, release_idx): the full buffered trace collected
    during the wait, with t re-zeroed so t[release_idx] == 0, i.e.
    t[release_idx] is the release instant. Raises TimeoutError if
    max_wait_s elapses with no qualifying release detected (e.g. the
    pendulum was never displaced past threshold_rad).
    """
    t_hist = []
    theta_hist = []

    t0 = time.time()
    peak_amp = 0.0
    peak_idx = None

    while True:
        now = time.time()
        elapsed = now - t0
        if elapsed > max_wait_s:
            raise TimeoutError(
                f"No release detected within {max_wait_s:.0f}s -- did you displace the "
                f"pendulum past {np.degrees(threshold_rad):.1f} deg and let go?"
            )

        theta = read_theta(cart)
        if theta is None:
            continue

        t_hist.append(elapsed)
        theta_hist.append(theta)

        amp = abs(theta)
        if amp >= peak_amp:
            peak_amp = amp
            peak_idx = len(theta_hist) - 1
        elif peak_amp >= threshold_rad and (peak_amp - amp) >= drop_rad:
            t_arr = np.array(t_hist)
            theta_arr = np.array(theta_hist)
            t_release = t_arr[peak_idx]
            return t_arr - t_release, theta_arr, peak_idx


def find_peaks(t, theta):
    """Locate local extrema (both maxima and minima) of theta(t) by sign
    changes in the discrete derivative. Returns arrays of (peak_times,
    peak_abs_amplitudes) -- one entry per half-swing.

    This is a simple, dependency-free stand-in for scipy.signal.find_peaks
    that works fine on a fairly clean encoder trace like this one; if your
    trace is noisy enough to produce spurious extrema, smooth theta with a
    small moving-average window before calling this.
    """
    if len(theta) < 3:
        return np.array([]), np.array([])

    dtheta = np.diff(theta)
    sign = np.sign(dtheta)
    # index i is a peak if the derivative changes sign between i-1 and i
    peak_idx = []
    for i in range(1, len(sign)):
        if sign[i] != 0 and sign[i - 1] != 0 and sign[i] != sign[i - 1]:
            peak_idx.append(i)

    peak_idx = np.array(peak_idx, dtype=int)
    if len(peak_idx) == 0:
        return np.array([]), np.array([])

    return t[peak_idx], np.abs(theta[peak_idx])


def half_life_by_interpolation(peak_t, peak_amp):
    """Direct answer to 'time to decay to half the starting amplitude':
    find where the peak-amplitude envelope first crosses A0/2, linearly
    interpolating between the two straddling peaks. Returns
    (t_half, half_life_s) or (None, None) if it never gets there."""
    if len(peak_amp) < 2:
        return None, None

    A0 = peak_amp[0]
    target = A0 / 2

    for i in range(1, len(peak_amp)):
        if peak_amp[i] <= target:
            t_prev, a_prev = peak_t[i - 1], peak_amp[i - 1]
            t_curr, a_curr = peak_t[i], peak_amp[i]
            if a_prev == a_curr:
                t_half = t_curr
            else:
                frac = (a_prev - target) / (a_prev - a_curr)
                t_half = t_prev + frac * (t_curr - t_prev)
            return t_half, t_half - peak_t[0]

    return None, None


def half_life_by_exponential_fit(peak_t, peak_amp):
    """Cross-check: fit peak_amp(t) = A0 * exp(-(t - t0) / tau) via a
    linear least-squares fit of ln(amplitude) vs time across all peaks,
    then convert tau to a half-life. More robust to noise on any single
    peak than the direct-interpolation method, but assumes the decay is
    close to a single exponential envelope."""
    if len(peak_amp) < 3 or np.any(peak_amp <= 0):
        return None, None

    t0 = peak_t[0]
    x = peak_t - t0
    y = np.log(peak_amp)

    A = np.stack([np.ones_like(x), x], axis=1)
    coeffs, *_ = np.linalg.lstsq(A, y, rcond=None)
    ln_A0, slope = coeffs  # slope = -1/tau

    if slope >= 0:
        return None, None  # growing or flat, not decaying

    tau = -1.0 / slope
    half_life = tau * np.log(2)
    return tau, half_life


def maybe_plot(t, theta, peak_t, peak_amp, t_half_interp):
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not available, skipping plot")
        return

    plt.figure(figsize=(9, 5))
    plt.plot(t, np.degrees(theta), label="pendulum angle", linewidth=1)
    plt.plot(peak_t, np.degrees(peak_amp), "o", label="peak envelope (+)")
    plt.plot(peak_t, -np.degrees(peak_amp), "o", color="tab:orange", label="peak envelope (-)")
    if t_half_interp is not None:
        plt.axvline(t_half_interp, color="k", linestyle="--", label="half-amplitude time")
    plt.xlabel("time (s)")
    plt.ylabel("theta (deg)")
    plt.title("Free pendulum decay")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=20.0,
                         help="Recording length in seconds after you press Enter (default 20)")
    parser.add_argument("--out", type=str, default="pendulum_decay.csv",
                         help="CSV output path for the raw trace")
    parser.add_argument("--plot", action="store_true",
                         help="Show a matplotlib plot of the decay and peak envelope")
    parser.add_argument("--release-threshold-deg", type=float, default=8.0,
                         help="Minimum hand-displacement angle (deg) before a release can "
                              "be detected -- filters out encoder noise near zero (default 8)")
    parser.add_argument("--release-drop-deg", type=float, default=3.0,
                         help="How far (deg) the amplitude must fall back from its peak to "
                              "confirm the pendulum has been let go and is swinging back "
                              "(default 3)")
    parser.add_argument("--max-wait", type=float, default=60.0,
                         help="Max seconds to wait for a release before giving up (default 60)")
    args = parser.parse_args()

    enc_count_file = os.path.join(os.path.dirname(__file__), "enc_counts.txt")

    ser = serial.Serial(COM_PORT, 115200, timeout=1)
    cart = CartControl(ser, enc_count_file)

    try:
        enc_count = cart.handshake()
        print("Loaded saved encoder count:", enc_count)
        cart.set_motor(0)
        print("Centering cart (motor will be held at 0 during the actual test)...")
        cart.center_cart()
        cart.set_motor(0)

        input("\nPress Enter, then displace the pendulum by hand whenever you're ready...")
        print("Watching the pendulum -- lift it and let go; recording will start "
              "automatically the instant you release it...")

        threshold_rad = np.radians(args.release_threshold_deg)
        drop_rad = np.radians(args.release_drop_deg)
        try:
            t_pre, theta_pre, release_idx = wait_for_release(
                cart, threshold_rad, drop_rad, max_wait_s=args.max_wait
            )
        except TimeoutError as e:
            print(f"\n{e}")
            return

        release_amp_deg = np.degrees(abs(theta_pre[release_idx]))
        print(f"Release detected (peak amplitude {release_amp_deg:.1f} deg) -- "
              f"recording for {args.duration:.1f}s from this point (motor stays at 0)...")

        # Continue sampling from where wait_for_release left off, picking the clock
        # up at t_pre[-1] so the two segments splice into one continuous trace.
        t_post, theta_post = record_free_swing(cart, args.duration, t_offset=t_pre[-1])
        cart.set_motor(0)

        # Keep only the pre-release samples from the release point onward (drop the
        # hand-holding portion before release, which isn't part of the free swing).
        t = np.concatenate([t_pre[release_idx:], t_post])
        theta = np.concatenate([theta_pre[release_idx:], theta_post])

        if len(t) < 10:
            print("Not enough samples collected -- check the connection and try again.")
            return

        with open(args.out, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["t_s", "theta_rad", "theta_deg"])
            for tt, th in zip(t, theta):
                writer.writerow([tt, th, np.degrees(th)])
        print(f"Saved raw trace ({len(t)} samples) to {args.out}")

        peak_t, peak_amp = find_peaks(t, theta)
        if len(peak_t) < 2:
            print("Couldn't find at least two swing peaks -- was the pendulum actually "
                  "displaced and released before recording started?")
            return

        print("\nPeak envelope (time, amplitude):")
        for pt, pa in zip(peak_t, peak_amp):
            print(f"  t={pt:6.3f}s  amplitude={np.degrees(pa):6.2f} deg "
                  f"({pa / constants.PENDULUM_RAW_TO_RADIANS:.0f} raw counts)")

        A0 = peak_amp[0]
        print(f"\nStarting amplitude (first peak): {np.degrees(A0):.2f} deg")

        t_half_abs, half_life_interp = half_life_by_interpolation(peak_t, peak_amp)
        if t_half_abs is not None:
            print(f"Time to decay to half amplitude (direct interpolation): "
                  f"{half_life_interp:.3f} s  (at t={t_half_abs:.3f}s)")
        else:
            print(f"Amplitude never dropped to half of {np.degrees(A0):.2f} deg "
                  f"within the {args.duration:.1f}s recording -- rerun with a longer "
                  f"--duration.")

        tau, half_life_fit = half_life_by_exponential_fit(peak_t, peak_amp)
        if half_life_fit is not None:
            print(f"Time to decay to half amplitude (exponential fit across all "
                  f"{len(peak_t)} peaks, cross-check): {half_life_fit:.3f} s  (tau={tau:.3f}s)")

        # rough period / natural frequency from peak spacing, useful context
        if len(peak_t) >= 3:
            half_periods = np.diff(peak_t)
            period_est = 2 * np.median(half_periods)
            print(f"\nEstimated oscillation period: {period_est:.3f} s "
                  f"(natural frequency ~{1/period_est:.3f} Hz)")

        if args.plot:
            maybe_plot(t, theta, peak_t, peak_amp, t_half_abs)

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