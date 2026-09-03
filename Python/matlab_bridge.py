# matlab_bridge.py
"""
Wraps a MATLAB Engine session so the Python control CLI can start/stop
MATLAB on demand and call the existing LQG swing-up/balance controller
(hybrid_controller_lqg.m) once per control loop iteration, instead of
Python re-implementing any of the control or estimation math.

Requires:
    pip install matlabengine     # version must match your installed MATLAB release
    MATLAB Control System Toolbox (lqr/lqe are used once, at connect() time)

The .m files (pendsys.m, lqg_design.m, hybrid_controller_lqg.m, cartpend.m)
must live in the folder you pass in as `matlab_dir`.
"""

import matlab.engine


def _col(vec):
    """Python list -> MATLAB Nx1 column vector (matlab.double)."""
    return matlab.double([[v] for v in vec])


class MatlabLQG:
    def __init__(self, matlab_dir):
        self.matlab_dir = matlab_dir
        self.eng = None
        self.params = None
        self.A = self.B = self.C = self.K = self.L_gain = None

    @property
    def connected(self):
        return self.eng is not None

    def connect(self):
        """Start MATLAB and run the LQR/Kalman design once. Idempotent."""
        if self.connected:
            return
        print("Starting MATLAB engine (first call can take ~10-20s)...")
        self.eng = matlab.engine.start_matlab()
        self.eng.addpath(self.matlab_dir, nargout=0)
        (self.params, self.A, self.B, self.C,
         self.K, self.L_gain) = self.eng.lqg_design(nargout=6)
        print("MATLAB engine ready. u_max = %.3f N" % self.params['u_max'])

    def disconnect(self):
        """Stop MATLAB. Safe to call even if never connected."""
        if not self.connected:
            return
        self.eng.quit()
        self.eng = None

    def step(self, t, xhat, x_meas, theta_meas, dt):
        """
        Advance the controller + estimator by one real-time tick.

        xhat:               python list [x, dx, theta, dtheta] - current estimate
        x_meas, theta_meas: latest sensor readings (meters, radians)
        dt:                 measured wall-clock time since the previous call (s)
                            -- NOT assumed fixed; pass whatever you actually measured.

        Returns (u, xhat_next):
            u:         float force command (N), already saturated at +-u_max
                        by hybrid_controller_lqg.m
            xhat_next: python list, the updated estimate to pass in next time
        """
        if not self.connected:
            raise RuntimeError("MatlabLQG.step() called before connect()")

        xhat_m = _col(xhat)
        y_meas_m = _col([x_meas, theta_meas])

        u, xhat_next_m = self.eng.hybrid_controller_lqg(
            float(t), xhat_m, y_meas_m, self.params,
            self.K, self.L_gain, self.A, self.B, self.C, float(dt),
            nargout=2)

        xhat_next = [row[0] for row in xhat_next_m]
        return float(u), xhat_next