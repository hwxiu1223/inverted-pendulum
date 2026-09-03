function [params, A, B, C, K, L_gain] = lqg_design()
% LQG_DESIGN  One-time model + gain setup for the real-time hardware loop.
%
% This is the "design phase" that henry_lqgsim.m used to do inline before
% its simulation loop, pulled out into its own callable function so
% matlab_bridge.py (MatlabLQG.connect()) can call it once at start-of-
% control from Python:
%
%     [params, A, B, C, K, L_gain] = eng.lqg_design(nargout=6)
%
% hybrid_controller_lqg.m then gets called once per real-time tick with
% these same A, B, C, K, L_gain, and params, exactly like it's called
% inside henry_lqgsim.m's simulation loop.

    params = pendsys();
    g = params.g;
    L = params.L;
    m = params.m;
    M = params.M;
    d = params.d;
    b = params.b;
    J = (4/3) * m * L^2;
    Delta = (M+m)*J - m^2*L^2;

    % linearized dynamics about the upright equilibrium
    A = [0 1 0 0;
        0 -J*d/Delta          -m^2*g*L^2/Delta      -m*L*b/Delta;
        0 0 0 1;
        0 -m*L*d/Delta        -(M+m)*m*g*L/Delta     -(M+m)*b/Delta];
    B = [0; J/Delta; 0; m*L/Delta];

    % sensors: cart position and pendulum angle
    C = [1 0 0 0;
        0 0 1 0];

    % LQR gain (same cost weights as henry_lqgsim.m)
    Q = [200 0 0 0;
        0 0 0 0;
        0 0 1000 0;
        0 0 0 0];
    R = 0.01;
    K = lqr(A, B, Q, R);

    % Kalman filter gain
    Qn = diag([0, params.q_dx, 0, params.q_dtheta]);
    Rn = diag([params.sigma_x^2, params.sigma_theta^2]);
    G = eye(4);
    [L_gain, ~, ~] = lqe(A, G, C, Qn, Rn);

    fprintf('Estimator poles (hopefully all negative real part):\n');
    disp(eig(A - L_gain*C).');
end