function [params, A, B, C, K, L_gain] = lqg_design()
% lqg_design  Build the linear model, LQR gain, and Kalman (lqe) gain
% used by hybrid_controller_lqg.m.
%
% This is factored out of henry_lqgsim.m so it can be called once,
% either from a MATLAB simulation script or from the Python control
% bridge (matlab_bridge.py) via the MATLAB Engine.

    params = pendsys();
    g = params.g;
    L = params.L;
    m = params.m;
    M = params.M;
    d = params.d;
    b = params.b;

    A = [0 1 0 0;
        0 -d/M -m*g/M -b/(M*L);
        0 0 0 1;
        0 -d/(M*L) -(M+m)*g/(M*L) -(M+m)*b/(m*M*L^2)];
    B = [0; 1/M; 0; 1/(M*L)];

    % sensors on cart position and pendulum angle
    C = [1 0 0 0;
        0 0 1 0];

    % lqr gain
    Q = [100 0 0 0;
        0 0 0 0;
        0 0 1000 0;
        0 0 0 0];
    R = 0.01;
    K = lqr(A, B, Q, R);

    % kalman filter gain
    Qn = diag([0, params.q_dx, 0, params.q_dtheta]);      % process noise
    Rn = diag([params.sigma_x^2, params.sigma_theta^2]);  % measurement noise
    G = eye(4);
    [L_gain, ~, ~] = lqe(A, G, C, Qn, Rn);
end