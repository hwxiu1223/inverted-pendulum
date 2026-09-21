% PID version of henry_lqgsim.m
% same cartpend.m, draw_cartpend.m, and pendsys.m
clc; clear; close all;
params = pendsys();

% PID gains
% Angle loop: keeps the pendulum upright
pid.Kp_th = 15;     % N per rad of angle error
pid.Ki_th = 1.0;    % N per (rad*s)
pid.Kd_th = 1.2;    % N per (rad/s)

% Position loop: keeps the cart near x_ref (output is SUBTRACTED, see controller)
pid.Kp_x  = 8;      % N per m
pid.Ki_x  = 0.8;    % N per (m*s)
pid.Kd_x  = 3.5;    % N per (m/s)
pid.x_ref = 0;      % desired cart position (m)

pid.u_int_max = 1.0;   % anti-windup: max force each integral term can add (N)
pid.tau_d = 0.01;      % derivative filter time constant, per stage (s)

% swing-up settings (ke, kv, switch angle same as LQG; tau_E and kx are new)
pid.ke = 24.0;
pid.kv = 0.2;
pid.kx = 2.0;          % cart-centering during swing-up
pid.tau_E = 0.15;      % energy-estimate low-pass time constant (s)
pid.switch_angle = 0.5;% hand over to PID within 0.5 rad of upright

% Simulation settings (same fixed-step structure as LQG)
dt = 0.002;
Tfinal = 30;
N = round(Tfinal/dt) + 1;
t_hist = (0:N-1)'*dt;

y_true   = zeros(N,4);   % true plant state
vel_est  = zeros(N,2);   % controller's filtered [dx, dtheta] (for plotting)
u_hist   = zeros(N,1);
y_true(1,:) = [0 0 0 0];

% controller state (replaces xhat in the LQG version)
cs.initialized = false;
cs.x_prev = 0;  cs.th_prev = 0;
cs.dx1 = 0;     cs.dth1 = 0;      % derivative filter, stage 1
cs.dx_f = 0;    cs.dth_f = 0;     % derivative filter, stage 2 (used)
cs.int_x = 0;   cs.int_th = 0;    % integrators
cs.E_f = NaN;                     % filtered swing-up energy
cs.balancing = false;

rng(1);

% Main loop
for k = 1:N-1
    x_k = y_true(k,:)';

    % same noisy sensors as the LQG sim: cart encoder + pendulum encoder
    y_meas = [x_k(1); x_k(3)] + [params.sigma_x; params.sigma_theta].*randn(2,1);

    [u, cs] = PIDcontroller(t_hist(k), y_meas, cs, params, pid, dt);

    x_next = rk4_step(@(tt,yy) cartpend(yy, u, params), t_hist(k), x_k, dt);

    y_true(k+1,:) = x_next';
    vel_est(k+1,:) = [cs.dx_f, cs.dth_f];
    u_hist(k) = u;
end
u_hist(end) = u_hist(end-1);

% Animate
figure('Name', 'Inverted Pendulum Simulation (PID)', 'Position', [100, 100, 800, 600]);
plot_every = round(0.1/dt);
for k = 1:plot_every:N
    draw_cartpend(t_hist(1:k), y_true(1:k, :), params);
end

% RK4 (identical to the LQG script)
function y_next = rk4_step(f, t, y, dt)
    k1 = f(t,          y);
    k2 = f(t + dt/2,   y + dt/2*k1);
    k3 = f(t + dt/2,   y + dt/2*k2);
    k4 = f(t + dt,     y + dt*k3);
    y_next = y + dt/6*(k1 + 2*k2 + 2*k3 + k4);
end