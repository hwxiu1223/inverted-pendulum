clc; clear; close all;

[params, A, B, C, K, L_gain] = lqg_design();

fprintf('Estimator poles (hopefully all negative real part):\n');
disp(eig(A - L_gain*C).');

% simulation settings (using fixed step)
dt = 0.002;     % estimator update / control period (s)
Tfinal = 30;
N = round(Tfinal/dt) + 1;
t_hist = (0:N-1)'*dt;

y_true = zeros(N,4); % true plant state
xhat_hist = zeros(N,4); % Kalman filter estimate
y_true(1,:) = [0 0 pi-0.1 0]; % initial plant state
xhat_hist(1,:) = [0 0 0 0];

rng(1); % reproducible sensor noise draw

for k = 1:N-1
    x_k = y_true(k,:)';
    xhat_k = xhat_hist(k,:)';

    % simulate noisy sensor reading
    y_meas = C*x_k + [params.sigma_x; params.sigma_theta].*randn(2,1);

    % controller computing u from estimate and advancing estimator by
    % one step
    [u, xhat_next] = hybrid_controller_lqg(t_hist(k), xhat_k, y_meas, ...
        params, K, L_gain, A, B, C, dt);

    % integrate true nonlinear plant one step
    x_next = rk4_step(@(tt,yy) cartpend(yy, u, params), t_hist(k), x_k, dt);

    y_true(k+1,:) = x_next';
    xhat_hist(k+1,:) = xhat_next;
end

% animate (using downsample)
figure('Name', 'Inverted Pendulum Simulation (LQG)', 'Position', [100, 100, 800, 600]);
plot_every = round(0.1/dt);   % redraw about every 0.1s of sim time
for k = 1:plot_every:N
    draw_cartpend(t_hist(1:k), y_true(1:k, :), params);
end

function y_next = rk4_step(f, t, y, dt)
    k1 = f(t,          y);
    k2 = f(t + dt/2,   y + dt/2*k1);
    k3 = f(t + dt/2,   y + dt/2*k2);
    k4 = f(t + dt,     y + dt*k3);
    y_next = y + dt/6*(k1 + 2*k2 + 2*k3 + k4);
end