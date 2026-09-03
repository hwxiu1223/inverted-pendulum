clc; clear; close all;
params = OGpendsys();
g = params.g; L = params.L; m = params.m; M = params.M; d = params.d;

A = [0 1 0 0;
     0 -d/M -m*g/M 0;
     0 0 0 1;
     0 -1*d/(M*L) -1*(M+m)*g/(M*L) 0];
B = [0; 1/M; 0; 1/(M*L)];

Q = diag([1 1 10 100]);
R = 0.001;
K = lqr(A,B,Q,R);

% --- Realism parameters (these are what were missing) ---
dt_ctrl = 0.02;              % 50 Hz digital control loop, not continuous ideal feedback
dt_sim  = 0.005;              % physics integration resolution within each control step
Tfinal  = 60;
nsteps  = round(Tfinal/dt_ctrl);

sensor_noise_std  = [0.004; 0.01; 0.002; 0.008]; % encoder/IMU noise, per state
process_noise_std = 30;                        % unmeasured disturbance force (N): friction, air, cogging
u_max = 25;                                      % actuator can't push infinitely hard

ref = [0; 0; pi; 0];
y_current = [-3; 0; pi+0.1; 0];
t_all = 0;
y_all = y_current';

for k = 1:nsteps
    % controller only ever sees a noisy measurement
    y_measured = y_current + sensor_noise_std .* randn(4,1);

    % control computed once per sample, held constant until next sample (zero-order hold)
    u = -K*(y_measured - ref);
    u = max(min(u, u_max), -u_max);

    % real disturbance entering the physical plant, unknown to the controller
    w = process_noise_std * randn;

    tspan_k = (k-1)*dt_ctrl : dt_sim : k*dt_ctrl;
    [tk, yk] = ode45(@(t,y) cartpend(y, u + w, params), tspan_k, y_current);

    y_all = [y_all; yk(2:end,:)];
    t_all = [t_all; tk(2:end)];
    y_current = yk(end,:)';
end

t = t_all;
y = y_all;
draw_every = 20;
figure('Name', 'Inverted Pendulum Simulation', 'Position', [100, 100, 800, 600]);
for k = 1:draw_every:length(t)
    draw_cartpend(t(1:k), y(1:k, :), params);
end