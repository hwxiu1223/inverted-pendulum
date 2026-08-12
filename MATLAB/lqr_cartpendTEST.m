clc; clear; close all;
params = pendsys();
    g = params.g;
    L = params.L; % length of the pendulum
    m = params.m;
    M = params.M;
    d = params.d;
    b = params.b;

    % Reduced-order design: drop x entirely from the LQR problem, since it
    % has no target and no effect on any other state's dynamics. Designing
    % on the full 4-state system with Q(1,1)=0 leaves an undetectable
    % zero-eigenvalue mode (x is a pure integrator), which is exactly what
    % makes lqr() fail to find a stabilizing Riccati solution.
    A_r = [-d/M,        -m*g/M,          -b/(M*L);
            0,            0,               1;
           -d/(M*L),  -(M+m)*g/(M*L),  -(M+m)*b/(m*M*L^2)];
    B_r = [1/M; 0; 1/(M*L)];

Q_r = [1 0 0;
       0 10 0;
       0 0 100];
R = 0.001;
K3 = lqr(A_r, B_r, Q_r, R);   % gains on [dx; theta; dtheta]
K = [0, K3];                  % pad a hard zero gain for x

tspan = 0:0.5:30;
y0 = [-3; 0; pi+.2; 0];

% (y - [y(1);0;pi;0]) keeps the x-error at exactly zero (belt-and-suspenders
% on top of K(1)=0), so there is no position target at all.
[t,y] = ode45(@(t,y)cartpend(y, -K*(y-[y(1); 0; pi; 0]), params), tspan, y0);

figure('Name', 'Inverted Pendulum Simulation', 'Position', [100, 100, 800, 600]);
for k = 1:length(t)
    draw_cartpend(t(1:k), y(1:k, :), params);
end