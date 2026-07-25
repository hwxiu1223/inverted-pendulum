clc; clear; close all;

params = pendsys();

    g = params.g;
    L = params.L; % length of the pendulum
    m = params.m;
    M = params.M;
    d = params.d;

    A = [0 1 0 0;
        0 -d/M -m*g/M 0;
        0 0 0 1;
        0 -1*d/(M*L) -1*(M+m)*g/(M*L) 0];

    B = [0; 1/M; 0; 1/(M*L)];

Q = [1 0 0 0;
    0 1 0 0;
    0 0 10 0;
    0 0 0 100];
R = 0.001;

K = lqr(A,B,Q,R);

tspan = 0:0.5:30;
y0 = [-3; 0; pi+.1; 0];
[t,y] = ode45(@(t,y)cartpend(y,-K*(y-[0; 0; pi; 0]),params),tspan,y0);

figure('Name', 'Inverted Pendulum Simulation', 'Position', [100, 100, 800, 600]);


for k = 1:length(t)
    draw_cartpend(t(1:k), y(1:k, :), params);
end
