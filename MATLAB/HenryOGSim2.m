clc; clear; close all;
params = pendsys();
    g = params.g;
    L = params.L; % length of the pendulum
    m = params.m;
    M = params.M;
    d = params.d;
    b = params.b;

    % Reduced-order LQR design
    A_r = [-d/M,        -m*g/M,          -b/(M*L);
            0,            0,               1;
           -d/(M*L),  -(M+m)*g/(M*L),  -(M+m)*b/(m*M*L^2)];
    B_r = [1/M; 0; 1/(M*L)];

    Q_r = [1 0 0;
           0 10 0;
           0 0 100];
    R = 0.001;
    K3 = lqr(A_r, B_r, Q_r, R);
    K = [0, K3];

    tspan = 0:0.1:30;
    y0 = [0; 0; 0; 0]; 
    
    [t,y] = ode45(@(t,y) hybrid_controller(t, y, params, K), tspan, y0);
    

    figure('Name', 'Inverted Pendulum Simulation', 'Position', [100, 100, 800, 600]);

    for k = 1:length(t)
        draw_cartpend(t(1:k), y(1:k, :), params);
    end