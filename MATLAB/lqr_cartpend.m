clc; clear; close all;

params = pendsys();

    g = params.g;
    L = params.L; % length of the pendulum
    m = params.m;
    M = params.M;
    d = params.d;

    A = [0 1 0 0;
        0 0 -m*g/M 0;
        0 0 0 1;
        0 0 (M+m)*g/(M*L) 0];

    B = [0; 1/M; 0; -1/(M*L)];

Q = [1 0 0 0;
    0 1 0 0;
    0 0 10 0;
    0 0 0 100];
R = 0.01;

K = lqr(A,B,Q,R);

tspan = 0:0.1:10;
y0 = [-3; 0; pi+.1; 0];
[t,y] = ode45(@(t,y)cartpend(y,params,d,-K*(y-[0; 0; pi; 0])),tspan,y0);

for k=1:length(t)
    draw_cartpend(y(k,:),params);
end
