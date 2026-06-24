clc; clear; close all;

params = pendsys();

tspan = 0:0.1:10;
y0 = [0; 0; pi; 0.5];
[t, y] = ode45(@(t,y)cartpend(y,params),tspan,y0);

for k=1:length(t)
    draw_cartpend_bw(y(k,:),params);
end
