function params = pendsys(params)
    arguments
        params = struct();
    end

    % constants
    params.g = 9.81; % gravitational acceleration (m/s^2)
    params.L = 2.0;  % length of the pendulum (m)
    params.m = 1.0;  % mass of the pendulum (kg)
    params.M = 5.0; % mass of the cart (kg)
    params.W = 1;  % cart width
    params.H = .5; % cart height
    params.wr = .2; % wheel radius
    params.mr = .3; % mass radius
    params.d = 1;
end
