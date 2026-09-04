function [u, xhat_nxt] = hybrid_controller_lqg(t, xhat, y_meas, params, K, L_gain, A, B, C, dt)
    % hybrid_controller_lqg modified swing-up and lqg balancing controller
    % driven by Kalman filter estimate

    % current state estimate xhat
    % sensor measurement y_meas
    % updated state estimate xhat_next

    % cart position and pendulum angle measured
    % reconstruct cart and pendulum velocity with Kalman filter

theta_hat = xhat(3);
theta_wrapped = wrapToPi(theta_hat);
theta_target = sign(theta_wrapped) * pi;
theta_target_global = theta_hat - theta_wrapped + theta_target;

% lqr controller

if abs(theta_wrapped) > (pi - 0.5)
    u = -K * (xhat - [0; 0; theta_target_global; 0]);
else

    % swing-up controller

    % kick if stuck at bottom
    if abs(theta_wrapped)< 0.05 && abs(xhat(4)) < 0.1
        u = params.u_max;

    % swing-up law
    else
        g_mag = abs(params.g);
        E = 0.5 * params.m * params.L^2 * xhat(4)^2 - params.m * g_mag * params.L * cos(theta_hat);
        E_target = params.m * g_mag * params.L;
        ke = 2.4;
        kx = 1.0;
        kv = 0.2;
        u = ke * (E - E_target) * xhat(4) * cos(theta_hat) - kx * xhat(1) - kv * xhat(2);
    end
end

% track limits

u_wall = track_limit_force(xhat(1), xhat(2), params);
u = u + u_wall;

% motor limit
u = max(min(u, params.u_max), -params.u_max);

% kalman filter stuff using simple forward-Euler step of length dt

y_hat = C * xhat;
dxhat = A * xhat + B * u + L_gain * (y_meas - y_hat);
xhat_nxt = xhat + dt * dxhat;

function u_wall = track_limit_force(x, dx, params)
    u_wall = 0;
    margin = params.x_margin;

    if (x) > (params.x_max - margin)
        penetration = (x) - (params.x_max - margin);
        u_wall = -params.k_wall*penetration - params.c_wall*max(dx, 0);
    elseif (x) < (params.x_min + margin)
        penetration = (x) - (params.x_min + margin);
        u_wall = -params.k_wall*penetration - params.c_wall*min(dx, 0);
    end
end
end