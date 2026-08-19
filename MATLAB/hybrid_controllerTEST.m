function dy = hybrid_controllerTEST(t, y, params, K)
    persistent in_lqr kicked
    if isempty(in_lqr), in_lqr = false; end
    if isempty(kicked), kicked = false; end

    x = y(1); dx = y(2); theta = y(3); dtheta = y(4);
    theta_wrapped = mod(theta + pi, 2*pi) - pi;
    theta_from_top = mod(theta - pi + pi, 2*pi) - pi;
    
    % Hysteresis: enter LQR at 0.5, only leave once past 0.7
    if in_lqr
        if abs(theta_from_top) > 0.7
            in_lqr = false;
        end
    else
        if abs(theta_from_top) < 0.5
            in_lqr = true;
        end
    end

    if in_lqr
        theta_for_control = pi + theta_from_top;
        y_wrapped = [y(1); y(2); theta_for_control; y(4)];
        u = -K * (y_wrapped - [y(1); 0; pi; 0]);
    else
        if ~kicked && abs(theta_wrapped) < 0.05 && abs(dtheta) < 0.05
            u = 2;
            kicked = true;
        else
            g_mag = abs(params.g);
            E = 0.5*params.m*params.L^2*dtheta^2 - params.m*g_mag*params.L*cos(theta);
            E_target = params.m*g_mag*params.L;
            ke = 0.01;
            u = ke*(E - E_target)*dtheta*cos(theta) - 0.2*dx;
            if abs(dtheta) < 0.02
                kicked = false;
            end
        end
        u = max(min(u, 4), -4);
    end
    dy = cartpend(y, u, params);
end