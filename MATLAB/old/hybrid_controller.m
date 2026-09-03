function dy = hybrid_controller(t, y, params, K)
    % Extract states
    x = y(1); 
    dx = y(2); 
    theta = y(3); 
    dtheta = y(4);
    
    % Wrap theta between -pi and pi
    theta_wrapped = wrapToPi(theta);
    
    % LQR Controller
    if abs(theta_wrapped) > (pi - 0.5)
        theta_target = sign(theta_wrapped) * pi;
        theta_target_global = theta - theta_wrapped + theta_target;
        u = -K * (y - [0; 0; theta_target_global; 0]); 
    else
    
    % Swing-up Controller
    
    % Initial Kick
        if abs(theta_wrapped) < 0.05 && abs(dtheta) < 0.1
            u = params.u_max;

    % Swing-up Law
        else
            g_mag = abs(params.g);
            E = 0.5 * params.m * params.L^2 * dtheta^2 - params.m * g_mag * params.L * cos(theta);
            E_target = params.m * g_mag * params.L;
            ke = 2.4; 
            u = ke * (E - E_target) * dtheta * cos(theta);
        end
    end

    % --- Soft track-limit repulsion (virtual wall near rail ends) ---
    u_wall = track_limit_force(x, dx, params);
    u = u + u_wall;

    % Final hard clamp: never command more than the motor's absolute max,
    % even after adding the wall force.
    u = max(min(u, params.u_max), -params.u_max);
    
    % Pass the computed force into your existing dynamics function
    dy = cartpend(y, u, params);
end

function u_wall = track_limit_force(x, dx, params)
    % Soft virtual wall: as the cart's BODY EDGE (not just its centerline)
    % nears either end of the physical rail (params.x_min/x_max), ramp in a
    % spring-damper restoring force so the cart decelerates and reverses
    % before the body actually reaches the hard end of travel.
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