function dy = hybrid_controller(t, y, params, K)
    % Extract states
    x = y(1); 
    dx = y(2); 
    theta = y(3); 
    dtheta = y(4);
    
    % Wrap theta between -pi and pi
    theta_wrapped = mod(theta - pi + pi, 2*pi) - pi;
    
    % LQR Controller
    if abs(theta_wrapped) < 0.6
        theta_target = theta - theta_wrapped;
        u = -K * (y - [y(1); 0; theta_target; 0]); 

    % Swing-up Controller
    else
            g_mag = abs(params.g);
            E = 0.5 * params.m * params.L^2 * dtheta^2 - params.m * g_mag * params.L * cos(theta);
            E_target = params.m * g_mag * params.L;
            
            % Energy gain
            ke = 20.0; 
            
            % Swing-up law
            u = ke * (E - E_target) * dtheta * cos(theta);
    end

    % --- Real motor force ceiling with torque-speed droop ---
    % A DC motor's available torque falls roughly linearly from its rated
    % value (at zero speed) to zero as it approaches its rated RPM/v_max.
    % This caps |u| accordingly instead of assuming full force is always
    % available, which is not physically realistic at high cart speed.
    if dx ~= 0 && sign(u) == sign(dx)
    % Commanding force that reinforces current motion -> droop applies
        u_max_eff = params.u_max * max(0, 1 - abs(dx)/params.v_max);
    else
    % Commanding braking/reversal -> full torque available
        u_max_eff = params.u_max;
end
u = max(min(u, u_max_eff), -u_max_eff);

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
    half_W = params.W/2;   % account for cart body thickness, not just center x

    if (x + half_W) > (params.x_max - margin)
        penetration = (x + half_W) - (params.x_max - margin);
        u_wall = -params.k_wall*penetration - params.c_wall*max(dx, 0);
    elseif (x - half_W) < (params.x_min + margin)
        penetration = (x - half_W) - (params.x_min + margin);
        u_wall = -params.k_wall*penetration - params.c_wall*min(dx, 0);
    end
end