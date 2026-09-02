function dy = hybrid_controller(t, y, params, K)
    % Reset the one-shot "initial kick" flag at the start of every new sim
    persistent kicked
    if isempty(kicked) || t == 0
        kicked = false;
    end

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
        u_ctrl = -K * (y - [0; 0; theta_target_global; 0]);
    else

        % Swing-up Controller

        % Initial Kick (fires once per simulation, not every time the
        % pendulum happens to pass near the bottom at low speed)
        if ~kicked && abs(theta_wrapped) < 0.05 && abs(dtheta) < 0.1
            u_ctrl = params.u_max;
            kicked = true;

        % Swing-up Law
        else
            g_mag = abs(params.g);
            E = 0.5 * params.m * params.L^2 * dtheta^2 - params.m * g_mag * params.L * cos(theta);
            E_target = params.m * g_mag * params.L;
            ke = 80;

            % NOTE: an earlier version added a continuous cart-centering
            % term (-kx*x - cx*dx) here. That fights the energy pump
            % directly -- it opposes the cart motion needed to build
            % pendulum energy, especially early on when the swing torque
            % is still small, and traps the system in a stable
            % small-amplitude limit cycle that never reaches E_target.
            % The two-tier track-limit logic below (soft nudge +
            % predictive critical override) is sufficient on its own to
            % keep the cart on the rail, so no centering term is needed
            % inside the swing-up law itself.
            u_ctrl = ke * (E - E_target) * dtheta * cos(theta);
        end
    end

    % --- Track-limit protection ---
    u = apply_track_limit(x, dx, u_ctrl, params);

    % Pass the computed force into your existing dynamics function
    dy = cartpend(y, u, params);
end

function u = apply_track_limit(x, dx, u_ctrl, params)
    % Two-tier soft/hard rail protection.
    %
    % Tier 1 (soft): while inside the margin band but not yet in danger,
    % add a gentle spring-damper correction on top of the controller's
    % own command, same spirit as before.
    %
    % Tier 2 (critical): predict the cart's stopping distance from its
    % CURRENT speed using the motor's best-case deceleration
    % (u_max / M). If that predicted stopping point would overshoot the
    % rail, the controller's command is fully overridden (not just
    % added-then-clamped) with maximum braking, so the wall is never
    % starved of actuator authority by whatever the swing-up/LQR law
    % happens to be asking for at that instant.

    a_max = params.u_max / params.M;   % best-case deceleration (m/s^2)
    stop_dist = dx^2 / (2 * a_max);    % distance needed to stop from current speed
    buffer = 0.02;                     % safety margin beyond the hard limit (m)

    heading_to_max = (dx > 0) && (x + stop_dist > params.x_max - buffer);
    heading_to_min = (dx < 0) && (x - stop_dist < params.x_min + buffer);

    if heading_to_max || heading_to_min
        % Critical: override entirely, full braking authority.
        u = -params.u_max * sign(dx);
    else
        % Soft zone: cooperative nudge layered on the controller command.
        u_wall = 0;
        if x > (params.x_max - params.x_margin)
            penetration = x - (params.x_max - params.x_margin);
            u_wall = -params.k_wall * penetration - params.c_wall * max(dx, 0);
        elseif x < (params.x_min + params.x_margin)
            penetration = (params.x_min + params.x_margin) - x;
            u_wall = params.k_wall * penetration - params.c_wall * min(dx, 0);
        end
        u = u_ctrl + u_wall;
    end

    % Final hard clamp: never command more than the motor's absolute max.
    u = max(min(u, params.u_max), -params.u_max);
end