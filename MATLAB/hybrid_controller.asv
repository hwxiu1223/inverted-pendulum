function dy = hybrid_controller(t, y, params, K)
    % Extract states
    x = y(1); 
    dx = y(2); 
    theta = y(3); 
    dtheta = y(4);
    
    % Wrap theta between -pi and pi
    theta_wrapped = mod(theta + pi, 2*pi) - pi;
    
    % --- 1. LQR CATCH & BALANCE ---
    if abs(theta - pi) < 0.6
        u = -K * (y - [y(1); 0; pi; 0]); 
        
   % --- 2. ENERGY SWING-UP ---
    else
        % Detect if we are stuck at the bottom
        if abs(theta_wrapped) < 0.05 && abs(dtheta) < 0.05
            % STRONGER KICK for a 5kg cart
            u = 50; % 5kg cart, need to lower maybe. See ai
            
        else
            % NORMAL SWING-UP: Use absolute value of gravity for energy math
            g_mag = abs(params.g);
            
            E = 0.5 * params.m * params.L^2 * dtheta^2 - params.m * g_mag * params.L * cos(theta);
            E_target = params.m * g_mag * params.L;
            
            % INCREASED ENERGY GAIN
            ke = 4.0; 
            
            % Swing-up law
            u = ke * (E - E_target) * dtheta * cos(theta) - 0.2 * dx;
        end
        
        % INCREASED MOTOR SATURATION (Allow up to 150 Newtons)
        u = max(min(u, 150), -150); 
    end
    
    % Pass the computed force into your existing dynamics function
    dy = cartpend(y, u, params);
end