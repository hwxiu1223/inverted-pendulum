function [u, cs] = PIDcontroller(t, y_meas, cs, params, pid, dt)
    % energy swing-up + PID balancing controller
    %
    % Same job as hybrid_controller_lqg, but with NO model-based estimator.
    %   y_meas : [x_meas; theta_meas] noisy encoder readings (same sensors as LQG)
    %   cs     : controller state struct (integrators, derivative filters, mode)
    %            -> this plays the role xhat played in the LQG version: it's the
    %               controller's "memory", passed in and handed back every step
    %
    % Velocities are NOT estimated by a Kalman filter here. They come from a
    % "dirty derivative": finite-difference the measurement, then low-pass it.

x_m  = y_meas(1);
th_m = y_meas(2);

% ---------- 1. velocity estimates (filtered derivatives) ----------
if ~cs.initialized
    % first call: seed the "previous" samples so the first derivative is 0
    cs.x_prev = x_m;
    cs.th_prev = th_m;
    cs.initialized = true;
end

% raw finite differences (VERY noisy: sigma_theta/dt ~ 5 rad/s of noise)
dx_raw  = (x_m  - cs.x_prev)/dt;
dth_raw = (th_m - cs.th_prev)/dt;
cs.x_prev = x_m;
cs.th_prev = th_m;

% two cascaded first-order low-pass filters, each with time constant tau_d
% (a single stage lets too much high-frequency noise through; see notes)
a = dt/(pid.tau_d + dt);
cs.dx1  = cs.dx1  + a*(dx_raw  - cs.dx1);
cs.dth1 = cs.dth1 + a*(dth_raw - cs.dth1);
cs.dx_f  = cs.dx_f  + a*(cs.dx1  - cs.dx_f);
cs.dth_f = cs.dth_f + a*(cs.dth1 - cs.dth_f);

dx_f  = cs.dx_f;
dth_f = cs.dth_f;

% ---------- 2. angle target (same wrapping logic as the LQG version) ----------
theta_wrapped = wrapToPi(th_m);
theta_target = sign(theta_wrapped) * pi;
theta_target_global = th_m - theta_wrapped + theta_target;

balancing = abs(theta_wrapped) > (pi - pid.switch_angle);

% bumpless-ish transfer: wipe the integrators every time we (re)enter
% balance mode so stale error from a previous attempt can't kick the cart
if balancing && ~cs.balancing
    cs.int_th = 0;
    cs.int_x  = 0;
end
cs.balancing = balancing;

if balancing
    % ---------- 3a. PID balance ----------
    % errors (setpoint - measurement)
    e_th = theta_target_global - th_m;
    e_x  = pid.x_ref - x_m;

    % integrators with clamping anti-windup: each integral term may
    % contribute at most u_int_max newtons
    cs.int_th = cs.int_th + e_th*dt;
    cs.int_x  = cs.int_x  + e_x*dt;
    if pid.Ki_th > 0
        cs.int_th = max(min(cs.int_th, pid.u_int_max/pid.Ki_th), -pid.u_int_max/pid.Ki_th);
    end
    if pid.Ki_x > 0
        cs.int_x = max(min(cs.int_x, pid.u_int_max/pid.Ki_x), -pid.u_int_max/pid.Ki_x);
    end

    % derivative on MEASUREMENT (setpoints are constant, so de/dt = -d(meas)/dt;
    % this also avoids a derivative "kick" if the setpoint ever jumps)
    u_th = pid.Kp_th*e_th + pid.Ki_th*cs.int_th - pid.Kd_th*dth_f;
    u_x  = pid.Kp_x *e_x  + pid.Ki_x *cs.int_x  - pid.Kd_x *dx_f;

    % NOTE THE MINUS SIGN: the position loop is "reverse acting".
    % To move the cart right you first have to push it LEFT so the
    % pendulum leans right, then chase it. (Non-minimum-phase behavior.)
    u = u_th - u_x;
else
    % ---------- 3b. swing-up (same energy law as the LQG version) ----------
    J = (4/3)*params.m*params.L^2;
    g_mag = abs(params.g);

    if abs(theta_wrapped) < 0.05 && abs(dth_f) < 0.1
        % initial kick
        u = params.u_max;
    else
        E = 0.5*J*dth_f^2 - params.m*g_mag*params.L*cos(th_m);
        E_target = params.m*g_mag*params.L;

        % the energy estimate uses dth_f^2, and noise in dth_f makes E jitter
        % by ~10% at the bottom of the swing. Energy only changes slowly, so
        % we low-pass it; without this the pump stalls at ~90% of E_target
        if isnan(cs.E_f)
            cs.E_f = E;
        end
        cs.E_f = cs.E_f + dt/(pid.tau_E + dt)*(E - cs.E_f);

        % extra -kx*x centering term (new vs LQG): noisier velocity estimates
        % made the cart wander into the walls during swing-up without it
        u = pid.ke*(cs.E_f - E_target)*dth_f*cos(th_m) ...
            - pid.kv*dx_f - pid.kx*x_m;
    end
end

% ---------- 4. actuator limits & track walls (unchanged from LQG) ----------
if dx_f ~= 0 && sign(u) == sign(dx_f)
    u_max_eff = params.u_max * max(0, 1 - abs(dx_f)/params.v_max);
else
    u_max_eff = params.u_max;
end
u = max(min(u, u_max_eff), -u_max_eff);

u = u + track_limit_force(x_m, dx_f, params);

u = max(min(u, params.u_max), -params.u_max);

end

% adjusted force as cart enters marginal distance from wall (same as LQG)
function u_wall = track_limit_force(x, dx, params)
    u_wall = 0;
    margin = params.x_margin;
    if x > (params.x_max - margin)
        penetration = x - (params.x_max - margin);
        u_wall = -params.k_wall*penetration - params.c_wall*max(dx, 0);
    elseif x < (params.x_min + margin)
        penetration = x - (params.x_min + margin);
        u_wall = -params.k_wall*penetration - params.c_wall*min(dx, 0);
    end
end