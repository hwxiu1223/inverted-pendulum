function [u, xhat_nxt, cs] = mpc_controller(t, xhat, y_meas, cs, params, mpc, L_gain, C, dt)
    % hybrid_controller_mpc  energy swing-up + MPC balancing, driven by the
    % same Kalman filter estimate as the LQG version.
    %
    % LQG = LQR + Kalman filter.  This file = MPC + the SAME Kalman filter.
    % The only thing swapped out is the line u = -K*(xhat - target).
    %
    %   cs : controller state (warm-start input sequence, held input, mode,
    %        step counter). MPC runs slower than the estimator, so it has to
    %        remember what it decided last time.

theta_hat = xhat(3);
theta_wrapped = wrapToPi(theta_hat);
theta_target = sign(theta_wrapped) * pi;
theta_target_global = theta_hat - theta_wrapped + theta_target;

balancing = abs(theta_wrapped) > (pi - 0.5);

if balancing
    % ---------- MPC balance ----------
    % re-plan every mpc.every estimator steps (Ts = 0.01 s), or right away
    % if we just caught the pendulum; hold the input in between (zero-order hold)
    if ~cs.balancing || mod(cs.count, mpc.every) == 0
        if ~cs.balancing
            cs.U_warm = zeros(mpc.Nh, 1);   % old plan is meaningless after a swing
        end

        % error state about upright (x stays absolute; the reference is
        % handled inside the cost so we can preview future setpoints)
        z = xhat - [0; 0; theta_target_global; 0];

        % reference preview: where we want the cart at each future sample
        r_stack = zeros(4*mpc.Nh, 1);
        for i = 1:mpc.Nh
            r_stack(4*(i-1)+1) = mpc.x_ref_fun(t + i*mpc.Ts);
        end

        U = mpc_solve(mpc, z, r_stack, cs.U_warm);

        % receding horizon: apply ONLY the first move, keep the rest
        % (shifted by one sample) as the starting guess for next time
        cs.u_hold = U(1);
        cs.U_warm = [U(2:end); 0];
    end
    u = cs.u_hold;
else
    % ---------- swing-up (identical to the LQG version) ----------
    J = (4/3)*params.m*params.L^2;
    if abs(theta_wrapped) < 0.05 && abs(xhat(4)) < 0.1
        u = params.u_max;
    else
        g_mag = abs(params.g);
        E = 0.5*J*xhat(4)^2 - params.m*g_mag*params.L*cos(theta_hat);
        E_target = params.m*g_mag*params.L;
        ke = 24.0;
        kv = 0.2;
        u = ke*(E - E_target)*xhat(4)*cos(theta_hat) - kv*xhat(2);
    end
end
cs.balancing = balancing;
cs.count = cs.count + 1;

% ---------- actuator limits & walls (unchanged safety layer) ----------
if xhat(2) ~= 0 && sign(u) == sign(xhat(2))
    u_max_eff = params.u_max * max(0, 1 - abs(xhat(2))/params.v_max);
else
    u_max_eff = params.u_max;
end
u = max(min(u, u_max_eff), -u_max_eff);
u = u + track_limit_force(xhat(1), xhat(2), params);
u = max(min(u, params.u_max), -params.u_max);

% ---------- Kalman filter (identical to the LQG version) ----------
dxhat_nonlin = cartpend(xhat, u, params);
xhat_nxt = xhat + dt*dxhat_nonlin + dt*L_gain*(y_meas - C*xhat);

end

%% ------------------------------------------------------------------------
function U = mpc_solve(mpc, z, r_stack, U0)
    % Solves the MPC quadratic program
    %
    %   min_U  0.5*U'*H*U + f'*U  +  0.5*rho * sum(violation of |x_k| <= x_lim)^2
    %   s.t.   -u_max <= U_k <= u_max            (hard: the motor can't do more)
    %
    % U = [u_0 ... u_{Nh-1}] is the whole planned force sequence.
    % The track limit is a SOFT constraint (penalized, not forbidden): a hard
    % state constraint can become impossible to satisfy after a disturbance,
    % and an infeasible QP gives you no answer at all mid-balance.
    %
    % Method: projected Newton. Take a Newton step on the inputs that aren't
    % pinned at +-u_max, clip back into the box, backtrack if the cost didn't
    % drop. When no constraint is active this finishes in ONE step, and the
    % answer is exactly the (discrete) LQR input.

    H = mpc.H;
    f = mpc.F*z - mpc.GQ*r_stack;
    n = numel(f);
    umax = mpc.u_max;
    rho = mpc.rho_x;

    G = [mpc.Gx; -mpc.Gx];                 % predicted positions = Px*z + Gx*U
    px = mpc.Px*z;
    h = [mpc.x_lim - px; mpc.x_lim + px];  % so constraint is G*U <= h

    clampU = @(U) min(max(U, -umax), umax);
    cost = @(U) 0.5*(U'*H*U) + f'*U + 0.5*rho*sum(max(G*U - h, 0).^2);

    U = clampU(U0);
    for it = 1:mpc.max_iter
        v = max(G*U - h, 0);
        act = v > 0;
        g = H*U + f + rho*(G'*v);           % gradient

        pg = norm(U - clampU(U - g));        % optimality measure
        if pg < 1e-8
            break;
        end

        % inputs sitting on a bound and being pushed further into it are frozen
        eps_b = min(1e-6, pg);
        pinned = (U <= -umax + eps_b & g > 0) | (U >= umax - eps_b & g < 0);
        free = ~pinned;

        Hh = H + rho*(G(act,:)'*G(act,:));   % Hessian incl. active penalties
        d = zeros(n, 1);
        if any(free)
            d(free) = -Hh(free,free) \ g(free);
        end

        % backtracking line search along the clipped path
        a = 1;
        J0 = cost(U);
        for ls = 1:30
            Un = clampU(U + a*d);
            if cost(Un) <= J0 + 1e-4*(g'*(Un - U))
                break;
            end
            a = a/2;
        end
        U = Un;
    end
end

%% ------------------------------------------------------------------------
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