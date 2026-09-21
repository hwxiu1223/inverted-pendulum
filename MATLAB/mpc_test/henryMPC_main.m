% MPC version of henry_lqgsim.m
% Reuses cartpend.m, draw_cartpend.m, and pendsys.m unchanged.
clc; clear; close all;
params = pendsys();
    g = params.g;
    L = params.L;
    m = params.m;
    M = params.M;
    d = params.d;
    b = params.b;
    J = (4/3) * m * L^2;
    Delta = (M+m)*J - m^2*L^2;

    % linearized model about upright (identical to the LQG script)
    A = [0 1 0 0;
        0 -J*d/Delta          -m^2*g*L^2/Delta      -m*L*b/Delta;
        0 0 0 1;
        0 -m*L*d/Delta        -(M+m)*m*g*L/Delta     -(M+m)*b/Delta];
    B = [0; J/Delta; 0; m*L/Delta];
    C = [1 0 0 0;
         0 0 1 0];

    %% Kalman filter (identical to the LQG script)
    Qn = diag([0, params.q_dx, 0, params.q_dtheta]);
    Rn = diag([params.sigma_x^2, params.sigma_theta^2]);
    G = eye(4);
    [L_gain, ~, ~] = lqe(A, G, C, Qn, Rn);

    %% MPC setup (replaces K = lqr(A,B,Q,R))
    % same weights as the LQG design
    Q = [200 0 0 0;
         0 0 0 0;
         0 0 1000 0;
         0 0 0 0];
    R = 0.01;

    mpc.Ts = 0.01;          % MPC re-plans every 10 ms (estimator still runs at 2 ms)
    mpc.Nh = 30;            % horizon: 30 steps = 0.3 s of look-ahead
    mpc.u_max = params.u_max;
    mpc.x_lim = 0.35;       % keep predicted cart position within +-0.35 m
                            % (inside the 0.40 m soft-wall zone, so the walls
                            %  never have to fight the MPC)
    mpc.rho_x = 1e5;        % penalty weight on track-limit violation
    mpc.max_iter = 50;      % QP solver iteration cap

    % reference for the cart position (MPC sees this into the future)
    demo_maneuver = true;
    if demo_maneuver
        % deliberately asks for 0.45 m, which is OUTSIDE x_lim, to show
        % the MPC choosing to respect the constraint instead
        mpc.x_ref_fun = @(t) 0.45*(t >= 10 & t < 18) - 0.20*(t >= 18 & t < 24);
    else
        mpc.x_ref_fun = @(t) 0;
    end

    % discretize the linear model with a zero-order hold
    Mexp = expm([A B; zeros(1,5)] * mpc.Ts);
    Ad = Mexp(1:4, 1:4);
    Bd = Mexp(1:4, 5);

    % terminal cost = discrete LQR cost-to-go. This makes the MPC act like
    % an infinite-horizon LQR whenever no constraints are active.
    [K_d, P_term] = dlqr(Ad, Bd, Q, R);

    % stack the predictions: X = Phi*z + Gam*U, X = [x_1; x_2; ...; x_Nh]
    Nh = mpc.Nh;
    Phi = zeros(4*Nh, 4);
    Gam = zeros(4*Nh, Nh);
    Ak = eye(4);
    for i = 1:Nh
        Ak = Ad*Ak;
        Phi(4*(i-1)+1:4*i, :) = Ak;
        for j = 1:i
            Gam(4*(i-1)+1:4*i, j) = Ad^(i-j) * Bd;
        end
    end
    Qbar = kron(eye(Nh), Q);
    Qbar(end-3:end, end-3:end) = P_term;

    % cost J = sum x'Qx + u'Ru  ->  0.5*U'*H*U + f'*U  (built once, offline)
    mpc.H  = Gam'*Qbar*Gam + R*eye(Nh);
    mpc.H  = (mpc.H + mpc.H')/2;
    mpc.F  = Gam'*Qbar*Phi;
    mpc.GQ = Gam'*Qbar;
    mpc.Gx = Gam(1:4:end, :);     % rows of the prediction that are cart position
    mpc.Px = Phi(1:4:end, :);

    % sanity check: with no active constraints, MPC's first move is a linear
    % gain on the state. It should match the discrete LQR gain exactly.
    K_mpc = [1, zeros(1, Nh-1)] * (mpc.H \ mpc.F);
    fprintf('Unconstrained MPC first-move gain:  '); disp(K_mpc);
    fprintf('Discrete LQR gain (dlqr):           '); disp(K_d);
    fprintf('Continuous LQR gain (LQG script):   '); disp(lqr(A, B, Q, R));

    %% Simulation settings
    dt = 0.002;
    Tfinal = 30;
    N = round(Tfinal/dt) + 1;
    t_hist = (0:N-1)'*dt;
    mpc.every = round(mpc.Ts/dt);

    y_true = zeros(N,4);
    xhat_hist = zeros(N,4);
    u_hist = zeros(N,1);
    xref_hist = zeros(N,1);

    % controller memory
    cs.U_warm = zeros(Nh, 1);
    cs.u_hold = 0;
    cs.balancing = false;
    cs.count = 0;

    rng(1);

    %% Main loop
    for k = 1:N-1
        x_k = y_true(k,:)';
        xhat_k = xhat_hist(k,:)';

        y_meas = C*x_k + [params.sigma_x; params.sigma_theta].*randn(2,1);

        [u, xhat_next, cs] = mpc_controller(t_hist(k), xhat_k, y_meas, ...
            cs, params, mpc, L_gain, C, dt);

        x_next = rk4_step(@(tt,yy) cartpend(yy, u, params), t_hist(k), x_k, dt);

        y_true(k+1,:) = x_next';
        xhat_hist(k+1,:) = xhat_next;
        u_hist(k) = u;
        xref_hist(k) = mpc.x_ref_fun(t_hist(k));
    end
    u_hist(end) = u_hist(end-1);
    xref_hist(end) = mpc.x_ref_fun(t_hist(end));

    %% Animate
    figure('Name', 'Inverted Pendulum Simulation (MPC)', 'Position', [100, 100, 800, 600]);
    plot_every = round(0.1/dt);
    for k = 1:plot_every:N
        draw_cartpend(t_hist(1:k), y_true(1:k, :), params);
    end

    %% Tracking and constraint plot
    figure('Name', 'MPC: tracking, constraints, effort', 'Position', [950, 100, 700, 500]);
    subplot(2,1,1);
    plot(t_hist, y_true(:,1), 'b', t_hist, xref_hist, 'k--'); hold on;
    yline( mpc.x_lim, 'r:'); yline(-mpc.x_lim, 'r:');
    grid on; ylabel('cart x (m)');
    legend('actual', 'requested x_{ref}', 'MPC track limit', 'Location', 'southwest');
    title('Cart position vs. reference');
    subplot(2,1,2);
    plot(t_hist, u_hist, 'b'); hold on;
    yline( params.u_max, 'r:'); yline(-params.u_max, 'r:');
    grid on; ylabel('u (N)'); xlabel('Time (s)');
    title('Commanded force');

function y_next = rk4_step(f, t, y, dt)
    k1 = f(t,          y);
    k2 = f(t + dt/2,   y + dt/2*k1);
    k3 = f(t + dt/2,   y + dt/2*k2);
    k4 = f(t + dt,     y + dt*k3);
    y_next = y + dt/6*(k1 + 2*k2 + 2*k3 + k4);
end