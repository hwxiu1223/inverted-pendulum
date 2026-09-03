% This is the main simulation in MATLAB. 
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
 
    % matrices created from equations of motion
    % A acts as matrix for the system dynamics (state 'y')
    % B acts as matrix mapping the input to the state ('u')
    A = [0 1 0 0;
        0 -J*d/Delta          -m^2*g*L^2/Delta      -m*L*b/Delta;
        0 0 0 1;
        0 -m*L*d/Delta        -(M+m)*m*g*L/Delta     -(M+m)*b/Delta];
    B = [0; J/Delta; 0; m*L/Delta];

    % lqg
    % sensors on cart position and pendulum angle
    C = [1 0 0 0;
        0 0 1 0];

    % lqr gain

    % Q matrix represents the allowed deviation of a state variable from
    % its desired target
    % eg. Q(1,1) = 200 means our simulation will prioritize fixing our
    % position to its desired value over cart velocity (Q(2,2)) and
    % pendulum velocity (Q(4,4)), but less than the angle (Q(3,3))

    % R value represents the penalty associated with the usage of the
    % controller (Lower value faster correction; may need tuning)

    % K runs everything through the lqr cost function and returns an
    % optimized value that will be given to the input for control
    Q = [200 0 0 0;
        0 0 0 0;
        0 0 1000 0;
        0 0 0 0];
    R = 0.01;
    K = lqr(A, B, Q, R);


    % kalman filter gain
    % Qn represents any disturbances or unmodeled dynamics affecting the
    % plant states (true physical states)
    Qn = diag([0, params.q_dx, 0, params.q_dtheta]);

    % Rn represents any noise or inaccuracies affecting the sensors
    Rn = diag([params.sigma_x^2, params.sigma_theta^2]);

    % process noise enter through matrix G
    G = eye(4);

    % lqe acts as modified lqr running through new Kalman filter
    [L_gain, ~, ~] = lqe(A, G, C, Qn, Rn);

    % checking that filter's error converges
    fprintf('Estimator poles (hopefully all negative real part):\n');
    disp(eig(A - L_gain*C).');

    % simulation settings (using fixed step)
    dt = 0.002;     % estimator update / control period (s)
    Tfinal = 30;
    N = round(Tfinal/dt) + 1;    % total number of points for time grid
    t_hist = (0:N-1)'*dt;        % build fixed time grid

    y_true = zeros(N,4); % true plant state
    xhat_hist = zeros(N,4); % Kalman filter estimate
    y_true(1,:) = [0 0 0 0]; % initial plant state
    xhat_hist(1,:) = [0 0 0 0]; % estimator starts with no information

    rng(1); % reproducible sensor noise draw; fixes the random seed

    % main loop
    for k = 1:N-1
        % pull out this step's true state and estimated state
        x_k = y_true(k,:)';
        xhat_k = xhat_hist(k,:)';

        % simulate noisy sensor reading
        y_meas = C*x_k + [params.sigma_x; params.sigma_theta].*randn(2,1);
        
        % controller computing u from estimate and advancing estimator by
        % one step and return the force needed
        [u, xhat_next] = hybrid_controller_lqg(t_hist(k), xhat_k, y_meas, ...
            params, K, L_gain, A, B, C, dt);

        % advances true nonlinear plant state one step
        x_next = rk4_step(@(tt,yy) cartpend(yy, u, params), t_hist(k), x_k, dt);

        % log trajectories
        y_true(k+1,:) = x_next';
        xhat_hist(k+1,:) = xhat_next;
    end

    % animate (using downsample of roughly 1 frame per 0.1s)

    figure('Name', 'Inverted Pendulum Simulation (LQG)', 'Position', [100, 100, 800, 600]);
    plot_every = round(0.1/dt);   % redraw about every 0.1s of sim time
    for k = 1:plot_every:N
        draw_cartpend(t_hist(1:k), y_true(1:k, :), params);
    end
 
    % samples derivative f at four points across the interval and combines
    % them in a 1-2-2-1 weighted average
function y_next = rk4_step(f, t, y, dt)
    k1 = f(t,          y);
    k2 = f(t + dt/2,   y + dt/2*k1);
    k3 = f(t + dt/2,   y + dt/2*k2);
    k4 = f(t + dt,     y + dt*k3);
    y_next = y + dt/6*(k1 + 2*k2 + 2*k3 + k4);
end
 