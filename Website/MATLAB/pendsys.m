function params = pendsys(params)
    arguments
        params = struct();
    end

    % constants
    params.g = -9.81; % gravitational acceleration (m/s^2)
    params.L = .15395;  % length of the pendulum (m) length of paint stick is 30.79cm
    params.m = .012;  % mass of the pendulum (kg) Mass of pendulum W/o drill holl
    params.M = .061; % mass of the cart (kg) Mass of cart, encoder, screws, ball bearings, and nuts
    params.W = .0381;  % cart width (m)
    params.H = .05; % cart height (m)
    params.wr = .015; % wheel radius (m)
    params.mr = .015; % mass radius (m)
    params.d = 5e-4;         % damping coeff
    params.b = 1e-6;        % pendulum pivot/bearing friction (N*m*s/rad)
    params.cd_cart = 1e-4;  % cart aerodynamic drag coeff, quadratic in v (N*s^2/m^2)
    params.cd_pend = 1e-6;  % pendulum aerodynamic drag coeff, quadratic in w (N*m*s^2/rad^2)

    params.r_belt = 0.006;                 % belt/pulley drive radius (m)
    params.rpm_max = 1000;                  % motor rated speed (RPM)
    params.tau_rated = 0.15 * 9.80665 * 0.01; % kgf*cm -> N*m (0.15 kgf*cm = 0.01471 N*m)
    params.u_max = 2.55; % params.tau_rated / params.r_belt; rated continuous force ceiling (N) ~= 2.45 N
    params.v_max = (2*pi*params.rpm_max/60) * params.r_belt; % max cart speed at motor's rated RPM (m/s) ~= 0.628 m/s
    
    params.x_min    = -0.48;   % available space on rail to left (m)
    params.x_max    =  0.48;   % available space on rail to right (m)
    params.x_margin =  0.08;  % distance from the end where the soft wall starts pushing back; accounts for cart W (m)
    params.k_wall   =  10;   % soft-wall spring stiffness (N/m) -- tune so it decelerates cart before the true end
    params.c_wall   =  2;    % soft-wall damping (N*s/m) -- kills residual velocity into the wall
    
    % sensor & process noise, used by the lqg design
    params.sigma_x     = 5e-4;  % cart position encoder noise, 1-sigma (m)   ~0.5 mm
    params.sigma_theta = 1e-2;  % pendulum angle encoder noise, 1-sigma (rad) ~0.57 deg
 
    % process noise intensity: how much we distrust the model's prediction
    % of the (unmeasured) velocity states between samples
    % larger values make the filter trust fresh measurements more and lag 
    % less, at the cost of noisier velocity estimates
    % needs tuning
    params.q_dx     = 1e-4;  % process noise on cart velocity state
    params.q_dtheta = 1e-3;  % process noise on pendulum angular velocity state
end
 