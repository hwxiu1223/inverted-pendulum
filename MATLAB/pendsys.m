function params = pendsys(params)
    arguments
        params = struct();
    end

    % constants
    params.g = -9.81; % gravitational acceleration (m/s^2)
    params.L = .3079/2;  % length of the pendulum (m) length of paint stick is 30.79cm
    params.m = .011;  % mass of the pendulum (kg) Mass of pendulum W/o drill holl is 11g. remeasure w/ drill hole
    params.M = .052; % mass of the cart (kg) Mass of cart, dowel, encoder, screws, and nuts: 0.052kg
    params.W = .0381;  % cart width (m) IS .0381 IRL
    params.H = .05; % cart height (m) IS .05 IRL
    params.wr = .015; % wheel radius
    params.mr = .015; % mass radius
    params.d = 0.02;
    params.b = 0.001;        % pendulum pivot/bearing friction (N*m*s/rad)
    params.cd_cart = 0.0025;  % cart aerodynamic drag coeff, quadratic in v (N*s^2/m^2)
    params.cd_pend = 0.00025;  % pendulum aerodynamic drag coeff, quadratic in w (N*m*s^2/rad^2)

    params.r_belt = 0.006;                 % belt/pulley drive radius (m)
    params.rpm_max = 1000;                  % motor rated speed (RPM)
    params.tau_rated = 0.15 * 9.80665 * 0.01; % kgf*cm -> N*m (0.15 kgf*cm = 0.01471 N*m)
    params.u_max = params.tau_rated / params.r_belt;               % rated continuous force ceiling (N) ~= 2.45 N
    params.v_max = (2*pi*params.rpm_max/60) * params.r_belt; % max cart speed at motor's rated RPM (m/s) ~= 0.628 m/s
    
    params.x_min    = -0.5;   % left end of rail (m), cart position coordinate
    params.x_max    =  0.5;   % right end of rail (m)
    params.x_margin =  0.2;  % distance from the end where the soft wall starts pushing back (m)
    params.k_wall   =  200;   % soft-wall spring stiffness (N/m) -- tune so it decelerates cart before the true end
    params.c_wall   =  20;    % soft-wall damping (N*s/m) -- kills residual velocity into the wall
end
