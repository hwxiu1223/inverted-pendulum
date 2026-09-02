function dy = cartpend(y,u,params)

    g = params.g;
    L = params.L; % Length of the penduLum
    m = params.m;
    M = params.M;
    d = params.d;
    Sy = sin(y(3));
    Cy = cos(y(3));
    D = m*L*L* (M+m*(1-Cy^2)); % damping (N)
    b = params.b;       % pendulum pivot (bearing) friction
    cd_cart = params.cd_cart;  % cart aerodynamic drag coeff (quadratic in v)
    cd_pend = params.cd_pend;  % pendulum aerodynamic drag coeff (quadratic in w)


    F_cart = m*L*y(4)^2*Sy - d*y(2) - cd_cart*y(2)*abs(y(2));

    % Pivot-side damping "torque": linear bearing friction + nonlinear
    % aerodynamic drag on the swinging pendulum (scales as w|w|)
    F_pivot = b*y(4) + cd_pend*y(4)*abs(y(4));

    dy(1,1) = y(2);
    dy(2,1) = (1/D)*(-m^2*L^2*g*Cy*Sy + m*L^2*F_cart + m*L*Cy*F_pivot) + m*L*L*(1/D)*u;
    dy(3,1) = y(4);
    dy(4,1) = (1/D)*((m+M)*m*g*L*Sy - m*L*Cy*F_cart - (m+M)*F_pivot) - m*L*Cy*(1/D)*u;
end