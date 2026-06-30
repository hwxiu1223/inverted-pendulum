function dy = cartpend(y,u,params)

    params = pendsys();
    g = params.g;
    L = params.L; % Length of the penduLum
    m = params.m;
    M = params.M;
    d = params.d;
    Sy = sin(y(3));
    Cy = cos(y(3));
    D = m*L*L* (M+M*(1-Cy^2)); % damping (N)

    dy(1,1) = y(2);
    dy(2,1) = (1/D)*(-m^2*L^2*g*Cy*Sy + m*L^2*(m*L*y(4)^2*Sy - d*y(2))) + m*L*L*(1/D)*u;
    dy(3,1) = y(4);
    dy(4,1) = (1/D)*((m+M)*m*g*L*Sy - m*L*Cy*(m*L*y(4)^2*Sy - d*y(2))) - m*L*Cy*(1/D)*u;
end
