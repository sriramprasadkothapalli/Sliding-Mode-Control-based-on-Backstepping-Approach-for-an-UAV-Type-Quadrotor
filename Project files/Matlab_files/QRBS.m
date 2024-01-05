function dx = QRBS(t, x)
    % System parameters
    m = 486e-3;
    d = 25e-2;
    ob = x(13) - x(14) + x(15) - x(16);
    beta0 = 189.63;
    beta1 = 6.0612;
    beta2 = 0.0122;
    b = 280.19;
    g = 9.8;
    Ix = 3.8278e-3; Iy = 3.8288e-3; Iz = 7.6566e-3; 
    Kfax = 5.567e-4; Kfay = 5.567e-4; Kfaz = 6.354e-4;
    Kftx = 5.567e-4; Kfty = 5.567e-4; Kftz = 6.354e-4; 
    Jr = 2.8385e-5;

    % System dynamics coefficients
    a1 = (Iy - Iz) / Ix;
    a2 = -Kfax / Ix;
    a3 = -Jr / Ix;
    a4 = (Iz - Ix) / Iy;
    a5 = -Kfay / Iy;
    a6 = Jr / Iy;
    a7 = (Ix - Iy) / Iz;
    a8 = -Kfaz / Iz;
    a9 = -Kftx / m;
    a10 = -Kfty / m;
    a11 = -Kftz / m;
    b1 = d / Ix;
    b2 = d / Iy;
    b3 = 1 / Iz;

    % Desired trajectory
    xd = [sin(t) cos(t) cos(t) -sin(t) 0.1*t 0.1 sin(t) cos(t) 2*t 2 3*t 3]';

    % Error variables and tuning parameters
    z = zeros(12, 1);
    alpha = [0.2285737, 0.2285737, 0.2285737, 0.2285737, 0.2285737, 0.2285737, 0.2285737, 0.2285737, 0.2285737, 0.2285737, 0.2285737, 0.2285737];
    xdd = [cos(t) -sin(t) -sin(t) -cos(t) 0.1 0 cos(t) -sin(t) 2 0 3 0];

    % Compute tracking errors
    for w = 1:2:11
        z(w) = xd(w) - x(w);
    end
    for q = 2:2:12
        z(q) = x(q) - xdd(q - 1) - alpha(q - 1) * z(q - 1); % Tune alpha parameters
    end

    % Control gains
    q1 = 0.1; q2 = 0.1; q3 = 0.1; q4 = 0.1; q5 = 0.1; q6 = 0.1;
    k1 = 0.1; k2 = 0.1; k3 = 0.1; k4 = 0.1; k5 = 0.1; k6 = 0.1;

    % Lyapunov-like functions
    V1 = z(1)^2 / 2; V3 = z(3)^2 / 2; V2 = (V1 + z(2)^2) / 2; V4 = (V3 + z(4)^2) / 2;

    % Control inputs
    U2 = 1 / b1 * (-q1 * sign(z(2)) - k1 * z(2) - a1 * x(4) * x(6) - a2 * x(2)^2 - a3 * ob * x(4) + xdd(2) + (xd(2) - x(2)));
    U3 = 1 / b2 * (-q2 * sign(z(4)) - k2 * z(4) - a4 * x(2) * x(6) - a5 * x(4)^2 - a6 * ob * x(2) + xdd(4) + (xd(4) - x(4)));
    U4 = 1 / b3 * (-q3 * sign(z(6)) - k3 * z(6) - a7 * x(2) * x(6) - a8 * x(6)^2 + xdd(6) + (xd(6) - x(6)));

    U1 = m / (cos(x(1)) * cos(x(3))) * (-q6 * sign(z(12)) - k6 * z(12) - a11 * x(12) + xdd(12) + (xd(12) - x(12)) + g); % x1 = phi, x3 = theta
    if U1 == 0
        Ux = 0; Uy = 0;
    else
        Ux = m / U1 * (-q4 * sign(z(8)) - k4 * z(8) - a9 * x(8) + xdd(8) + (xd(8) - x(8)));
        Uy = m / U1 * (-q5 * sign(z(10)) - k5 * z(10) - a10 * x(10) + xdd(10) + (xd(10) - x(10)));
    end

    % System dynamics
    dx1 = x(2);
    dx2 = a1 * x(4) * x(6) + a2 * (x(2)^2) + a3 * ob * x(4) + b1 * U2;
    dx3 = x(4);
    dx4 = a4 * x(2) * x(6) + a5 * (x(4)^2) + a6 * ob * x(2) + b2 * U3;
    dx5 = x(6);
    dx6 = a7 * x(2) * x(4) + a8 * (x(6)^2) + b3 * U4;
    dx7 = x(8);
    dx8 = a9 * x(8) + (Ux * (U1 / m));
    dx9 = x(10);
    dx10 = a10 * x(10) + (Uy * (U1 / m));
    dx11 = x(12);
    dx12 = a11 * x(12) + ((cos(x(1)) * cos(x(3)) * U1) / m) - g;
    %Rotor dynamics
    dx13 = b * V1 - beta0 - beta1 * x(13) - beta2 * x(13)^2; 
    dx14 = b * V2 - beta0 - beta1 * x(14) - beta2 * x(14)^2;
    dx15 = b * V1 - beta0 - beta1 * x(15) - beta2 * x(15)^2;
    dx16 = b * V1 - beta0 - beta1 * x(16) - beta2 * x(16)^2;

    % State vector
    dx = [dx1; dx2; dx3; dx4; dx5; dx6; dx7; dx8; dx9; dx10; dx11; dx12; dx13; dx14; dx15; dx16];
end

