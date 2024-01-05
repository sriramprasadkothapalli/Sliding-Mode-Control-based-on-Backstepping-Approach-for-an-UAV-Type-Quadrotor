% Clear the workspace and close all figures
clc; clear; close all;

% Solve the ODE and obtain the state vector
[t, x] = ode45(@QRBS, [0 15], [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]);

figure;
subplot(2,1,1);
plot3(x(:,7), x(:,9), x(:,11));
hold on;
plot3(sin(t),2*t,3*t,'r');
title('X, Y , Z Trajectory');
legend('Obtained Trajectory', 'Desired Trajectory');
grid on;

subplot(2,1,2);
plot3(x(:,1), x(:,3), x(:,5));
hold on;
plot3(sin(t),cos(t),0.1*t,'r');
title('Roll, Pitch, Yaw Trajectory');
legend('Obtained Trajectory', 'Desired Trajectory');
grid on;