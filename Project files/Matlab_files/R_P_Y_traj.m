% Clear the workspace and close all figures
clc; clear; close all;

% Solve the ODE and obtain the state vector
[t, x] = ode45(@QRBS, [0 15], [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]);
yaw_desired = 0.1*t;
yaw_obtained = x(:, 5);
yaw_error = yaw_desired - yaw_obtained;

pitch_desired = cos(t);
pitch_obtained = x(:, 3);
pitch_error = pitch_desired - pitch_obtained;

roll_desired = sin(t);
roll_obtained = x(:, 1);
roll_error = roll_desired - roll_obtained;


figure;

% Combined subplot for Yaw Trajectory and Yaw Tracking Error
subplot(3, 2, 1);

% Plot Yaw Trajectory
plot(x(:, 5));
hold on;
plot(0.1 * t, 'Color', [1, 0.7529, 0.7961]);
title('Yaw Trajectory');
%legend('Obtained Trajectory', 'Desired Trajectory');
grid on;

% Plot Yaw Tracking Error
subplot(3, 2, 2);  
plot(t, yaw_error);
title('Yaw Tracking Error');
xlabel('Time (s)');
ylabel('Yaw Tracking Error');
grid on;

% New subplot for Pitch Trajectory and Pitch Tracking Error
subplot(3, 2, 3);
plot(x(:, 3));
hold on;
plot(cos(t), 'Color', [1, 0.7529, 0.7961]);
title('Pitch Trajectory');
%legend('Obtained Trajectory', 'Desired Trajectory');
grid on;

subplot(3, 2, 4);
plot(t, pitch_error);
title('Pitch Tracking Error');
xlabel('Time (s)');
ylabel('Pitch Tracking Error');
grid on;


subplot(3,2,5);
plot(x(:,1));
hold on;
plot(sin(t),'Color',[1, 0.7529, 0.7961]);
title('Roll Trajectory');
%legend('Obtained Trajectory', 'Desired Trajectory');
grid on;

subplot(3, 2, 6);
plot(t, roll_error);
title('Roll Tracking Error');
xlabel('Time (s)');
ylabel('Roll Tracking Error');
grid on;

sgtitle('Roll, Pitch, Yaw Trajectories and Tracking Errors');
