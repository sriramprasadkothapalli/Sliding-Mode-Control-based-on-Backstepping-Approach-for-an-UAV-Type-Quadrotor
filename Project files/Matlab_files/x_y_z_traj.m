[t, x] = ode45(@QRBS, [0 15], [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]);
x_desired = sin(t);
x_obtained = x(:, 7);
x_error = x_desired - x_obtained;

y_desired = 2*t;
y_obtained = x(:, 9);
y_error = y_desired - y_obtained;

z_desired = 3*t;
z_obtained = x(:, 11);
z_error = z_desired - z_obtained;

figure;
subplot(3,2,1);
plot(x(:,7));
hold on;
plot(sin(t),'Color',[1, 0.7529, 0.7961]);
title('x Trajectory');
%legend('Obtained Trajectory', 'Desired Trajectory');
grid on;

subplot(3, 2, 2);
plot(t, x_error);
title('x Tracking Error');
xlabel('Time (s)');
ylabel('x Tracking Error');
grid on;

subplot(3, 2, 3);
plot(x(:,9));
hold on;
plot(2*t,'Color',[1, 0.7529, 0.7961]);
title('y Trajectory');
%legend('Obtained Trajectory', 'Desired Trajectory');
grid on;

subplot(3, 2, 4);
plot(t, y_error);
title('y Tracking Error');
xlabel('Time (s)');
ylabel('y Tracking Error');
grid on;

subplot(3, 2, 5);
plot(x(:,11));
hold on;
plot(3*t,'Color',[1, 0.7529, 0.7961]);
title('z Trajectory');
%legend('Obtained Trajectory', 'Desired Trajectory');
grid on;

subplot(3, 2, 6);
plot(t, z_error);
title('z Tracking Error');
xlabel('Time (s)');
ylabel('z Tracking Error');
grid on;

sgtitle('x,y,z Trajectories and Tracking Errors');