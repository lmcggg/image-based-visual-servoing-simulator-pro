function plot_PPIBVS_func(time, erorimage, Tx, Ty, Tz, omegax, omegay, omegaz, camera_x, camera_y, camera_z, camera_wx, camera_wy, camera_wz)
% Function to plot results for PPIBVS simulation
% Input parameters:
%   time: time vector
%   erorimage: image error vector
%   Tx, Ty, Tz: translational velocity components
%   omegax, omegay, omegaz: angular velocity components
%   camera_x, camera_y, camera_z: camera position components
%   camera_wx, camera_wy, camera_wz: camera orientation components

dt = 0.033; % sampling time
ubound = 0.2; % velocity saturation

% Create a new figure for PPIBVS results
figure('Name', 'PPIBVS Results', 'NumberTitle', 'off')

% 获取向量长度
time_length = length(time);
error_length = size(erorimage, 2);

% 确保使用最小的长度以避免索引错误
plot_length = min(time_length-1, error_length-1);

% Plot control commands
subplot(2, 2, 1)
hold on
box on
title('Control commands velocities - PPIBVS')
ylabel('Translational (m/s)')
plot(time(1:plot_length)*dt, Tx(1:plot_length), 'b', 'LineWidth', 1.5)
plot(time(1:plot_length)*dt, Ty(1:plot_length), 'g', 'LineWidth', 1.5)
plot(time(1:plot_length)*dt, Tz(1:plot_length), 'r', 'LineWidth', 1.5)
line([0 time(plot_length)*dt], [ubound ubound], 'Color', 'k', 'LineStyle', '-.', 'LineWidth', 1)
line([0 time(plot_length)*dt], [-ubound -ubound], 'Color', 'k', 'LineStyle', '-.', 'LineWidth', 1)
legend('T_x', 'T_y', 'T_z')
grid on

subplot(2, 2, 3)
hold on
box on
ylabel('Rotational (rad/s)')
plot(time(1:plot_length)*dt, omegax(1:plot_length), 'c', 'LineWidth', 1.5)
plot(time(1:plot_length)*dt, omegay(1:plot_length), 'm', 'LineWidth', 1.5)
plot(time(1:plot_length)*dt, omegaz(1:plot_length), 'k', 'LineWidth', 1.5)
xlabel('Time (s)')
legend('\omega_x', '\omega_y', '\omega_z')
line([0 time(plot_length)*dt], [ubound ubound], 'Color', 'k', 'LineStyle', '-.', 'LineWidth', 1)
line([0 time(plot_length)*dt], [-ubound -ubound], 'Color', 'k', 'LineStyle', '-.', 'LineWidth', 1)
grid on

% Plot feature errors
subplot(2, 2, 2)
hold on
grid on
box on
xlabel('Time (s)')
ylabel('Feature Error')
title('Feature Errors')
plot(time(1:plot_length)*dt, erorimage(1,1:plot_length), 'r', 'LineWidth', 1.5)
plot(time(1:plot_length)*dt, erorimage(2,1:plot_length), 'r', 'LineStyle', '-.', 'LineWidth', 1.5)
plot(time(1:plot_length)*dt, erorimage(3,1:plot_length), 'c', 'LineWidth', 1.5)
plot(time(1:plot_length)*dt, erorimage(4,1:plot_length), 'c', 'LineStyle', '-.', 'LineWidth', 1.5)
plot(time(1:plot_length)*dt, erorimage(5,1:plot_length), 'b', 'LineWidth', 1.5)
plot(time(1:plot_length)*dt, erorimage(6,1:plot_length), 'b', 'LineStyle', '-.', 'LineWidth', 1.5)
plot(time(1:plot_length)*dt, erorimage(7,1:plot_length), 'm', 'LineWidth', 1.5)
plot(time(1:plot_length)*dt, erorimage(8,1:plot_length), 'm', 'LineStyle', '-.', 'LineWidth', 1.5)
legend('e u_1', 'e v_1', 'e u_2', 'e v_2', 'e u_3', 'e v_3', 'e u_4', 'e v_4', 'Location', 'eastoutside')

% Plot the error norm
subplot(2, 2, 4)
% Calculate the norm of the error at each time step
error_norm = zeros(1, plot_length);
for i = 1:plot_length
    error_norm(i) = norm(erorimage(:,i));
end
plot(time(1:plot_length)*dt, error_norm, 'LineWidth', 1.5)
xlabel('Time (s)')
ylabel('Error Norm')
title('Feature Error Norm')
grid on

% Display metrics in title
final_error = error_norm(end);
convergence_time = NaN;
% Find convergence time (when error norm falls below 5% of initial)
threshold = 0.05 * error_norm(1);
for i = 1:length(error_norm)
    if error_norm(i) < threshold
        convergence_time = time(i)*dt;
        break;
    end
end

if isnan(convergence_time)
    subtitle_text = ['Method: PPIBVS, Final error: ', num2str(final_error), ', Did not converge'];
else
    subtitle_text = ['Method: PPIBVS, Final error: ', num2str(final_error), ', Convergence time: ', num2str(convergence_time), 's'];
end

sgtitle(subtitle_text)

% Create another figure for camera trajectory
figure('Name', 'PPIBVS Camera Trajectory', 'NumberTitle', 'off')
hold on
grid on
plot3(camera_x, camera_y, camera_z, 'r-', 'LineWidth', 1.5)
scatter3(camera_x(1), camera_y(1), camera_z(1), 100, 'b', 'filled')
scatter3(camera_x(end), camera_y(end), camera_z(end), 100, 'g', 'filled')
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')
title('Camera Trajectory')
legend('Trajectory', 'Start', 'End')
view(45, 30)
hold off

end 