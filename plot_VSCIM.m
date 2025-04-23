function plot_VSCIM(time, erorimage, Tx, Ty, Tz, omegax, omegay, omegaz, camera_x, camera_y, camera_z, camera_wx, camera_wy, camera_wz, compensation_norms)
% Function to plot results for VSC-IM (Visual Servoing Control with Input Mapping) simulation
% Input parameters:
%   time: time vector
%   erorimage: image error vector
%   Tx, Ty, Tz: translational velocity components
%   omegax, omegay, omegaz: angular velocity components
%   camera_x, camera_y, camera_z: camera position components
%   camera_wx, camera_wy, camera_wz: camera orientation components
%   compensation_norms: norm of compensation terms over time

% Ensure all vectors are of the same length
n = length(time);
if size(erorimage, 2) ~= n
    erorimage = erorimage(:, 1:n);
end

% Ensure control signals have the same length
control_signals = {Tx, Ty, Tz, omegax, omegay, omegaz};
for i = 1:length(control_signals)
    if length(control_signals{i}) > n
        control_signals{i} = control_signals{i}(1:n);
    elseif length(control_signals{i}) < n
        % Pad with last value if shorter
        control_signals{i}(end+1:n) = control_signals{i}(end);
    end
end
[Tx, Ty, Tz, omegax, omegay, omegaz] = deal(control_signals{:});

% Create figure for VSC-IM results
figure('Name', 'VSC-IM Results', 'NumberTitle', 'off', 'Position', [100 100 1200 800])

% Plot feature errors
subplot(3, 3, 1)
t = time(1:length(erorimage))*0.033;
plot(t, erorimage(1,:), 'LineWidth', 1.5)
hold on
plot(t, erorimage(2,:), 'LineWidth', 1.5)
plot(t, erorimage(3,:), 'LineWidth', 1.5)
plot(t, erorimage(4,:), 'LineWidth', 1.5)
plot(t, erorimage(5,:), 'LineWidth', 1.5)
plot(t, erorimage(6,:), 'LineWidth', 1.5)
plot(t, erorimage(7,:), 'LineWidth', 1.5)
plot(t, erorimage(8,:), 'LineWidth', 1.5)
xlabel('Time (s)')
ylabel('Feature Error')
title('Feature Errors')
grid on

% Plot translational velocities
subplot(3, 3, 2)
t = time(1:length(Tx))*0.033;
plot(t, Tx, 'LineWidth', 1.5)
hold on
plot(t, Ty, 'LineWidth', 1.5)
plot(t, Tz, 'LineWidth', 1.5)
xlabel('Time (s)')
ylabel('Velocity (m/s)')
title('Translational Velocities')
legend('v_x', 'v_y', 'v_z')
grid on

% Plot angular velocities
subplot(3, 3, 3)
t = time(1:length(omegax))*0.033;
plot(t, omegax, 'LineWidth', 1.5)
hold on
plot(t, omegay, 'LineWidth', 1.5)
plot(t, omegaz, 'LineWidth', 1.5)
xlabel('Time (s)')
ylabel('Angular Velocity (rad/s)')
title('Angular Velocities')
legend('\omega_x', '\omega_y', '\omega_z')
grid on

% Plot camera position
subplot(3, 3, 4)
plot(time*0.033, camera_x, 'LineWidth', 1.5)
hold on
plot(time*0.033, camera_y, 'LineWidth', 1.5)
plot(time*0.033, camera_z, 'LineWidth', 1.5)
xlabel('Time (s)')
ylabel('Position (m)')
title('Camera Position')
legend('x', 'y', 'z')
grid on

% Plot camera orientation
subplot(3, 3, 5)
plot(time*0.033, camera_wx, 'LineWidth', 1.5)
hold on
plot(time*0.033, camera_wy, 'LineWidth', 1.5)
plot(time*0.033, camera_wz, 'LineWidth', 1.5)
xlabel('Time (s)')
ylabel('Orientation (rad)')
title('Camera Orientation')
legend('\phi', '\theta', '\psi')
grid on

% Plot the error norm
subplot(3, 3, 6)
error_norm = zeros(1, size(erorimage,2));
for i = 1:size(erorimage,2)
    error_norm(i) = norm(erorimage(:,i));
end
plot(time(1:length(error_norm))*0.033, error_norm, 'LineWidth', 1.5)
xlabel('Time (s)')
ylabel('Error Norm')
title('Feature Error Norm')
grid on

% Plot compensation norm
subplot(3, 3, 7)
plot(time(1:length(compensation_norms))*0.033, compensation_norms, 'r-', 'LineWidth', 1.5)
xlabel('Time (s)')
ylabel('Compensation Norm')
title('Historical Data Compensation Magnitude')
grid on

% Plot error norm in log scale
subplot(3, 3, 8)
semilogy(time(1:length(error_norm))*0.033, error_norm, 'LineWidth', 1.5)
xlabel('Time (s)')
ylabel('log(Error Norm)')
title('Error Convergence (Log Scale)')
grid on

% Plot compensation vs error norm
subplot(3, 3, 9)
plot(error_norm(1:length(compensation_norms)), compensation_norms, 'b.', 'MarkerSize', 2)
xlabel('Error Norm')
ylabel('Compensation Norm')
title('Compensation vs Error')
grid on

% Create a new figure for 3D trajectory
figure('Name', 'VSC-IM 3D Trajectory', 'NumberTitle', 'off')
plot3(camera_x, camera_y, camera_z, 'r-', 'LineWidth', 2)
hold on
plot3(camera_x(1), camera_y(1), camera_z(1), 'bo', 'MarkerSize', 10, 'LineWidth', 2)
plot3(camera_x(end), camera_y(end), camera_z(end), 'go', 'MarkerSize', 10, 'LineWidth', 2)
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')
title('Camera Trajectory')
legend('Path', 'Start', 'End')
grid on
axis equal

end 