function plot_LambdaIBVS(time, erorimage, Tx, Ty, Tz, omegax, omegay, omegaz, camera_x, camera_y, camera_z, camera_wx, camera_wy, camera_wz, scenario_type)
% Function to plot results for Lambda-based IBVS simulation
% Input parameters:
%   time: time vector
%   erorimage: image error vector
%   Tx, Ty, Tz: translational velocity components
%   omegax, omegay, omegaz: angular velocity components
%   camera_x, camera_y, camera_z: camera position components
%   camera_wx, camera_wy, camera_wz: camera orientation components
%   scenario_type: 'single_lambda' or 'sequential_lambda'

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

% Create a new figure for Lambda-IBVS results
figure('Name', ['Lambda-IBVS Results: ', scenario_type], 'NumberTitle', 'off')

% Plot feature errors
subplot(3, 2, 1)
t = time(1:length(erorimage))*0.033;
plot(t, erorimage(1,:))
hold on
plot(t, erorimage(2,:))
plot(t, erorimage(3,:))
plot(t, erorimage(4,:))
plot(t, erorimage(5,:))
plot(t, erorimage(6,:))
plot(t, erorimage(7,:))
plot(t, erorimage(8,:))
xlabel('Time (s)')
ylabel('Feature Error')
title('Feature Errors')
grid on

% Plot translational velocities
subplot(3, 2, 2)
t = time(1:length(Tx))*0.033;
plot(t, Tx)
hold on
plot(t, Ty)
plot(t, Tz)
xlabel('Time (s)')
ylabel('Velocity (m/s)')
title('Translational Velocities')
legend('v_x', 'v_y', 'v_z')
grid on

% Plot angular velocities
subplot(3, 2, 3)
t = time(1:length(omegax))*0.033;
plot(t, omegax)
hold on
plot(t, omegay)
plot(t, omegaz)
xlabel('Time (s)')
ylabel('Angular Velocity (rad/s)')
title('Angular Velocities')
legend('\omega_x', '\omega_y', '\omega_z')
grid on

% Plot camera position
subplot(3, 2, 4)
plot(time*0.033, camera_x)
hold on
plot(time*0.033, camera_y)
plot(time*0.033, camera_z)
xlabel('Time (s)')
ylabel('Position (m)')
title('Camera Position')
legend('x', 'y', 'z')
grid on

% Plot camera orientation
subplot(3, 2, 5)
plot(time*0.033, camera_wx)
hold on
plot(time*0.033, camera_wy)
plot(time*0.033, camera_wz)
xlabel('Time (s)')
ylabel('Orientation (rad)')
title('Camera Orientation')
legend('\phi', '\theta', '\psi')
grid on

% Plot the error norm
subplot(3, 2, 6)
% Calculate the norm of the error at each time step
error_norm = zeros(1, size(erorimage,2));
for i = 1:size(erorimage,2)
    error_norm(i) = norm(erorimage(:,i));
end
plot(time(1:length(error_norm))*0.033, error_norm)
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
        convergence_time = time(i)*0.033;
        break;
    end
end

if isnan(convergence_time)
    subtitle_text = ['Method: Lambda-IBVS (', scenario_type, '), Final error: ', num2str(final_error), ', Did not converge'];
else
    subtitle_text = ['Method: Lambda-IBVS (', scenario_type, '), Final error: ', num2str(final_error), ', Convergence time: ', num2str(convergence_time), 's'];
end

sgtitle(subtitle_text)

% Create a figure with MPC optimization explanation
figure('Name', 'MPC Optimization Explanation', 'NumberTitle', 'off', 'Position', [100, 100, 800, 600]);

% Create text explanation
if strcmp(scenario_type, 'single_lambda')
    explanation_text = {
        '\fontsize{14}\bf Lambda-based IBVS with Single Lambda MPC 可视化说明', ...
        '', ...
        '\fontsize{12}\rm 在单一Lambda MPC中:', ...
        '  • 对一系列Lambda值进行网格搜索', ...
        '  • 对每个Lambda值模拟整个预测时域内的系统行为', ...
        '  • 考虑多个扰动情景以进行最坏情况分析', ...
        '  • 选择使最坏情况成本最小的Lambda值', ...
        '', ...
        '实时可视化窗口显示:', ...
        '  • 左上: Lambda值与对应成本的关系图（红点为最优值）', ...
        '  • 右上: 当前特征误差的大小', ...
        '  • 左下: 历史最优Lambda值的变化趋势', ...
        '  • 右下: 历史最小成本值的变化趋势', ...
        '', ...
        '命令窗口中显示每步的最优Lambda值和对应的最小成本'
    };
else % sequential_lambda
    explanation_text = {
        '\fontsize{14}\bf Lambda-based IBVS with Sequential Lambda MPC 可视化说明', ...
        '', ...
        '\fontsize{12}\rm 在序列Lambda MPC中:', ...
        '  • 对预测时域中的每一步单独优化Lambda值', ...
        '  • 对每一步的一系列候选Lambda值进行网格搜索', ...
        '  • 考虑扰动影响，计算每个Lambda值的步骤成本', ...
        '  • 为预测时域中的每一步选择最优Lambda值', ...
        '', ...
        '实时可视化窗口显示:', ...
        '  • 左上: 预测时域内各步骤的最优Lambda值', ...
        '  • 右上: 第一步的Lambda值与对应成本关系（红点为最优值）', ...
        '  • 左下: 历史上第一步Lambda值的变化趋势', ...
        '  • 右下: 当前特征误差的大小', ...
        '', ...
        '命令窗口中显示每个控制周期的Lambda序列'
    };
end

% Create annotation with explanation
annotation('textbox', [0.1, 0.1, 0.8, 0.8], 'String', explanation_text, ...
    'EdgeColor', 'none', 'HorizontalAlignment', 'left', 'VerticalAlignment', 'top');

end 