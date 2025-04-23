function [control] = LambdaIBVS(dt, qd, q, control, ubound, wbound, lamda, z, umax, vmax, K, t, alpha, scenario_type)
% Lambda-based IBVS control method using Broyden update for Jacobian estimation
% Implementation based on the paper: "Lambda-based IBVS with Broyden online Jacobian
% estimation and Min-Max NMPC for AUV control"
%
% Inputs:
%   dt: sampling time (Tc in the paper)
%   qd: desired feature points (s* in the paper)
%   q: current feature points (s_k in the paper)
%   control: previous control input (v_k-1 in the paper)
%   ubound: velocity saturation for translation
%   wbound: velocity saturation for rotation
%   lamda: camera intrinsic parameter (focal length f in the paper)
%   z: depth of feature points (Z in the paper)
%   umax: max velocity in u axis
%   vmax: max velocity in v axis
%   K: control gain
%   t: current time step (k in the paper)
%   alpha: Broyden update parameter (0 < alpha < 2)
%   scenario_type: 'single_lambda' or 'sequential_lambda'

% Initialize static variables for Jacobian estimation
persistent J_hat;  % Estimated Jacobian matrix (L_hat in the paper)
persistent prev_q; % Previous feature points (s_{k-1} in the paper)
persistent prev_control; % Previous control input (v_{k-1} in the paper)
persistent fig_mpc; % Figure handle for MPC visualization
persistent lambda_vals_history; % History of lambda values for plotting
persistent cost_vals_history;   % History of cost values for plotting

% Initialize Lambda-NMPC parameters according to the paper
lambda_max = 5.0;  % Maximum lambda value
N = 5; % Prediction horizon length
N_d = 5; % Number of disturbance scenarios for Min-Max optimization

% Initialize Jacobian matrix if first call
if isempty(J_hat) || t == 0
    % Calculate Jacobian for each feature point according to the paper formula
    % L(s,Z) = [-f/Z, 0, m/Z, mn/f, -(f^2+m^2)/f, n;
    %           0, -f/Z, n/Z, (f^2+n^2)/f, -mn/f, -m]
    
    % Point 1
    J1 = [ -lamda/z(1,1), 0, q(1,1)/z(1,1), (q(1,1)*q(2,1))/lamda, -(lamda^2+q(1,1)^2)/lamda, q(2,1);
           0, -lamda/z(1,1), q(2,1)/z(1,1), (lamda^2+q(2,1)^2)/lamda, -(q(1,1)*q(2,1))/lamda, -q(1,1)];
     
    % Point 2
    J2 = [ -lamda/z(2,1), 0, q(3,1)/z(2,1), (q(3,1)*q(4,1))/lamda, -(lamda^2+q(3,1)^2)/lamda, q(4,1);
           0, -lamda/z(2,1), q(4,1)/z(2,1), (lamda^2+q(4,1)^2)/lamda, -(q(3,1)*q(4,1))/lamda, -q(3,1)];

    % Point 3
    J3 = [ -lamda/z(3,1), 0, q(5,1)/z(3,1), (q(5,1)*q(6,1))/lamda, -(lamda^2+q(5,1)^2)/lamda, q(6,1);
           0, -lamda/z(3,1), q(6,1)/z(3,1), (lamda^2+q(6,1)^2)/lamda, -(q(5,1)*q(6,1))/lamda, -q(5,1)];
     
    % Point 4
    J4 = [ -lamda/z(4,1), 0, q(7,1)/z(4,1), (q(7,1)*q(8,1))/lamda, -(lamda^2+q(7,1)^2)/lamda, q(8,1);
           0, -lamda/z(4,1), q(8,1)/z(4,1), (lamda^2+q(8,1)^2)/lamda, -(q(7,1)*q(8,1))/lamda, -q(7,1)];
    
    % Stack Jacobians for all points as per paper
    J_hat = [J1; J2; J3; J4];
    prev_q = q;
    prev_control = zeros(6,1);
    
    % Initialize visualization
    if isempty(fig_mpc) || ~ishandle(fig_mpc)
        fig_mpc = figure('Name', 'MPC Optimization Visualization', 'NumberTitle', 'off');
    end
    
    % Initialize history arrays
    lambda_vals_history = {};
    cost_vals_history = {};
else
    % Update Jacobian using Broyden method as per the paper
    if t > 1
        % Calculate Delta_s_k = s_k - s_{k-1}
        Delta_s = q - prev_q;
        
        % Calculate Delta_eta_k = T_c * v_{k-1}
        Delta_eta = dt * prev_control;
        
        % Broyden update formula: L_{k+1} = L_k + alpha * ((Delta_s_k - L_k*Delta_eta_k)*Delta_eta_k^T)/(Delta_eta_k^T*Delta_eta_k)
        if norm(Delta_eta) > 1e-6 % Avoid division by zero
            J_hat = J_hat + alpha * ((Delta_s - J_hat * Delta_eta) * Delta_eta') / (Delta_eta' * Delta_eta);
        end
    end
end

% Store current values for next iteration
prev_q = q;
prev_control = control;

% Calculate feature error as per paper: e_k = s_k - s*
e = q - qd;

% Get pseudoinverse of Jacobian: L_k^+ = (L_k^T*L_k)^{-1}*L_k^T
J_hat_pinv = pinv(J_hat);

% Define cost function weight matrices as per paper's objective function
Q = 100000*eye(8); % Feature error weight matrix
R = 0.1 * eye(6); % Control input weight matrix
P = 100 * eye(8); % Terminal cost weight matrix (higher weight for final state error)

% Define bounds for disturbance based on Jacobian uncertainty
% w_k = T_c * Delta_L_k * v_k where w_min <= w_k <= w_max
w_min = -0.01 * ones(8, 1);
w_max = 0.01 * ones(8, 1);

% Implement control strategy based on scenario type
if strcmp(scenario_type, 'single_lambda')
    % ===== Single Lambda-Based NMPC (Section 4 of paper) =====
    % Using grid search instead of fminimax due to MATLAB limitations
    
    % Setup lambda search range for grid search
    lambda_values = linspace(0.1, lambda_max, 20);
    min_cost = inf;
    best_lambda = 1.0; % Default value
    
    % Arrays to store optimization data for visualization
    all_lambda_vals = zeros(size(lambda_values));
    all_cost_vals = zeros(size(lambda_values));
    
    % Min-Max optimization using grid search
    for i = 1:length(lambda_values)
        lambda_val = lambda_values(i);
        max_scenario_cost = 0;
        
        % Consider multiple disturbance scenarios for worst-case analysis
        for d = 1:N_d
            % Generate disturbance
            w = w_min + rand(8, 1) .* (w_max - w_min);
            
            % Simulate system for this disturbance scenario
            e_sim = e;
            cost = 0;
            
            % Simulate over prediction horizon
            for k = 0:N-1
                % Control input
                v_k = -lambda_val * J_hat_pinv * e_sim;
                
                % Add cost for this step
                cost = cost + e_sim' * Q * e_sim + v_k' * R * v_k;
                
                % State update (if not at the end of horizon)
                if k < N-1
                    e_sim = e_sim + dt * J_hat * v_k + w;
                end
            end
            
            % Add terminal cost
            cost = cost + e_sim' * P * e_sim;
            
            % Keep track of worst-case cost
            max_scenario_cost = max(max_scenario_cost, cost);
        end
        
        % Store results for visualization
        all_lambda_vals(i) = lambda_val;
        all_cost_vals(i) = max_scenario_cost;
        
        % Find lambda that minimizes the worst-case cost
        if max_scenario_cost < min_cost
            min_cost = max_scenario_cost;
            best_lambda = lambda_val;
        end
    end
    
    % Store history for trends
    if isempty(lambda_vals_history)
        lambda_vals_history = {all_lambda_vals};
        cost_vals_history = {all_cost_vals};
    else
        % 限制历史长度，防止内存溢出
        if length(lambda_vals_history) >= 100
            lambda_vals_history = lambda_vals_history(2:end);
            cost_vals_history = cost_vals_history(2:end);
        end
        lambda_vals_history{end+1} = all_lambda_vals;
        cost_vals_history{end+1} = all_cost_vals;
    end
    
    % Visualization of the optimization process
    figure(fig_mpc);
    clf;
    
    % Plot cost vs lambda
    subplot(2, 2, 1);
    plot(all_lambda_vals, all_cost_vals, 'b-o');
    hold on;
    plot(best_lambda, min_cost, 'ro', 'MarkerSize', 10, 'LineWidth', 2);
    xlabel('Lambda Value');
    ylabel('Worst-Case Cost');
    title(sprintf('Single Lambda MPC Optimization (Step %d)', t));
    grid on;
    
    % Add a text box with key information
    annotation('textbox', [0.15, 0.85, 0.3, 0.1], 'String', ...
        {sprintf('Best lambda: %.4f', best_lambda), ...
         sprintf('Min cost: %.2e', min_cost)}, ...
        'EdgeColor', 'none', 'BackgroundColor', [0.9, 0.9, 0.9]);
    
    % Plot error norm
    subplot(2, 2, 2);
    bar(1:8, abs(e), 'b');
    xlabel('Feature Component');
    ylabel('Error Magnitude');
    title('Current Feature Error');
    grid on;
    
    % Plot lambda history if we have enough data
    subplot(2, 2, 3);
    if length(lambda_vals_history) > 1
        % Extract best lambda from each iteration
        best_lambdas = zeros(1, length(lambda_vals_history));
        for i = 1:length(lambda_vals_history)
            if ~isempty(cost_vals_history{i})
                [~, idx] = min(cost_vals_history{i});
                if ~isempty(idx) && idx > 0 && idx <= length(lambda_vals_history{i})
                    best_lambdas(i) = lambda_vals_history{i}(idx);
                else
                    best_lambdas(i) = NaN;
                end
            else
                best_lambdas(i) = NaN;
            end
        end
        
        % 移除NaN值
        valid_indices = ~isnan(best_lambdas);
        best_lambdas = best_lambdas(valid_indices);
        
        if ~isempty(best_lambdas)
            % 修复向量长度不匹配问题
            time_steps = length(best_lambdas);
            x_values = t-time_steps+1:t;
            if x_values(1) < 1
                % 如果起始时间小于1，调整时间和lambda历史以保持一致
                valid_indices = (x_values >= 1);
                x_values = x_values(valid_indices);
                best_lambdas = best_lambdas(valid_indices);
            end
            
            if ~isempty(x_values) && ~isempty(best_lambdas) && length(x_values) == length(best_lambdas)
                plot(x_values, best_lambdas, 'b-o');
            end
        end
        xlabel('Time Step');
        ylabel('Best Lambda');
        title('Lambda History');
        grid on;
        
        % Plot minimum costs history
        subplot(2, 2, 4);
        min_costs = zeros(1, length(cost_vals_history));
        for i = 1:length(cost_vals_history)
            if ~isempty(cost_vals_history{i})
                min_costs(i) = min(cost_vals_history{i});
            else
                min_costs(i) = NaN;
            end
        end
        
        % 移除NaN值
        valid_indices = ~isnan(min_costs);
        min_costs = min_costs(valid_indices);
        
        if ~isempty(min_costs)
            % 同样修复成本历史的向量长度问题
            time_steps = length(min_costs);
            x_values = t-time_steps+1:t;
            if x_values(1) < 1
                valid_indices = (x_values >= 1);
                x_values = x_values(valid_indices);
                min_costs = min_costs(valid_indices);
            end
            
            if ~isempty(x_values) && ~isempty(min_costs) && length(x_values) == length(min_costs)
                semilogy(x_values, min_costs, 'r-o');
            end
        end
        xlabel('Time Step');
        ylabel('Min Cost (log scale)');
        title('Cost History');
        grid on;
    end
    
    % Display information in command window
    fprintf('Step %d - Single Lambda MPC: Best lambda = %.4f, Min cost = %.2e\n', t, best_lambda, min_cost);
    
    % Apply the optimal lambda
    control = -best_lambda * J_hat_pinv * e;
    
elseif strcmp(scenario_type, 'sequential_lambda')
    % ===== Sequential Lambda-Based NMPC (Section 4 of paper) =====
    % Using grid search for each lambda in the sequence
    
    % Initialize sequence of lambda values
    lambda_seq = zeros(N, 1);
    all_costs = cell(N, 1);
    all_lambdas = cell(N, 1);
    
    % For each step in the horizon, find optimal lambda
    for horizon_step = 1:N
        min_step_cost = inf;
        best_step_lambda = 1.0;
        
        % Search range for this step
        step_lambda_values = linspace(0.1, lambda_max, 10);
        step_costs = zeros(size(step_lambda_values));
        
        for i = 1:length(step_lambda_values)
            lambda_val = step_lambda_values(i);
            max_scenario_cost = 0;
            
            % Generate random disturbance
            w = w_min + rand(8, 1) .* (w_max - w_min);
            
            % Predict one step ahead with this lambda
            v_k = -lambda_val * J_hat_pinv * e;
            e_pred = e + dt * J_hat * v_k + w;
            
            % Cost for this step
            step_cost = e' * Q * e + v_k' * R * v_k;
            if horizon_step == N
                step_cost = step_cost + e_pred' * P * e_pred; % Add terminal cost for last step
            else
                step_cost = step_cost + e_pred' * Q * e_pred; % Add next state cost
            end
            
            % Keep track of worst case
            max_scenario_cost = max(max_scenario_cost, step_cost);
            step_costs(i) = max_scenario_cost;
            
            if max_scenario_cost < min_step_cost
                min_step_cost = max_scenario_cost;
                best_step_lambda = lambda_val;
            end
        end
        
        lambda_seq(horizon_step) = best_step_lambda;
        all_costs{horizon_step} = step_costs;
        all_lambdas{horizon_step} = step_lambda_values;
    end
    
    % Store best lambda sequence for history
    if isempty(lambda_vals_history)
        lambda_vals_history = {lambda_seq};
    else
        % 限制历史长度，防止内存溢出
        if length(lambda_vals_history) >= 100
            lambda_vals_history = lambda_vals_history(2:end);
        end
        lambda_vals_history{end+1} = lambda_seq;
    end
    
    % Visualization of the sequential optimization process
    figure(fig_mpc);
    clf;
    
    % Plot lambda values for each horizon step
    subplot(2, 2, 1);
    bar(1:N, lambda_seq);
    xlabel('Horizon Step');
    ylabel('Lambda Value');
    title(sprintf('Sequential Lambda MPC (Step %d)', t));
    grid on;
    
    % Plot costs vs lambda for first horizon step
    subplot(2, 2, 2);
    if ~isempty(all_lambdas) && length(all_lambdas) >= 1 && ~isempty(all_costs) && length(all_costs) >= 1
        if ~isempty(all_lambdas{1}) && ~isempty(all_costs{1}) && length(all_lambdas{1}) == length(all_costs{1})
            plot(all_lambdas{1}, all_costs{1}, 'b-o');
            hold on;
            [min_cost_val, min_idx] = min(all_costs{1});
            if ~isempty(min_idx) && min_idx > 0 && min_idx <= length(all_lambdas{1})
                best_lambda_val = all_lambdas{1}(min_idx);
                plot(best_lambda_val, min_cost_val, 'ro', 'MarkerSize', 10, 'LineWidth', 2);
            end
        end
    end
    xlabel('Lambda Value (First Step)');
    ylabel('Cost');
    title('Cost vs Lambda (First Step)');
    grid on;
    
    % Plot lambda sequences history
    subplot(2, 2, 3);
    if length(lambda_vals_history) > 1
        % Plot lambda history for the first step only
        first_step_lambdas = zeros(1, length(lambda_vals_history));
        for i = 1:length(lambda_vals_history)
            if ~isempty(lambda_vals_history{i}) && length(lambda_vals_history{i}) >= 1
                first_step_lambdas(i) = lambda_vals_history{i}(1);
            else
                first_step_lambdas(i) = NaN; % 设置无效值为NaN
            end
        end
        
        % 移除NaN值
        valid_indices = ~isnan(first_step_lambdas);
        first_step_lambdas = first_step_lambdas(valid_indices);
        
        if ~isempty(first_step_lambdas)
            % 修复向量长度不匹配问题
            time_steps = length(first_step_lambdas);
            x_values = t-time_steps+1:t;
            if x_values(1) < 1
                % 如果起始时间小于1，调整时间和lambda历史以保持一致
                valid_indices = (x_values >= 1);
                x_values = x_values(valid_indices);
                first_step_lambdas = first_step_lambdas(valid_indices);
            end
            
            if ~isempty(x_values) && ~isempty(first_step_lambdas) && length(x_values) == length(first_step_lambdas)
                plot(x_values, first_step_lambdas, 'b-o');
            end
        end
        xlabel('Time Step');
        ylabel('First Lambda');
        title('First Lambda History');
        grid on;
    end
    
    % Plot error norm
    subplot(2, 2, 4);
    bar(1:8, abs(e), 'b');
    xlabel('Feature Component');
    ylabel('Error Magnitude');
    title('Current Feature Error');
    grid on;
    
    % Display information in command window
    lambda_str = sprintf('%.4f ', lambda_seq);
    fprintf('Step %d - Sequential Lambda MPC: Lambdas = [%s]\n', t, lambda_str);
    
    % Apply first lambda from the sequence
    control = -lambda_seq(1) * J_hat_pinv * e;
else
    % Default to standard IBVS with direction consistent with Lambda IBVS
    control = -K * J_hat_pinv * e;
    fprintf('Step %d - Standard IBVS: Lambda = %.4f\n', t, K);
end

% Apply velocity constraints
for ii = 1
    % Translational velocity constraints
    for n = 1:3
        if control(n,ii) >= ubound
            control(n,ii) = ubound;
        elseif control(n,ii) <= -ubound
            control(n,ii) = -ubound;
        end
    end
    
    % Angular velocity constraints
    for n = 4:6
        if control(n,ii) >= wbound
            control(n,ii) = wbound;
        elseif control(n,ii) <= -wbound
            control(n,ii) = -wbound;
        end
    end
end

% Force a small pause to allow visualization to be seen
drawnow;
pause(0.05);

end 