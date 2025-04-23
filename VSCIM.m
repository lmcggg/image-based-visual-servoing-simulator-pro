function [control, compensation_norm] = VSCIM(dt, qd, q, control, ubound, wbound, lamda, z, umax, vmax, K, t, eta)
% Visual Servoing Control with Input Mapping (VSC-IM) method
%
% Inputs:
%   dt: sampling time (T in the paper)
%   qd: desired feature points (M* in the paper)
%   q: current feature points (M(k) in the paper)
%   control: previous control input (V_c(k-1) in the paper)
%   ubound: velocity saturation for translation
%   wbound: velocity saturation for rotation
%   lamda: camera intrinsic parameter (focal length)
%   z: depth of feature points
%   umax: max velocity in u axis
%   vmax: max velocity in v axis
%   K: control gain (F in the paper)
%   t: current time step
%   eta: learning rate for input mapping (0 < eta < 1)
%
% Outputs:
%   control: computed control input
%   compensation_norm: norm of the compensation term (for visualization)

% Initialize static variables
persistent prev_errors;    % Store previous error values
persistent prev_controls;  % Store previous control inputs
persistent l;             % Window length for historical data
persistent J_perturbed;   % Store perturbed initial Jacobian

% Initialize on first call
if isempty(prev_errors) || t == 0
    % Define window length for historical data
    l = 5;  % Use past 5 steps
    
    % Initialize storage arrays
    prev_errors = zeros(8, l);
    prev_controls = zeros(6, l);
    
    % Initialize perturbed Jacobian flag
    J_perturbed = false;
end

% Calculate current error: e(k) = M(k) - M*
e = q - qd;

% Calculate standard Jacobian for each feature point
J1 = [ -lamda/z(1,1),      0,        q(1,1)/z(1,1),    (q(1,1)*q(2,1))/lamda,    -(lamda^2+q(1,1)^2)/lamda,  q(2,1);
         0,       -lamda/z(1,1),    q(2,1)/z(1,1),   (lamda^2+q(2,1)^2)/lamda,    -(q(1,1)*q(2,1))/lamda,    -q(1,1)];
     
J2 = [ -lamda/z(2,1),      0,        q(3,1)/z(2,1),    (q(3,1)*q(4,1))/lamda,    -(lamda^2+q(3,1)^2)/lamda,  q(4,1);
         0,       -lamda/z(2,1),    q(4,1)/z(2,1),   (lamda^2+q(4,1)^2)/lamda,    -(q(3,1)*q(4,1))/lamda,    -q(3,1)];

J3 = [ -lamda/z(3,1),      0,        q(5,1)/z(3,1),    (q(5,1)*q(6,1))/lamda,    -(lamda^2+q(5,1)^2)/lamda,  q(6,1);
         0,       -lamda/z(3,1),    q(6,1)/z(3,1),   (lamda^2+q(6,1)^2)/lamda,    -(q(5,1)*q(6,1))/lamda,    -q(5,1)];
     
J4 = [ -lamda/z(4,1),      0,        q(7,1)/z(4,1),    (q(7,1)*q(8,1))/lamda,    -(lamda^2+q(7,1)^2)/lamda,  q(8,1);
         0,       -lamda/z(4,1),    q(8,1)/z(4,1),   (lamda^2+q(8,1)^2)/lamda,    -(q(7,1)*q(8,1))/lamda,    -q(7,1)];

% Stack Jacobians for all points
J = [J1; J2; J3; J4];

% Add perturbation to initial Jacobian (only once)
if ~J_perturbed
    % Add 10% random perturbation to the Jacobian
    perturbation = 0.1 * randn(size(J));
    J = J .* (1 + perturbation);
    J_perturbed = true;
    disp('Initial Jacobian perturbed by 10%');
end

% Initialize compensation term
compensation_norm = 0;

% If t > 0, we have historical data to use for compensation
if t > l
    % 1. Construct the historical data matrices (Equation 15)
    Delta_vec_e = zeros(8, l);
    vec_V_c = zeros(6, l);
    
    for i = 1:l
        Delta_vec_e(:, i) = prev_errors(:, i) - prev_errors(:, mod(i, l) + 1);
        vec_V_c(:, i) = prev_controls(:, i);
    end
    
    % 2. Compute the lambda coefficient (Equation 22)
    Q = eye(8);
    
    % Calculate lambda* using pseudoinverse
    epsilon = 1e-6;
    mat = Delta_vec_e' * Q * Delta_vec_e;
    
    if rcond(mat) < epsilon
        lambda_star = zeros(l, 1);
    else
        lambda_star = -eta * pinv(Delta_vec_e' * Q * Delta_vec_e) * Delta_vec_e' * Q * e;
    end
    
    % 3. Compute the modified error term
    e_hat = e + Delta_vec_e * lambda_star;
    
    % 4. Compute the control law with compensation
    control_standard = -K * pinv(J) * e;
    control_compensation = vec_V_c * lambda_star;
    control = -K * pinv(J) * e_hat + control_compensation;
    
    % Calculate compensation norm for visualization
    compensation_norm = norm(control_compensation);
    
    % For visualization and debugging
    if t <= 10 || mod(t, 20) == 0
        disp(['VSC-IM Step ', num2str(t)]);
        disp(['  Compensation norm: ', num2str(compensation_norm)]);
        disp(['  Lambda* norm: ', num2str(norm(lambda_star))]);
    end
else
    % When insufficient historical data, use conventional IBVS
    control = -K * pinv(J) * e;
    compensation_norm = 0;
end

% Apply velocity constraints
% Translate constraints
for n = 1:3
    if control(n) >= ubound
        control(n) = ubound;
    elseif control(n) <= -ubound
        control(n) = -ubound;
    end
end

% Rotate constraints
for n = 4:6
    if control(n) >= wbound
        control(n) = wbound;
    elseif control(n) <= -wbound
        control(n) = -wbound;
    end
end

% Update historical data storage
prev_errors = [e, prev_errors(:, 1:l-1)];
prev_controls = [control, prev_controls(:, 1:l-1)];

end 