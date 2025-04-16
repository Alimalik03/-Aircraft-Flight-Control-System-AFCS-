clear; clc;

% === Initial Guess for Trim Variables ===
% Format: [phi, theta, alpha, delta_e, delta_a, delta_r, delta_T]
X0 = [0, 0.05, 0.05, 0, 0, 0, 0.5];

% === Optimization Options ===
options = optimoptions('fsolve', 'Display', 'iter', 'MaxFunctionEvaluations', 5000, 'MaxIterations', 1000);

% === Call fsolve for Trim ===
X_trimmed = fsolve(@Main_Trim, X0, options);

disp(X_trimmed)

% === Display Trimmed Output ===
disp('Trimmed Conditions:');
disp(['phi: ', num2str(X_trimmed(1) * 180/pi), ' deg']);
disp(['theta: ', num2str(X_trimmed(2) * 180/pi), ' deg']);
disp(['alpha: ', num2str(X_trimmed(3) * 180/pi), ' deg']);
disp(['delta_e: ', num2str(X_trimmed(4) * 180/pi), ' deg']);
disp(['delta_a: ', num2str(X_trimmed(5) * 180/pi), ' deg']);
disp(['delta_r: ', num2str(X_trimmed(6) * 180/pi), ' deg']);
disp(['delta_T: ', num2str(X_trimmed(7))]);
