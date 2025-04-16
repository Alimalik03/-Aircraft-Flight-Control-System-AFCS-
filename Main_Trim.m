function [F] = Main_Trim(X_trim)
% TRIM_V - Computes residuals for steady flight trimming
% Input: X_trim = [phi, theta, alpha, delta_e, delta_a, delta_r, delta_T]
% Output: F - Residual vector to be minimized by optimizer

% === Constants ===
g = 9.80665; 
rho = 1.2256;
Mass = 4500;
C_b = 2.5; 
b = 21;
S = 45;
I_xx = 100000;
I_yy = 230000;
I_zz = 280000; I_xz = 23000;
I_yz = 0; I_xy = 0;

% === Aerodynamic Coefficients ===
C_L_0 = 0.19; C_L_alpha = 6.1879; C_L_q = 4.66; C_L_delta_e = 0.4727; C_L_alpha_dot = 0;
C_D_0 = 0.0230; C_D_alpha = 0.1318; C_D_delta_e = 0.0178;
C_Y_beta = -1.1803; C_Y_p = 0.3; C_Y_r = 0.91; C_Y_delta_r = 0.4016; C_Y_delta_a = 0;
C_l_beta = -0.1352; C_l_p = -0.45; C_l_r = 0.1; C_l_delta_a = -0.59; C_l_delta_r = 0.0289;
C_M_0 = 0.0133; C_M_alpha = -2.0168; C_M_alpha_dot = -6.76; C_M_delta_e = -1.6788; C_M_q = -36.3;
C_N_beta = 0.1209; C_N_p = -0.114; C_N_r = -0.3; C_N_delta_a = -0.00063; C_N_delta_r = -0.102;

% === Trim Conditions ===
turn_rate = 0 / 57.3;  % rad/s
gamma = 0;%5 / 57.3;       % flight path angle in rad
V = 100;                % flight speed in m/s
Qs = 0.5 * rho * V^2;
T_max = 68000;
n_v = 0; 
n_rho = 0.75;

% === Input Variables ===
phi = X_trim(1); 
theta = X_trim(2); 
alpha = X_trim(3);
delta_e = X_trim(4);
delta_a = X_trim(5);
delta_r = X_trim(6);
delta_T = X_trim(7);
beta = 0;

% === Derived Velocities ===
u = V * cos(alpha) * cos(beta);
v = V * sin(beta);
w = V * sin(alpha) * cos(beta);

% === Angular Rates for Coordinated Turn ===
p = -turn_rate * sin(theta);
q = turn_rate * sin(phi) * cos(theta);
r = turn_rate * cos(phi) * cos(theta);

% === State and Control Vectors ===
X = [0 0 -1000 phi theta 0 u v w V alpha beta p q r]'; % state vector
U = [delta_e; delta_a; delta_r; delta_T]; %control vectors

% === Model Parameters Vector ===
M = [g, rho, Mass, C_b, b, S, ...
     I_xx, I_yy, I_zz, I_xz, I_yz, I_xy, ...
     C_L_0, C_L_alpha, C_L_q, C_L_delta_e, C_L_alpha_dot, ...
     C_D_0, C_D_alpha, C_D_delta_e, ...
     C_Y_beta, C_Y_p, C_Y_r, C_Y_delta_r, C_Y_delta_a, ...
     C_l_beta, C_l_p, C_l_r, C_l_delta_a, C_l_delta_r, ...
     C_M_0, C_M_alpha, C_M_alpha_dot, C_M_delta_e, C_M_q, ...
     C_N_beta, C_N_p, C_N_r, C_N_delta_a, C_N_delta_r, ...
     V, Qs, T_max, n_v, n_rho, turn_rate, gamma]';

% === Evaluate Model and Extract Steady-Flight Residuals ===
X_dot = SixDof_eqm(X, M, U);

F = X_dot([3,4,5,6,7,8,9,10,11,12,13,14,15]); % Trimmed residuals
end
