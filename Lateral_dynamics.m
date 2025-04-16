
% Lateral Dynamics Simulation Script

clear; clc; close all;

%% Step 1: Define Trim State and Control
V = 100; 
alpha = -0.0018; 
theta = -0.0018;
beta = 0; 
phi = 0;
u = V * cos(alpha) * cos(beta);
v = V * sin(beta);
w = V * sin(alpha) * cos(beta);

X_trim = [0; 0; -1000; phi; theta; 0; u; v; w; V; alpha; beta; 0; 0; 0];

delta_e = -0.0383;
delta_a = 0;
delta_r = 0;
delta_T = -0.0995;

U_trim = [delta_e; delta_a; delta_r; delta_T];


%% Step 2: Define Parameters Directly
g = 9.80665; rho = 1.2256; Mass = 4500;
C_b = 2.5; b = 21; S = 45;
I_xx = 100000; I_yy = 230000; I_zz = 280000; I_xz = 23000; I_yz = 0; I_xy = 0;
T_max = 68000; n_v = 0; n_rho = 0.75;

C_L = [0.19, 6.1879, 4.66, 0.4727, 1.93];
C_D = [0.023, 0.1318, 0.0178];
C_Y = [-1.1803, 0.3, 0.91, 0.4016, 0];
C_l = [-0.1352, -0.45, 0.1, -0.59, 0.0289];
C_M = [0.0133, -2.0168, -6.76, -1.6788, -36.3];
C_N = [0.1209, -0.114, -0.3, -0.00063, -0.102];

Qs = 0.5 * rho * V^2;
M = [g, rho, Mass, C_b, b, S, I_xx, I_yy, I_zz, I_xz, I_yz, I_xy, ...
     C_L, C_D, C_Y, C_l, C_M, C_N, V, Qs, T_max, n_v, n_rho, 0, 0]';

%% Step 3: Linearize the Model
[A, B, C, D, sys_full] = linearization(X_trim, U_trim, M);

% Extract lateral-directional states: phi, beta, p, r
lat_states = [4, 12, 13, 15];
A_lat = A(lat_states, lat_states);
B_lat = B(lat_states, [2, 3]); % Aileron, Rudder
C_lat = eye(4);
D_lat = zeros(4, 2);
sys_lat = ss(A_lat, B_lat, C_lat, D_lat);

%% Step 4: Simulate Step Response to Aileron Input
t = 0:0.1:60;
delta_a_input = deg2rad(5); % 5 deg step
delta_r_input = 0;          % No rudder input
u = [delta_a_input * ones(length(t), 1), delta_r_input * ones(length(t), 1)];

[y, t_out, x] = lsim(sys_lat, u, t);

%% Step 5: Plot Lateral States
labels = {'Roll Angle \phi (deg)', 'Side-Slip Angle \beta (deg)', ...
          'Roll Rate p (deg/s)', 'Yaw Rate r (deg/s)'};

figure('Name', 'Lateral Step Response');
for i = 1:4
    subplot(2,2,i);
    if i <= 2
        plot(t, rad2deg(y(:,i)), 'LineWidth', 1.5);
    else
        plot(t, y(:,i) * 180/pi, 'LineWidth', 1.5);
    end
    grid on;
    xlabel('Time (s)');
    ylabel(labels{i});
    title(labels{i});
end
