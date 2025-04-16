clear all;
close all;
clc;

%% ============================== CONSTANTS ==============================
g     = 9.80665;         % gravity (m/s^2)
rho   = 1.2256;          % air density (kg/m^3)
Mass  = 4500;            % aircraft mass (kg)

%% ============================== GEOMETRY ==============================
C_b   = 2.5;             % mean aerodynamic chord (m)
b     = 21;              % wingspan (m)
S     = 45;              % wing area (m^2)

%% ============================== INERTIA ==============================
I_xx = 100000; I_yy = 230000; I_zz = 280000;
I_xz = 23000;  I_yz = 0;       I_xy = 0;

%% ============================== AERO COEFFICIENTS ==============================
% Lift
C_L_0 = 0.19;  
C_L_alpha = 6.1879; 
C_L_q = 4.66;
C_L_delta_e = 0.4727;
C_L_alpha_dot = 1.93;

% Drag
C_D_0 = 0.023;
C_D_alpha = 0.1318;
C_D_delta_e = 0.0178;

% Side-force
C_Y_beta = -1.1803;
C_Y_p = 0.3;
C_Y_r = 0.91;
C_Y_delta_r = 0.4016;
C_Y_delta_a = 0;

% Roll moment
C_l_beta = -0.1352;
C_l_p = -0.45;
C_l_r = 0.1;
C_l_delta_a = -0.59;
C_l_delta_r = 0.0289;

% Pitch moment
C_M_0 = 0.0133;
C_M_alpha = -2.0168;
C_M_alpha_dot = -6.76;
C_M_delta_e = -1.6788;
C_M_q = -36.3;

% Yaw moment
C_N_beta = 0.1209;
C_N_p = -0.114;
C_N_r = -0.3;
C_N_delta_a = -0.00063;
C_N_delta_r = -0.1020;

%% ============================== INITIAL STATES ==============================
x = 0;
y = 0;
z = -1000;
phi = 0;
theta = -0.0018;
psi = 0;
u = 0;
v = 0;
w = 0;
V = 100;
alpha = -0.0018;
beta = 0;
p = 0;
q = 0;
r = 0;

X = [x y z phi theta psi u v w V alpha beta p q r]';

%% ============================== FLIGHT PARAMS ==============================
Qs     = 0.5 * rho * V^2;   % Dynamic pressure
T_max  = 68000;             % Max thrust
n_v    = 0;                 % Velocity exponent
n_rho  = 0.75;              % Density exponent
turn_rate = 10;             % turn rate
gamma     = 5;              % flight path angle

%% ============================== CONSTANTS INPUT VECTOR ==============================
M = [g, rho, Mass, C_b, b, S, ...
     I_xx, I_yy, I_zz, I_xz, I_yz, I_xy, ...
     C_L_0, C_L_alpha, C_L_q, C_L_delta_e, C_L_alpha_dot, ...
     C_D_0, C_D_alpha, C_D_delta_e, ...
     C_Y_beta, C_Y_p, C_Y_r, C_Y_delta_r, C_Y_delta_a, ...
     C_l_beta, C_l_p, C_l_r, C_l_delta_a, C_l_delta_r, ...
     C_M_0, C_M_alpha, C_M_alpha_dot, C_M_delta_e, C_M_q, ...
     C_N_beta, C_N_p, C_N_r, C_N_delta_a, C_N_delta_r, ...
     V, Qs, T_max, n_v, n_rho, turn_rate, gamma]';

%% ============================== INITIAL CONTROL INPUTS ==============================
delta_e = -0.0383;
delta_a = 0;
delta_r = 0;
delta_T = -0.0995;
U = [delta_e, delta_a, delta_r, delta_T]';

%% ============================== SIMULATION SETUP ==============================
s = 0.01;              % timestep
f = 10;                % final time
t = 0:s:f;
Afcs = zeros(15, length(t));
uas  = zeros(4, length(t));


%% ============================== MAIN SIMULATION LOOP ==============================
for i = 1:length(t)

    commanded = struct('c_theta', deg2rad(5), 'c_phi', deg2rad(0), 'c_V', 100);
    
    gains = struct('k_theta', 5.2, ...
    'kp_q', -1, ...
    'ki_q', 0, ...
    'eq_i', 0,['' ...
    'kp_phi'], 5.5, ...
    'kp_p', -0.09, ...
    'ki_p', 0.003, ...
    'eqi_p', 0, ...
    'kp_V', 0.6, ...
    'ki_V', 0.001, ...
    'kd_V', 0, ...
    'pe_V', 0, ...
    'eqi_V', 0);

    [U] = flight_controller(X, s, gains, commanded);
    % U = [delta_e, delta_a, delta_r, delta_T]'; % controller inputs if
    % want to check trim results
    
    %% Runge-Kutta Integration
    K1 = SixDof_eqm(X, M, U);
    K2 = SixDof_eqm(X + 0.5*s*K1, M, U);
    K3 = SixDof_eqm(X + 0.5*s*K2, M, U);
    K4 = SixDof_eqm(X + s*K3, M, U);
    X = X + (s/6)*(K1 + 2*K2 + 2*K3 + K4);

    %% Storing results in initialised arrays
    Afcs(:, i) = X;
    uas(:, i) = U;
end

%% ============================== PLOTS ==============================
figure;
labels = {'x', 'y', 'z', 'phi', 'theta', 'psi', 'u', 'v', 'w', 'V', 'alpha', 'beta', 'p', 'q', 'r'};
for k = 1:15
    subplot(6, 3, k);
    plot(t, Afcs(k, :));
    grid on;
    xlabel('Time (s)');
    ylabel(labels{k});
    title(labels{k});
end

% Control input plots
subplot(6, 3, 16); plot(t, uas(2, :)*57.3); title('Aileron Deflection'); ylabel('deg');
subplot(6, 3, 17); plot(t, uas(1, :)*57.3); title('Elevator Deflection'); ylabel('deg');
subplot(6, 3, 18); plot(t, uas(3, :)*57.3); title('Rudder Deflection'); ylabel('deg');

% 3D Trajectory
figure;
plot3(Afcs(1,:), Afcs(2,:), Afcs(3,:));
xlabel('x'); ylabel('y'); zlabel('z');
title('3D Aircraft Trajectory');
grid on;
