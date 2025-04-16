function Xdot = SixDof_eqm(X, M, U)
% AFCS_V - Aircraft Equations of Motion
% This function computes the derivatives of the aircraft states for 
% 6-DOF motion using aerodynamic and thrust models.
% Inputs:
%   X - State vector [x, y, z, phi, theta, psi, u, v, w, V, alpha, beta, p, q, r]
%   M - Model parameters (aerodynamic coefficients, inertias, geometry, etc.)
%   U - Control inputs [delta_e, delta_a, delta_r, delta_T]
% Output:
%   Xdot - State derivatives

%% ============================== STATES ==============================
x       = X(1);   
y       = X(2);   
z       = X(3);
phi     = X(4);
theta   = X(5);
psi     = X(6);
u       = X(7);
v       = X(8);
w       = X(9);
V       = X(10);
alpha   = X(11);
beta    = X(12);
p       = X(13);
q       = X(14);
r       = X(15);

%% ============================== INPUTS & CONSTANTS ==============================
g = M(1); rho = M(2); Mass = M(3); C_b = M(4); b = M(5); S = M(6);
I_xx = M(7); I_yy = M(8); I_zz = M(9); I_xz = M(10); I_yz = M(11); I_xy = M(12);
C_L_0 = M(13); C_L_alpha = M(14); C_L_q = M(15); C_L_delta_e = M(16); C_L_alpha_dot = M(17);
C_D_0 = M(18); C_D_alpha = M(19); C_D_delta_e = M(20);
C_Y_beta = M(21); C_Y_p = M(22); C_Y_r = M(23); C_Y_delta_r = M(24); C_Y_delta_a = M(25);
C_l_beta = M(26); C_l_p = M(27); C_l_r = M(28); C_l_delta_a = M(29); C_l_delta_r = M(30);
C_M_0 = M(31); C_M_alpha = M(32); C_M_alpha_dot = M(33); C_M_delta_e = M(34); C_M_q = M(35);
C_N_beta = M(36); C_N_p = M(37); C_N_r = M(38); C_N_delta_a = M(39); C_N_delta_r = M(40);
V_ref = M(41); Qs = M(42); T_max = M(43); n_v = M(44); n_rho = M(45);
turn_rate = M(46); gamma = M(47);

% Control Inputs
delta_e = U(1);
delta_a = U(2); 
delta_r = U(3); 
delta_T = U(4);

%% ============================== GEOMETRY & FRAME ==============================
C_G = 0.3 * C_b; % MAC center
X_cg = (C_G - 0.25) * C_b; Y_cg = 0; Z_cg = -0.4;

% AoA rate
alpha_dot = 0;

% Wind-to-body transformation
SBT = inv([cos(alpha), 0, -sin(alpha); 0, 1, 0; sin(alpha), 0, cos(alpha)]);

%% ============================== AERODYNAMIC COEFFICIENTS ==============================
CL = C_L_0 + C_L_alpha * alpha + C_L_delta_e * delta_e + C_L_alpha_dot * (alpha_dot * C_b / (2 * V)) + C_L_q * (q * C_b / (2 * V));
CM = C_M_0 + C_M_alpha * alpha + C_M_delta_e * delta_e + C_M_alpha_dot * (alpha_dot * C_b / (2 * V)) + C_M_q * (q * C_b / (2 * V));
CD = C_D_0 + C_D_alpha * alpha + C_D_delta_e * delta_e;
CY = C_Y_beta * beta + C_Y_delta_a * delta_a + C_Y_delta_r * delta_r + C_Y_p * (p * b / (2 * V)) + C_Y_r * (r * b / (2 * V));
Cl = C_l_beta * beta + C_l_delta_a * delta_a + C_l_delta_r * delta_r + C_l_p * (p * b / (2 * V)) + C_l_r * (r * b / (2 * V));
CN = C_N_beta * beta + C_N_delta_a * delta_a + C_N_delta_r * delta_r + C_N_p * (p * b / (2 * V)) + C_N_r * (r * b / (2 * V));

% Body-axis force/moment coefficients
CX_b = CL * sin(alpha) - CD * cos(alpha);
CZ_b = CD * sin(alpha) + CL * cos(alpha);
CY_b = CY;
CM_b = CM;
Cl_b = Cl * cos(alpha) - CN * sin(alpha);
CN_b = Cl * sin(alpha) + CN * cos(alpha);

%% ============================== THRUST ==============================
Thrust = delta_T * T_max * (0.9^n_rho); % thrust decay

%% ============================== FORCES ==============================
Fx_A = Qs * S * CX_b;
Fy_A = Qs * S * CY_b;
Fz_A = Qs * S * CZ_b;
Fx_t = Thrust * cosd(-1.8); % Convert degrees
Fy_t = 0;
Fz_t = Thrust * sind(-1.8);
FA = [Fx_A; Fy_A; Fz_A];

%% ============================== MOMENTS ==============================
M_xext = Qs * S * b * Cl_b - Fy_A * Z_cg - Fz_A * Y_cg;
M_yext = Qs * S * C_b * CM_b + Fx_A * Z_cg - Fz_A * X_cg - (Thrust * cosd(-1.8) * (-0.6) + Thrust * sind(-1.8) * (-3.8));
M_zext = Qs * S * b * CN_b + Fy_A * X_cg + Fx_A * Y_cg;

%% ============================== KINEMATICS ==============================
% Velocity states
u = V * cos(alpha) * cos(beta);
v = V * sin(beta);
w = V * sin(alpha) * cos(beta);

% Position derivatives
xe_dot = u * (cos(theta)*cos(psi)) + v * (sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi)) + w * (cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi));
ye_dot = u * (cos(theta)*sin(psi)) + v * (sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi)) + w * (cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi));
ze_dot = -u * sin(theta) + v * sin(phi) * cos(theta) + w * cos(phi) * cos(theta);

% Attitude rates
phi_dot   = p + q * sin(phi) * tan(theta) + r * cos(phi) * tan(theta);
theta_dot = q * cos(phi) - r * sin(phi);
psi_dot   = (q * sin(phi) + r * cos(phi)) / cos(theta);

%% ============================== DYNAMICS ==============================
u_dot = (1/Mass) * (-Fx_A + Fx_t - Mass*g*sin(theta)) + r*v - q*w;
v_dot = (1/Mass) * (Fy_A + Fy_t + Mass*g*cos(theta)*sin(phi)) + p*w - r*u;
w_dot = (1/Mass) * (-Fz_A + Fz_t + Mass*g*cos(theta)*cos(phi)) + q*u - p*v;

V_dot = (u*u_dot + v*v_dot + w*w_dot) / V;
alpha_dot = (u*w_dot - w*u_dot) / (u^2 + w^2);
beta_dot  = (v_dot * V - v * V_dot) / (V * sqrt(u^2 + w^2));

%% ============================== ROTATIONAL DYNAMICS ==============================
p_dot = (1/(I_xx*I_zz - I_xz^2)) *(I_xz*M_zext + I_zz*M_xext + I_xz*(I_xx - I_yy + I_zz)*p*q - ((I_xz^2 - I_yy*I_zz + I_zz^2)*q*r));
q_dot = (1/I_yy) * (M_yext - (I_xx - I_zz)*r*p - I_xz*(p^2 - r^2));
r_dot = (1/(I_xx*I_zz - I_xz^2)) * (I_xx*M_zext + I_xz*M_xext - I_xz*(I_xx - I_yy + I_zz)*q*r + ((I_xz^2 - I_yy*I_xx + I_xx^2)*p*q));

%% ============================== STATE DERIVATIVES ==============================
Xdot = [xe_dot; ye_dot; ze_dot; phi_dot; theta_dot; psi_dot;u_dot; v_dot; w_dot; V_dot; alpha_dot; beta_dot;p_dot; q_dot; r_dot];
end
