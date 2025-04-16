function [U] = flight_controller(X, s, gains, commanded)
% AFCS_CONTROLLER - Computes control inputs based on feedback laws.
%
% Inputs:
%   X - State vector [15x1]
%   s - Time step (scalar)
%   gains - Struct holding gains and integrators
%
% Outputs:
%   U - Control vector [delta_e, delta_a, delta_r, delta_T]'
%   gains - Updated integrator and memory states

% state variables
phi = X(4); 
theta = X(5);
u = X(7);
V = X(10);
alpha = X(11);
p = X(13);
q = X(14);
r = X(15);

%% === Pitch Control ===
e_theta = commanded.c_theta - theta; %pitch error
d_q = gains.k_theta * e_theta;
e_q = d_q - q;
gains.eq_i = gains.eq_i + e_q * s;
delta_e = gains.kp_q * e_q + gains.ki_q * gains.eq_i + 0.0256;

%% === Roll Control ===
e_phi = commanded.c_phi - phi; %roll error
d_p = gains.kp_phi * e_phi;
e_p = d_p - p;
gains.eqi_p = gains.eqi_p + e_p * s;
delta_a = gains.kp_p * e_p + gains.ki_p * gains.eqi_p;

%% === Airspeed Control ===
e_V = commanded.c_V - V; %speed error
p_V = gains.kp_V * e_V;
gains.eqi_V = gains.eqi_V + e_V * s;
de_V = (e_V - gains.pe_V) / s;
dc_V = gains.kd_V * de_V;
delta_T = p_V + gains.ki_V * gains.eqi_V + dc_V;
gains.pe_V = e_V;

%% === Assemble Control Vector ===
delta_r = 0; % Rudder not in use (or replace with yaw control logic)
U = [delta_e; delta_a; delta_r; delta_T];

end
