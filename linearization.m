function [A, B, C, D, sys] = linearization(X_trim, U_trim, M)
% LINEARISE_AFCS - Linearizes the nonlinear AFCS dynamics around a trim condition
% Inputs:
%   X_trim - 15x1 state vector at trim condition
%   U_trim - 4x1 control vector at trim condition
%   M      - parameter vector (from aircraft config)
% Outputs:
%   A, B, C, D - linearized state-space matrices
%   sys        - state-space object

% Perturbation step for finite difference
DE = 1e-5;
nx = length(X_trim);
nu = length(U_trim);
ny = nx;  % Assuming full state feedback for now

% Preallocate matrices
A = zeros(ny, nx);
B = zeros(ny, nu);
C = eye(ny);
D = zeros(ny, nu);

% Nominal dynamics at trim
Y0 = SixDof_eqm(X_trim, M, U_trim);

% Compute A matrix (dynamics w.r.t. states)
for i = 1:nx
    dX = zeros(nx, 1); dX(i) = DE;
    Y = SixDof_eqm(X_trim + dX, M, U_trim);
    A(:, i) = (Y - Y0) / DE;
end

% Compute B matrix (dynamics w.r.t. controls)
for j = 1:nu
    dU = zeros(nu, 1); dU(j) = DE;
    Y = SixDof_eqm(X_trim, M, U_trim + dU);
    B(:, j) = (Y - Y0) / DE;
end

% Build state-space system
sys = ss(A, B, C, D);
end
