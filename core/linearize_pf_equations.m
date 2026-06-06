function [Pij_lin, Qij_lin] = linearize_pf_equations(Ybus, V0, theta0, branch, baseMVA)
% LINEARIZE_PF_EQUATIONS Linearize power flow equations using first-order Taylor expansion
%
% Syntax:
%   [Pij_lin, Qij_lin] = linearize_pf_equations(Ybus, V0, theta0, branch, baseMVA)
%
% Description:
%   Implements the linear approximation of power flow equations as described
%   in the paper. Uses first-order Taylor series expansion around an initial
%   point (V0, theta0) to approximate the nonlinear P-V-θ relationship.
%
%   Key approximations (from paper equations 16-20):
%   - Linear approximation of viVj product
%   - Decoupling of voltage and angle effects
%   - Accurate modeling of reactive power from shunt devices
%
% Input:
%   Ybus     - (n_bus x n_bus) Bus admittance matrix
%   V0       - (n_bus x 1) Initial voltage magnitudes (p.u.)
%   theta0   - (n_bus x 1) Initial voltage angles (radians)
%   branch   - Branch data matrix from mpc.branch
%   baseMVA  - Base power (MVA)
%
% Output:
%   Pij_lin  - Structure with linear approximation coefficients for active power
%   Qij_lin  - Structure with linear approximation coefficients for reactive power
%
% Theory (Paper Reference):
%   The nonlinear power flow equation:
%     P_ij = v_i^2 * g_ij - v_i*v_j*(g_ij*cos(θ_ij) + b_ij*sin(θ_ij))
%   
%   Is linearized to:
%     P_ij^lin ≈ g_ij*v_i^2 - (g_ij^PC + b_ij^PC) + g_ij^PC*(θ_i - θ_j)
%   
%   Where the coefficients are evaluated at the initial point (V0, θ0)
%
% Author: Successive Linear OPF Team
% Date: 2026

    % Extract system dimensions
    n_bus = length(V0);
    n_branch = size(branch, 1);
    
    % Initialize output structures
    Pij_lin = struct('G', [], 'H', [], 'K', []);
    Qij_lin = struct('G', [], 'H', [], 'K', []);
    
    % Matrices to store coefficients
    P_G = zeros(n_branch, 1);      % Coefficient of v^2
    P_H = zeros(n_branch, 1);      % Constant term
    P_K = zeros(n_branch, 2);      % Coefficients for angle difference
    
    Q_G = zeros(n_branch, 1);
    Q_H = zeros(n_branch, 1);
    Q_K = zeros(n_branch, 2);
    
    % Create bus number to index mapping
    bus_idx = (1:n_bus)';
    
    % Process each branch
    for k = 1:n_branch
        
        % Get branch data: fbus, tbus, r, x, b, rateA, ..., ratio, angle, status
        fbus = branch(k, 1);
        tbus = branch(k, 2);
        r = branch(k, 3);
        x = branch(k, 4);
        b_shunt = branch(k, 5);
        tap = branch(k, 9);
        shift = branch(k, 10);
        status = branch(k, 11);
        
        if status == 0 || tap == 0
            continue;
        end
        
        % Find bus indices
        i = fbus;
        j = tbus;
        
        % Series admittance
        if (r ~= 0) || (x ~= 0)
            y_series = 1 / (r + 1i*x);
            g_ij = real(y_series);
            b_ij = imag(y_series);
        else
            g_ij = 0;
            b_ij = 0;
        end
        
        % Initial voltages and angles
        v_i = V0(i);
        v_j = V0(j);
        theta_i = theta0(i);
        theta_j = theta0(j);
        theta_ij = theta_i - theta_j;
        
        % Complex tap ratio
        a = tap * exp(1i * shift);
        a_mag = abs(a);
        
        % ===== ACTIVE POWER LINEARIZATION (Equations 19 in paper) =====
        
        % Coefficient of v_i^2
        P_G(k) = g_ij / (a_mag^2);
        
        % Taylor expansion around (V0, theta0)
        % cos(theta_ij) ≈ cos(theta_ij0) - sin(theta_ij0)*(theta_ij - theta_ij0)
        % sin(theta_ij) ≈ sin(theta_ij0) + cos(theta_ij0)*(theta_ij - theta_ij0)
        
        cos_th = cos(theta_ij);
        sin_th = sin(theta_ij);
        
        % Linearized active power flow from i to j
        % P_ij^lin = g*v_i^2/a^2 - v_i*v_j*(g*cos(th) + b*sin(th)) - v_i*v_j*(-g*sin(th) + b*cos(th))*dtheta
        
        P_coeff_th = v_i * v_j * (-g_ij * sin_th + b_ij * cos_th) / a_mag;
        
        P_H(k) = -v_i * v_j * (g_ij * cos_th + b_ij * sin_th) / a_mag;
        
        % Angle sensitivity
        P_K(k, 1) = P_coeff_th;     % ∂P_ij/∂θ_i
        P_K(k, 2) = -P_coeff_th;    % ∂P_ij/∂θ_j
        
        % ===== REACTIVE POWER LINEARIZATION (Equations 20 in paper) =====
        
        % Coefficient of v_i^2
        Q_G(k) = -b_ij / (a_mag^2) - b_shunt/2;
        
        % Linearized reactive power flow
        Q_coeff_th = v_i * v_j * (-b_ij * sin_th - g_ij * cos_th) / a_mag;
        
        Q_H(k) = -v_i * v_j * (-b_ij * cos_th + g_ij * sin_th) / a_mag;
        
        % Angle sensitivity
        Q_K(k, 1) = Q_coeff_th;     % ∂Q_ij/∂θ_i
        Q_K(k, 2) = -Q_coeff_th;    % ∂Q_ij/∂θ_j
        
    end
    
    % Store results
    Pij_lin.G = P_G;       % v^2 coefficient
    Pij_lin.H = P_H;       % Constant term
    Pij_lin.K = P_K;       % Angle coefficients
    
    Qij_lin.G = Q_G;
    Qij_lin.H = Q_H;
    Qij_lin.K = Q_K;
    
    % Display linearization information
    display_linearization_info(Pij_lin, Qij_lin, n_branch);
    
end


function display_linearization_info(Pij_lin, Qij_lin, n_branch)
% DISPLAY_LINEARIZATION_INFO Display information about linearization

    fprintf('\n========== Power Flow Linearization ==========\n');
    fprintf('Number of Branches: %d\n', n_branch);
    fprintf('Active Power (P) Coefficients:\n');
    fprintf('  - Max G coefficient: %.6f\n', max(abs(Pij_lin.G)));
    fprintf('  - Max H coefficient: %.6f\n', max(abs(Pij_lin.H)));
    fprintf('  - Max K coefficient: %.6f\n', max(max(abs(Pij_lin.K))));
    fprintf('Reactive Power (Q) Coefficients:\n');
    fprintf('  - Max G coefficient: %.6f\n', max(abs(Qij_lin.G)));
    fprintf('  - Max H coefficient: %.6f\n', max(abs(Qij_lin.H)));
    fprintf('  - Max K coefficient: %.6f\n', max(max(abs(Qij_lin.K))));
    fprintf('==============================================\n\n');
    
end
