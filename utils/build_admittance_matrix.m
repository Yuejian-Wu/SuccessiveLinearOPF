function [Ybus, Yf, Yt] = build_admittance_matrix(mpc)
% BUILD_ADMITTANCE_MATRIX Build bus and branch admittance matrices
%
% Syntax:
%   [Ybus, Yf, Yt] = build_admittance_matrix(mpc)
%
% Description:
%   Constructs the bus admittance matrix (Ybus) and branch admittance
%   matrices (Yf, Yt) from the power system network data.
%
% Author: Successive Linear OPF Team
% Date: 2026

    baseMVA = mpc.baseMVA;
    bus = mpc.bus;
    branch = mpc.branch;
    
    n_bus = size(bus, 1);
    n_branch = size(branch, 1);
    
    % Create bus number to index mapping
    bus_num = bus(:, 1);
    bus_idx_map = containers.Map(bus_num, 1:n_bus);
    
    % Initialize admittance matrix
    Ybus = sparse(n_bus, n_bus);
    Yf = sparse(n_branch, n_bus);
    Yt = sparse(n_branch, n_bus);
    
    % Process each branch
    for k = 1:n_branch
        fbus = branch(k, 1);
        tbus = branch(k, 2);
        r = branch(k, 3);
        x = branch(k, 4);
        bch = branch(k, 5);
        tap = branch(k, 9);
        shift = branch(k, 10);
        status = branch(k, 11);
        
        if status == 0 || tap == 0
            continue;
        end
        
        % Bus indices
        f = bus_idx_map(fbus);
        t = bus_idx_map(tbus);
        
        % Series admittance
        if (r == 0) && (x == 0)
            y = 1e6;  % Very high admittance (short circuit)
        else
            y = 1 / (r + 1j*x);
        end
        
        % Tap ratio
        a = tap * exp(1j * shift);
        a_conj = conj(a);
        a_mag_sq = a * a_conj;
        
        % Add to bus admittance matrix
        Ybus(f, f) = Ybus(f, f) + y/a_mag_sq + 1j*bch/2;
        Ybus(t, t) = Ybus(t, t) + y + 1j*bch/2;
        Ybus(f, t) = Ybus(f, t) - y/a_conj;
        Ybus(t, f) = Ybus(t, f) - y/a;
        
        % Branch admittance matrix (from-bus perspective)
        Yf(k, f) = y/a_mag_sq;
        Yf(k, t) = -y/a_conj;
        
        % Branch admittance matrix (to-bus perspective)
        Yt(k, t) = y;
        Yt(k, f) = -y/a;
    end
    
end


function [Pij_lin, Qij_lin] = linearize_pf_equations(Ybus, V0, theta0, branch, baseMVA)
% LINEARIZE_PF_EQUATIONS Linearize AC power flow equations
%
% Syntax:
%   [Pij_lin, Qij_lin] = linearize_pf_equations(Ybus, V0, theta0, branch, baseMVA)
%
% Description:
%   Creates a linear approximation of power flow equations around the
%   operating point (V0, theta0).
%
% Author: Successive Linear OPF Team
% Date: 2026

    n_branch = size(branch, 1);
    n_bus = size(Ybus, 1);
    
    % Complex voltage at initial point
    V_complex = V0 .* exp(1j * theta0);
    
    % Compute initial power flows
    Pij_lin = zeros(n_branch, 1);
    Qij_lin = zeros(n_branch, 1);
    
    for k = 1:n_branch
        fbus = branch(k, 1);
        tbus = branch(k, 2);
        
        % Find bus indices
        bus_idx_map = containers.Map(1:n_bus, 1:n_bus);
        f_idx = fbus;
        t_idx = tbus;
        
        V_f = V_complex(f_idx);
        V_t = V_complex(t_idx);
        
        % Admittance
        y_ij = -Ybus(f_idx, t_idx);
        g_ij = real(y_ij);
        b_ij = imag(y_ij);
        
        % Power flow from f to t
        V_f_mag = abs(V_f);
        V_t_mag = abs(V_t);
        theta_diff = angle(V_f) - angle(V_t);
        
        Pij = V_f_mag * V_t_mag * (g_ij * cos(theta_diff) + b_ij * sin(theta_diff));
        Qij = V_f_mag * V_t_mag * (g_ij * sin(theta_diff) - b_ij * cos(theta_diff));
        
        Pij_lin(k) = Pij * baseMVA;
        Qij_lin(k) = Qij * baseMVA;
    end
    
end
