function [Ybus, Yf, Yt] = build_admittance_matrix(mpc)
% BUILD_ADMITTANCE_MATRIX Build nodal admittance matrix for power system
%
% Syntax:
%   [Ybus, Yf, Yt] = build_admittance_matrix(mpc)
%
% Description:
%   Constructs the bus admittance matrix (Ybus) and branch admittance matrices
%   (Yf, Yt) from power system case data. This matrix is fundamental for
%   power flow calculations and linearization.
%
% Input:
%   mpc - MATPOWER case structure
%
% Output:
%   Ybus - (n_bus x n_bus) Bus admittance matrix
%   Yf   - (n_branch x n_bus) "From" bus admittance matrix
%   Yt   - (n_branch x n_bus) "To" bus admittance matrix
%
% Mathematical Background:
%   For a series impedance Z = R + jX on a line:
%     Y_series = 1/Z = G + jB
%   For shunt admittance (capacitive or inductive):
%     Y_shunt = G_shunt + jB_shunt
%   
%   The Ybus matrix is constructed using:
%   Y_ii = sum of admittances connected to bus i
%   Y_ij = -admittance of line between bus i and j
%
% Author: Successive Linear OPF Team
% Date: 2026

    % Extract data
    baseMVA = mpc.baseMVA;
    bus = mpc.bus;
    branch = mpc.branch;
    
    % Bus and branch indices
    bus_idx = bus(:, 1);
    n_bus = length(bus_idx);
    n_branch = size(branch, 1);
    
    % Create bus number to index mapping
    bus_map = containers.Map(bus_idx, 1:n_bus);
    
    % Initialize admittance matrices
    Ybus = complex(zeros(n_bus, n_bus));
    Yf = complex(zeros(n_branch, n_bus));
    Yt = complex(zeros(n_branch, n_bus));
    
    % Process each branch
    for k = 1:n_branch
        
        % Get branch data: fbus, tbus, r, x, b, rateA, ..., ratio, angle, status
        fbus = branch(k, 1);
        tbus = branch(k, 2);
        r = branch(k, 3);
        x = branch(k, 4);
        b = branch(k, 5);
        tap = branch(k, 9);      % Transformer tap ratio
        shift = branch(k, 10);   % Phase shift angle (radians)
        status = branch(k, 11);  % Line status (1=on, 0=off)
        
        if status == 0
            continue;  % Skip off-line branches
        end
        
        % Convert bus numbers to indices
        i = bus_map(fbus);
        j = bus_map(tbus);
        
        % Series admittance
        if (r ~= 0) || (x ~= 0)
            y_series = 1 / (r + 1i*x);
        else
            y_series = 0;
        end
        
        % Shunt admittance (typically for transmission lines)
        y_shunt = 1i * b / 2;  % Half-line charging susceptance on each end
        
        % Handle transformer tap (if tap != 0 or phase shift != 0)
        if (tap == 0)
            tap = 1;  % Default to 1 if not specified
        end
        
        % Complex tap ratio: a = tap * exp(j*shift)
        a = tap * exp(1i * shift);
        
        % Admittance from "from" bus perspective
        y_f = y_series / (a * conj(a)) + y_shunt;
        
        % Admittance from "to" bus perspective
        y_t = y_series + y_shunt;
        
        % Admittance between buses
        y_ft = -y_series / conj(a);
        
        % Build Ybus matrix
        Ybus(i, i) = Ybus(i, i) + y_f;
        Ybus(j, j) = Ybus(j, j) + y_t;
        Ybus(i, j) = Ybus(i, j) + y_ft;
        Ybus(j, i) = Ybus(j, i) + conj(y_ft);
        
        % Build branch admittance matrices for flow calculations
        Yf(k, i) = y_f;
        Yf(k, j) = y_ft;
        Yt(k, j) = y_t;
        Yt(k, i) = conj(y_ft);
        
    end
    
    % Add bus shunt admittances (capacitors, reactors)
    for i = 1:n_bus
        Gs = bus(i, 5);  % Shunt conductance
        Bs = bus(i, 6);  % Shunt susceptance
        
        if (Gs ~= 0) || (Bs ~= 0)
            Ybus(i, i) = Ybus(i, i) + Gs + 1i*Bs;
        end
    end
    
    % Display matrix information
    display_admittance_info(Ybus, n_bus, n_branch);
    
end


function display_admittance_info(Ybus, n_bus, n_branch)
% DISPLAY_ADMITTANCE_INFO Display information about admittance matrix

    % Calculate matrix statistics
    n_nonzero = nnz(Ybus);
    sparsity = (1 - n_nonzero / (n_bus^2)) * 100;
    
    % Diagonal elements (self-admittances)
    diag_elements = diag(Ybus);
    max_y_mag = max(abs(diag_elements));
    
    fprintf('\n========== Admittance Matrix Information ==========\n');
    fprintf('Matrix Size: %d x %d\n', n_bus, n_bus);
    fprintf('Non-zero Elements: %d\n', n_nonzero);
    fprintf('Sparsity: %.2f%%\n', sparsity);
    fprintf('Max Self-Admittance: %.4f p.u.\n', max_y_mag);
    fprintf('Number of Branches: %d\n', n_branch);
    fprintf('====================================================\n\n');
    
end
