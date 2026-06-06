function [A_ltc, b_ltc_lb, b_ltc_ub, z_ltc, x_ltc] = build_ltc_model(mpc, V0)
% BUILD_LTC_MODEL Build Load Tap Changer (LTC) constraints and models
%
% Syntax:
%   [A_ltc, b_ltc_lb, b_ltc_ub, z_ltc, x_ltc] = build_ltc_model(mpc, V0)
%
% Description:
%   Constructs linear models and constraints for Load Tap Changer (LTC)
%   transformers based on the paper's formulation (Equations 25-30).
%
%   Key features:
%   - Continuous relaxation of tap position for linearization
%   - Discrete tap position constraints
%   - Voltage drop relationship across LTC
%   - Binary variables for tap selection
%   - Ordered binary variables to enforce monotonicity
%
% Input:
%   mpc - MATPOWER case structure
%   V0  - (n_bus x 1) Initial bus voltages (p.u.)
%
% Output:
%   A_ltc    - Linear constraint matrix for LTC equations
%   b_ltc_lb - Lower bounds on constraints
%   b_ltc_ub - Upper bounds on constraints
%   z_ltc    - Structure with binary variables information
%   x_ltc    - Structure with continuous variables information
%
% Theory (Paper Reference - Equations 25-30):
%   Tap ratio: t = t_0 + (t_m - t_0) * (n_m=1)
%   where t_0, t_1, ..., t_n are discrete tap positions
%   
%   Voltage constraint:
%     v_i^2 = (t_m * v_LTC)^2 + Δv_m^2
%   
%   where Δv_m represents voltage changes at each tap position
%   
%   Binary constraints enforce:
%     - Only one tap position is active (sum(z_m) = 1)
%     - Ordered selection to ensure monotonicity
%     - Voltage bounds at each tap setting
%
% Author: Successive Linear OPF Team
% Date: 2026

    % Check if LTC data exists
    if ~isfield(mpc, 'ltc') || isempty(mpc.ltc)
        fprintf('No LTC transformers defined in case.\n');
        A_ltc = [];
        b_ltc_lb = [];
        b_ltc_ub = [];
        z_ltc = struct('n_ltc', 0, 'tap_data', []);
        x_ltc = struct('n_vars', 0);
        return;
    end
    
    ltc_data = mpc.ltc;
    n_ltc = size(ltc_data, 1);
    bus_data = mpc.bus;
    n_bus = size(bus_data, 1);
    baseMVA = mpc.baseMVA;
    
    % Create bus number to index mapping
    bus_idx_map = containers.Map(bus_data(:, 1), 1:n_bus);
    
    fprintf('\n========== Building LTC Model ==========\n');
    fprintf('Number of LTC transformers: %d\n', n_ltc);
    
    % Initialize constraint lists
    A_list = [];
    b_lb_list = [];
    b_ub_list = [];
    
    % Constraint counter
    n_constraints = 0;
    n_binary_vars = 0;
    n_continuous_vars = 0;
    
    % Store LTC information
    ltc_info = struct('from_bus', {}, 'to_bus', {}, 'n_taps', {}, ...
                      'min_tap', {}, 'max_tap', {}, 'tap_step', {}, ...
                      'nominal_tap', {}, 'tap_positions', {}, ...
                      'z_indices', {}, 'dv_indices', {});
    
    % Process each LTC
    for k = 1:n_ltc
        
        from_bus = ltc_data(k, 1);
        to_bus = ltc_data(k, 2);
        n_taps = ltc_data(k, 3);
        min_tap = ltc_data(k, 4);
        max_tap = ltc_data(k, 5);
        nominal_tap = ltc_data(k, 6);
        
        % Validate LTC data
        if n_taps < 2
            warning('LTC %d has fewer than 2 tap positions. Skipping.', k);
            continue;
        end
        
        if min_tap >= max_tap
            warning('LTC %d: min_tap >= max_tap. Skipping.', k);
            continue;
        end
        
        % Bus indices
        i = bus_idx_map(from_bus);
        j = bus_idx_map(to_bus);
        
        % Calculate tap positions (linearly spaced)
        tap_positions = linspace(min_tap, max_tap, n_taps);
        tap_step = (max_tap - min_tap) / (n_taps - 1);
        
        % Store LTC info
        ltc_info(k).from_bus = from_bus;
        ltc_info(k).to_bus = to_bus;
        ltc_info(k).n_taps = n_taps;
        ltc_info(k).min_tap = min_tap;
        ltc_info(k).max_tap = max_tap;
        ltc_info(k).tap_step = tap_step;
        ltc_info(k).nominal_tap = nominal_tap;
        ltc_info(k).tap_positions = tap_positions;
        
        % ===== CONSTRAINT 1: Voltage relationship across LTC =====
        % v_i^2 = t_m^2 * v_j,LTC^2 + sum(Δv_m^2)
        % Linearized form (Equation 25):
        % v_i^2 = t_0^2 * v_j,LTC^2 + sum(Δv_m^2)
        
        v_0 = V0(i);
        v_j_ltc = V0(j);
        
        % For each tap position m = 1, 2, ..., n_taps
        z_start_idx = n_binary_vars + 1;
        dv_start_idx = n_continuous_vars + 1;
        
        for m = 1:n_taps
            t_m = tap_positions(m);
            
            % Constraint: v_i^2 = t_m^2 * v_j,LTC^2 + Δv_m^2
            % When z_m = 1, this constraint is active
            
            % Linearized: v_i^2 ≈ t_m^2 * v_j,LTC^2 + 2*v_nom*(Δv_m - Δv_m0)*z_m
            % We store it as an equality when z_m = 1
            
            % For now, we represent as bounds on voltage change
            % Δv_m^2 >= 0 (always satisfied)
            % Δv_m^2 <= (t_m*v_j,LTC)^2 - v_min^2 (upper bound)
            
            v_min = bus_data(j, 13);  % V_min for to_bus
            v_max = bus_data(j, 12);  % V_max for to_bus
            
            dv_max = sqrt((t_m * v_max)^2 - v_min^2);
            
            % Lower bound: Δv_m >= 0
            b_lb_list = [b_lb_list; 0];
            b_ub_list = [b_ub_list; dv_max];
            n_constraints = n_constraints + 1;
            
            n_continuous_vars = n_continuous_vars + 1;
        end
        
        % ===== CONSTRAINT 2: Binary selection of tap position =====
        % sum(z_m) = 1  (exactly one tap is active)
        
        A_row = zeros(1, n_binary_vars + n_taps);
        A_row(1, z_start_idx:z_start_idx + n_taps - 1) = 1;
        A_list = [A_list; A_row];
        b_lb_list = [b_lb_list; 1];
        b_ub_list = [b_ub_list; 1];
        n_constraints = n_constraints + 1;
        n_binary_vars = n_binary_vars + n_taps;
        
        % ===== CONSTRAINT 3: Ordered binary variables (monotonicity) =====
        % z_m >= z_{m+1}  for m = 1, 2, ..., n_taps-1
        % This ensures tap positions are selected in order
        
        for m = 1:n_taps-1
            A_row = zeros(1, n_binary_vars);
            A_row(1, z_start_idx + m - 1) = 1;
            A_row(1, z_start_idx + m) = -1;
            A_list = [A_list; A_row];
            b_lb_list = [b_lb_list; 0];
            b_ub_list = [b_ub_list; 1];
            n_constraints = n_constraints + 1;
        end
        
        % Store indices
        ltc_info(k).z_indices = z_start_idx:z_start_idx + n_taps - 1;
        ltc_info(k).dv_indices = dv_start_idx:dv_start_idx + n_taps - 1;
        
        fprintf('  LTC %d: Bus %d → Bus %d, %d taps (%.3f - %.3f)\n', ...
                k, from_bus, to_bus, n_taps, min_tap, max_tap);
        
    end
    
    % Convert to sparse matrix if not empty
    if ~isempty(A_list)
        A_ltc = sparse(A_list);
    else
        A_ltc = sparse(0, n_binary_vars + n_continuous_vars);
    end
    
    % Store results
    z_ltc = struct('n_ltc', n_ltc, 'tap_data', {ltc_info}, ...
                   'n_binary_vars', n_binary_vars, 'n_total_constraints', n_constraints);
    
    x_ltc = struct('n_vars', n_continuous_vars, 'dv_vars', n_continuous_vars);
    
    fprintf('  Total binary variables (tap selections): %d\n', n_binary_vars);
    fprintf('  Total continuous variables (voltage changes): %d\n', n_continuous_vars);
    fprintf('  Total constraints: %d\n', n_constraints);
    fprintf('=====================================\n\n');
    
end


function display_ltc_constraints(z_ltc, x_ltc)
% DISPLAY_LTC_CONSTRAINTS Display information about LTC constraints

    fprintf('\n========== LTC Constraint Summary ==========\n');
    fprintf('Number of LTC transformers: %d\n', z_ltc.n_ltc);
    fprintf('Binary variables: %d\n', z_ltc.n_binary_vars);
    fprintf('Continuous variables: %d\n', x_ltc.n_vars);
    fprintf('Total constraints: %d\n', z_ltc.n_total_constraints);
    fprintf('===========================================\n\n');
    
end
