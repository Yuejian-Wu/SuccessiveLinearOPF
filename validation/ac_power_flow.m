function [P_flow, Q_flow, violation_info] = ac_power_flow(mpc, V, theta, Ybus)
% AC_POWER_FLOW Calculate AC power flow and branch power flows
%
% Syntax:
%   [P_flow, Q_flow, violation_info] = ac_power_flow(mpc, V, theta, Ybus)
%
% Description:
%   Computes the exact nonlinear AC power flow given bus voltages and angles.
%   Used for verification of solutions from the linearized OPF model.
%
%   Calculates:
%   - Active and reactive power injections at each bus
%   - Branch power flows (from-bus and to-bus)
%   - Constraint violations (thermal, voltage, reactive power limits)
%   - System losses
%
% Input:
%   mpc   - MATPOWER case structure
%   V     - (n_bus x 1) Bus voltage magnitudes (p.u.)
%   theta - (n_bus x 1) Bus voltage angles (radians)
%   Ybus  - (n_bus x n_bus) Bus admittance matrix (complex)
%
% Output:
%   P_flow - Structure with power flow results
%            .P_inj    - Active power injection at each bus (MW)
%            .Q_inj    - Reactive power injection at each bus (MVar)
%            .Pij      - Active power flow on each branch (MW)
%            .Qij      - Reactive power flow on each branch (MVar)
%            .loss     - Total system losses (MW)
%
%   Q_flow - Structure with reactive power details
%            .Q_gen    - Generator reactive power (MVar)
%            .Q_shunt  - Shunt reactive power injection (MVar)
%            .Q_load   - Load reactive power demand (MVar)
%
%   violation_info - Structure with constraint violations
%                    .voltage_violation - Buses with voltage violations
%                    .thermal_violation - Branches with thermal violations
%                    .reactive_violation - Generators with reactive limit violations
%                    .n_violations - Total number of violations
%                    .max_violation - Maximum violation magnitude
%
% Theory:
%   Bus voltage: V_i = V_i * exp(j*theta_i)
%   
%   Current injection at bus i:
%     I_i = sum_j(Y_ij * V_j)
%   
%   Power injection at bus i:
%     P_i + jQ_i = V_i * conj(I_i) = V_i * conj(sum_j(Y_ij * V_j))
%     P_i = |V_i| * sum_j(|Y_ij| * |V_j| * cos(angle(Y_ij) + theta_j - theta_i))
%     Q_i = |V_i| * sum_j(|Y_ij| * |V_j| * sin(angle(Y_ij) + theta_j - theta_i))
%
% Author: Successive Linear OPF Team
% Date: 2026

    % Extract system data
    baseMVA = mpc.baseMVA;
    bus = mpc.bus;
    gen = mpc.gen;
    branch = mpc.branch;
    
    n_bus = size(bus, 1);
    n_gen = size(gen, 1);
    n_branch = size(branch, 1);
    
    % Create bus number to index mapping
    bus_idx_map = containers.Map(bus(:, 1), 1:n_bus);
    
    fprintf('\n========== AC Power Flow Calculation ==========\n');
    
    % ===== CALCULATE BUS POWER INJECTIONS =====
    % P_i + jQ_i = V_i * conj(I_i) = V_i * conj(Ybus * V)
    
    % Complex voltage vector
    V_complex = V .* exp(1j * theta);
    
    % Current injection
    I = Ybus * V_complex;
    
    % Power injection (in base units: p.u.)
    S_inj = V_complex .* conj(I);
    P_inj_pu = real(S_inj);
    Q_inj_pu = imag(S_inj);
    
    % Convert to MW and MVar
    P_inj = P_inj_pu * baseMVA;
    Q_inj = Q_inj_pu * baseMVA;
    
    % ===== CALCULATE BRANCH POWER FLOWS =====
    % For branch from bus i to bus j:
    %   S_ij = V_i * conj(I_ij)
    %   where I_ij is current in the line
    
    Pij = zeros(n_branch, 1);
    Qij = zeros(n_branch, 1);
    Pji = zeros(n_branch, 1);
    Qji = zeros(n_branch, 1);
    
    for k = 1:n_branch
        fbus = branch(k, 1);
        tbus = branch(k, 2);
        r = branch(k, 3);
        x = branch(k, 4);
        b = branch(k, 5);
        tap = branch(k, 9);
        shift = branch(k, 10);
        status = branch(k, 11);
        
        if status == 0 || tap == 0
            continue;
        end
        
        % Bus indices
        i = bus_idx_map(fbus);
        j = bus_idx_map(tbus);
        
        % Voltages and angles
        V_i = V(i);
        V_j = V(j);
        theta_i = theta(i);
        theta_j = theta(j);
        
        % Series admittance
        if (r ~= 0) || (x ~= 0)
            y_series = 1 / (r + 1j*x);
            g_ij = real(y_series);
            b_ij = imag(y_series);
        else
            g_ij = 0;
            b_ij = 0;
        end
        
        % Shunt admittance
        b_shunt = b / 2;
        
        % Complex tap ratio
        a = tap * exp(1j * shift);
        a_conj = conj(a);
        a_mag_sq = a * a_conj;
        
        % ===== FROM BUS (i to j) POWER FLOW =====
        % Current from i to j
        I_ij = (1/a) * (g_ij + 1j*b_ij) * V_i - (1/a_conj) * (g_ij + 1j*b_ij) * V_j;
        
        % Add shunt current
        I_sh_i = 1j * b_shunt * V_i;
        
        % Total current injected at i due to this branch
        I_total_i = I_ij + I_sh_i;
        
        % Power flow from i
        S_ij = V_i * conj(I_total_i);
        Pij(k) = real(S_ij) * baseMVA;
        Qij(k) = imag(S_ij) * baseMVA;
        
        % ===== TO BUS (j to i) POWER FLOW =====
        % Current from j to i
        I_ji = (1/a_conj) * (g_ij + 1j*b_ij) * V_j - (1/a) * (g_ij + 1j*b_ij) * V_i;
        
        % Add shunt current
        I_sh_j = 1j * b_shunt * V_j;
        
        % Total current injected at j due to this branch
        I_total_j = I_ji + I_sh_j;
        
        % Power flow to j
        S_ji = V_j * conj(I_total_j);
        Pji(k) = real(S_ji) * baseMVA;
        Qji(k) = imag(S_ji) * baseMVA;
        
    end
    
    % ===== CALCULATE LOSSES =====
    P_loss = sum(Pij + Pji);
    Q_loss = sum(Qij + Qji);
    
    % ===== CHECK CONSTRAINT VIOLATIONS =====
    violation_info = struct();
    violation_info.voltage_violation = [];
    violation_info.thermal_violation = [];
    violation_info.reactive_violation = [];
    violation_info.power_balance_violation = [];
    
    n_violations = 0;
    max_violation = 0;
    
    % Voltage limit violations
    V_min = bus(:, 13);
    V_max = bus(:, 12);
    
    v_violation_lower = V < V_min;
    v_violation_upper = V > V_max;
    
    if any(v_violation_lower | v_violation_upper)
        violation_buses = find(v_violation_lower | v_violation_upper);
        violation_info.voltage_violation = violation_buses;
        
        for i = violation_buses'
            if V(i) < V_min(i)
                v_viol = V_min(i) - V(i);
            else
                v_viol = V(i) - V_max(i);
            end
            n_violations = n_violations + 1;
            max_violation = max(max_violation, v_viol);
        end
    end
    
    % Thermal limit violations (line rating A)
    S_magnitude = sqrt(Pij.^2 + Qij.^2);
    rateA = branch(:, 6);
    
    % Only check lines with defined limits
    has_limit = rateA > 0;
    thermal_violation = find((S_magnitude > rateA) & has_limit);
    
    if ~isempty(thermal_violation)
        violation_info.thermal_violation = thermal_violation;
        
        for k = thermal_violation'
            thermal_viol = S_magnitude(k) - rateA(k);
            n_violations = n_violations + 1;
            max_violation = max(max_violation, thermal_viol);
        end
    end
    
    % Reactive power limit violations
    Q_gen_max = gen(:, 4);
    Q_gen_min = gen(:, 5);
    
    for i = 1:n_gen
        gen_bus = gen(i, 1);
        gen_bus_idx = bus_idx_map(gen_bus);
        
        Q_actual = gen(i, 3);  % Current reactive power
        
        if Q_actual > Q_gen_max(i)
            violation_info.reactive_violation = [violation_info.reactive_violation; i];
            n_violations = n_violations + 1;
            max_violation = max(max_violation, Q_actual - Q_gen_max(i));
        elseif Q_actual < Q_gen_min(i)
            violation_info.reactive_violation = [violation_info.reactive_violation; i];
            n_violations = n_violations + 1;
            max_violation = max(max_violation, Q_gen_min(i) - Q_actual);
        end
    end
    
    violation_info.n_violations = n_violations;
    violation_info.max_violation = max_violation;
    
    % ===== PREPARE OUTPUT =====
    P_flow = struct();
    P_flow.P_inj = P_inj;
    P_flow.Q_inj = Q_inj;
    P_flow.Pij = Pij;
    P_flow.Pji = Pji;
    P_flow.Qij = Qij;
    P_flow.Qji = Qji;
    P_flow.loss_P = P_loss;
    P_flow.loss_Q = Q_loss;
    P_flow.loss_S = sqrt(P_loss^2 + Q_loss^2);
    
    Q_flow = struct();
    Q_flow.Q_inj = Q_inj;
    Q_flow.Q_load = bus(:, 4) * baseMVA;
    Q_flow.Q_gen = gen(:, 3);
    Q_flow.Q_shunt = bus(:, 6) * baseMVA;
    
    % ===== DISPLAY RESULTS =====
    display_power_flow_results(P_flow, Q_flow, violation_info, n_bus, n_branch);
    
end


function display_power_flow_results(P_flow, Q_flow, violation_info, n_bus, n_branch)
% DISPLAY_POWER_FLOW_RESULTS Display power flow calculation results

    fprintf('\n--- Bus Power Injections ---\n');
    fprintf('Average Active Power Injection: %.2f MW\n', mean(abs(P_flow.P_inj)));
    fprintf('Average Reactive Power Injection: %.2f MVar\n', mean(abs(Q_flow.Q_inj)));
    
    fprintf('\n--- Branch Power Flows ---\n');
    fprintf('Total Active Power Loss: %.4f MW\n', P_flow.loss_P);
    fprintf('Total Reactive Power Loss: %.4f MVar\n', P_flow.loss_Q);
    fprintf('Total Power Loss (S): %.4f MVA\n', P_flow.loss_S);
    
    fprintf('\n--- Constraint Violations ---\n');
    fprintf('Total violations: %d\n', violation_info.n_violations);
    fprintf('Max violation magnitude: %.6f\n', violation_info.max_violation);
    
    if ~isempty(violation_info.voltage_violation)
        fprintf('Buses with voltage violations: %s\n', ...
                num2str(violation_info.voltage_violation'));
    end
    
    if ~isempty(violation_info.thermal_violation)
        fprintf('Branches with thermal violations: %s\n', ...
                num2str(violation_info.thermal_violation'));
    end
    
    if ~isempty(violation_info.reactive_violation)
        fprintf('Generators with reactive violations: %s\n', ...
                num2str(violation_info.reactive_violation'));
    end
    
    fprintf('\n==============================================\n');
    
end
