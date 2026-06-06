function verify_success = verify_solution(mpc, solution, P_flow, Q_flow, violation_info)
% VERIFY_SOLUTION Verify OPF solution feasibility and optimality
%
% Syntax:
%   verify_success = verify_solution(mpc, solution, P_flow, Q_flow, violation_info)
%
% Description:
%   Comprehensive verification of the OPF solution including:
%   - Power balance equations
%   - Constraint satisfaction
%   - Physical feasibility
%   - Numerical consistency
%
% Author: Successive Linear OPF Team
% Date: 2026

    fprintf('\n');
    fprintf('════════════════════════════════════════════════════════════\n');
    fprintf('          SOLUTION VERIFICATION AND FEASIBILITY CHECK\n');
    fprintf('════════════════════════════════════════════════════════════\n\n');
    
    verify_success = true;
    
    % ===== CHECK 1: POWER BALANCE =====
    fprintf('CHECK 1: Power Balance Equations\n');
    fprintf('─────────────────────────────────\n');
    
    bus = mpc.bus;
    gen = mpc.gen;
    n_bus = size(bus, 1);
    n_gen = size(gen, 1);
    baseMVA = mpc.baseMVA;
    
    % Create bus number to index mapping
    bus_idx_map = containers.Map(bus(:, 1), 1:n_bus);
    
    % Check active power balance
    P_balance_error = zeros(n_bus, 1);
    Q_balance_error = zeros(n_bus, 1);
    
    for i = 1:n_bus
        bus_num = bus(i, 1);
        
        % Generation
        gen_mask = gen(:, 1) == bus_num;
        P_gen = sum(gen(gen_mask, 2));
        Q_gen = sum(gen(gen_mask, 3));
        
        % Load
        P_load = bus(i, 3);
        Q_load = bus(i, 4);
        
        % Shunt
        G_shunt = bus(i, 5);
        B_shunt = bus(i, 6);
        V = solution.V(i);
        
        P_shunt = G_shunt * V^2 * baseMVA;
        Q_shunt = -B_shunt * V^2 * baseMVA;
        
        % Flow in/out
        P_flow_net = P_flow.P_inj(i);
        Q_flow_net = Q_flow.Q_inj(i);
        
        % Balance equation: Pg - Pd - Gsh*V^2 + Pflow = 0
        P_balance_error(i) = abs(P_gen - P_load - P_shunt + P_flow_net);
        Q_balance_error(i) = abs(Q_gen - Q_load - Q_shunt + Q_flow_net);
    end
    
    max_P_error = max(P_balance_error);
    max_Q_error = max(Q_balance_error);
    
    fprintf('Max active power balance error:   %.2e MW\n', max_P_error);
    fprintf('Max reactive power balance error: %.2e MVar\n', max_Q_error);
    
    if max_P_error > 1e-3 || max_Q_error > 1e-3
        fprintf('⚠  WARNING: Large power balance errors detected\n');
        verify_success = false;
    else
        fprintf('✓  Power balance satisfied\n');
    end
    
    % ===== CHECK 2: VOLTAGE LIMITS =====
    fprintf('\nCHECK 2: Voltage Magnitude Limits\n');
    fprintf('─────────────────────────────────\n');
    
    V_min = bus(:, 13);
    V_max = bus(:, 12);
    
    v_violation = (solution.V < V_min) | (solution.V > V_max);
    n_v_violations = sum(v_violation);
    
    fprintf('Buses with voltage violations: %d\n', n_v_violations);
    
    if n_v_violations > 0
        fprintf('⚠  WARNING: %d buses violate voltage limits\n', n_v_violations);
        verify_success = false;
    else
        fprintf('✓  All voltages within limits\n');
    end
    
    % ===== CHECK 3: THERMAL LIMITS =====
    fprintf('\nCHECK 3: Thermal Limits (Line Rating)\n');
    fprintf('────────────────────────────────────\n');
    
    branch = mpc.branch;
    n_branch = size(branch, 1);
    rateA = branch(:, 6);
    
    S_magnitude = sqrt(P_flow.Pij.^2 + P_flow.Qij.^2);
    
    has_limit = rateA > 0;
    thermal_violation = find((S_magnitude > rateA) & has_limit);
    n_t_violations = length(thermal_violation);
    
    fprintf('Branches with thermal violations: %d\n', n_t_violations);
    
    if n_t_violations > 0
        fprintf('⚠  WARNING: %d branches exceed thermal limits\n', n_t_violations);
        verify_success = false;
    else
        fprintf('✓  All branches within thermal limits\n');
    end
    
    % ===== CHECK 4: GENERATOR LIMITS =====
    fprintf('\nCHECK 4: Generator Operating Limits\n');
    fprintf('───────────────────────────────────\n');
    
    P_gen_min = gen(:, 10);
    P_gen_max = gen(:, 9);
    Q_gen_min = gen(:, 5);
    Q_gen_max = gen(:, 4);
    
    P_gen_actual = gen(:, 2);
    Q_gen_actual = gen(:, 3);
    
    P_violation = (P_gen_actual < P_gen_min - 1e-3) | (P_gen_actual > P_gen_max + 1e-3);
    Q_violation = (Q_gen_actual < Q_gen_min - 1e-3) | (Q_gen_actual > Q_gen_max + 1e-3);
    
    n_p_violations = sum(P_violation);
    n_q_violations = sum(Q_violation);
    
    fprintf('Generators violating P limits: %d\n', n_p_violations);
    fprintf('Generators violating Q limits: %d\n', n_q_violations);
    
    if (n_p_violations + n_q_violations) > 0
        fprintf('⚠  WARNING: Generator limits violated\n');
        verify_success = false;
    else
        fprintf('✓  All generators within limits\n');
    end
    
    % ===== CHECK 5: NUMERICAL CONSISTENCY =====
    fprintf('\nCHECK 5: Numerical Consistency\n');
    fprintf('──────────────────────────────\n');
    
    % Check for NaN or Inf values
    has_nan = any(isnan(solution.V)) || any(isnan(solution.theta)) || ...
              any(isnan(P_flow.Pij)) || any(isnan(P_flow.Qij));
    
    has_inf = any(isinf(solution.V)) || any(isinf(solution.theta)) || ...
              any(isinf(P_flow.Pij)) || any(isinf(P_flow.Qij));
    
    if has_nan || has_inf
        fprintf('⚠  WARNING: NaN or Inf values detected in solution\n');
        verify_success = false;
    else
        fprintf('✓  No NaN or Inf values\n');
    end
    
    % Check voltage range reasonableness
    if min(solution.V) < 0.5 || max(solution.V) > 1.5
        fprintf('⚠  WARNING: Unreasonable voltage magnitudes\n');
        verify_success = false;
    else
        fprintf('✓  Voltage magnitudes in reasonable range\n');
    end
    
    % ===== CHECK 6: OPTIMALITY INDICATORS =====
    fprintf('\nCHECK 6: Optimality Indicators\n');
    fprintf('──────────────────────────────\n');
    
    total_cost = sum(P_flow.loss_P);
    
    fprintf('Total system loss:     %.4f MW\n', P_flow.loss_P);
    fprintf('Total constraint violations: %d\n', violation_info.n_violations);
    fprintf('Max constraint violation:   %.2e\n', violation_info.max_violation);
    
    % ===== FINAL VERDICT =====
    fprintf('\n════════════════════════════════════════════════════════════\n');
    
    if verify_success
        fprintf('                    ✓ SOLUTION IS FEASIBLE\n');
    else
        fprintf('                   ⚠ SOLUTION HAS INFEASIBILITIES\n');
    end
    
    fprintf('════════════════════════════════════════════════════════════\n\n');
    
end
