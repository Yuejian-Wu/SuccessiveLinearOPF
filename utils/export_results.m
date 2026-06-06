function export_results(filename, mpc, solution, P_flow, Q_flow, violation_info, iter_info)
% EXPORT_RESULTS Export OPF results to file
%
% Syntax:
%   export_results(filename, mpc, solution, P_flow, Q_flow, violation_info, iter_info)
%
% Description:
%   Exports comprehensive OPF results to a text file for reporting and analysis.
%   Includes:
%   - Case information
%   - Solution summary
%   - Bus data and voltages
%   - Generator dispatch
%   - Branch power flows
%   - Constraint violations
%   - Convergence history
%
% Author: Successive Linear OPF Team
% Date: 2026

    % Open file for writing
    fid = fopen(filename, 'w');
    
    if fid == -1
        error('Unable to open file: %s', filename);
    end
    
    % Get current date and time
    now_str = datestr(now, 'yyyy-mm-dd HH:MM:SS');
    
    % ===== HEADER =====
    fprintf(fid, '╔════════════════════════════════════════════════════════════╗\n');
    fprintf(fid, '║         SUCCESSIVE LINEAR APPROXIMATION OPF RESULTS         ║\n');
    fprintf(fid, '╚════════════════════════════════════════════════════════════╝\n\n');
    
    fprintf(fid, 'Report Generated: %s\n\n', now_str);
    
    % ===== CASE INFORMATION =====
    fprintf(fid, '═══════════════════════════════════════════════════════════════\n');
    fprintf(fid, '1. CASE INFORMATION\n');
    fprintf(fid, '═══════════════════════════════════════════════════════════════\n\n');
    
    fprintf(fid, 'Base Power:        %10.2f MVA\n', mpc.baseMVA);
    fprintf(fid, 'Number of Buses:   %10d\n', size(mpc.bus, 1));
    fprintf(fid, 'Number of Gens:    %10d\n', size(mpc.gen, 1));
    fprintf(fid, 'Number of Branches:%10d\n', size(mpc.branch, 1));
    fprintf(fid, 'LTC Transformers:  %10d\n', size(mpc.ltc, 1));
    fprintf(fid, 'Shunt Capacitors:  %10d\n\n', size(mpc.shunt, 1));
    
    % ===== CONVERGENCE INFORMATION =====
    fprintf(fid, '═══════════════════════════════════════════════════════════════\n');
    fprintf(fid, '2. CONVERGENCE INFORMATION\n');
    fprintf(fid, '═══════════════════════════════════════════════════════════════\n\n');
    
    fprintf(fid, 'Convergence Status:    %s\n', num2str(solution.convergence));
    fprintf(fid, 'Number of Iterations:  %d\n', iter_info.n_iterations);
    fprintf(fid, 'Computation Time:      %.2f seconds\n', iter_info.time_elapsed);
    fprintf(fid, 'Final System Loss:     %.6f MW\n\n', P_flow.loss_P);
    
    % ===== SYSTEM LOSSES =====
    fprintf(fid, '═══════════════════════════════════════════════════════════════\n');
    fprintf(fid, '3. SYSTEM LOSSES\n');
    fprintf(fid, '═══════════════════════════════════════════════════════════════\n\n');
    
    fprintf(fid, 'Active Power Loss (P):     %10.4f MW\n', P_flow.loss_P);
    fprintf(fid, 'Reactive Power Loss (Q):   %10.4f MVar\n', P_flow.loss_Q);
    fprintf(fid, 'Total Power Loss (S):      %10.4f MVA\n\n', P_flow.loss_S);
    
    % ===== CONSTRAINT VIOLATIONS =====
    fprintf(fid, '═══════════════════════════════════════════════════════════════\n');
    fprintf(fid, '4. CONSTRAINT VIOLATIONS\n');
    fprintf(fid, '═══════════════════════════════════════════════════════════════\n\n');
    
    fprintf(fid, 'Total Violations:          %d\n', violation_info.n_violations);
    fprintf(fid, 'Max Violation Magnitude:   %.2e\n', violation_info.max_violation);
    
    if ~isempty(violation_info.voltage_violation)
        fprintf(fid, '\nVoltage Violations (Buses): ');
        fprintf(fid, '%d ', violation_info.voltage_violation);
        fprintf(fid, '\n');
    end
    
    if ~isempty(violation_info.thermal_violation)
        fprintf(fid, 'Thermal Violations (Lines): ');
        fprintf(fid, '%d ', violation_info.thermal_violation);
        fprintf(fid, '\n');
    end
    
    if ~isempty(violation_info.reactive_violation)
        fprintf(fid, 'Reactive Violations (Gens): ');
        fprintf(fid, '%d ', violation_info.reactive_violation);
        fprintf(fid, '\n');
    end
    
    fprintf(fid, '\n');
    
    % ===== BUS DATA =====
    fprintf(fid, '═══════════════════════════════════════════════════════════════\n');
    fprintf(fid, '5. BUS VOLTAGE AND ANGLE DATA\n');
    fprintf(fid, '═══════════════════════════════════════════════════════════════\n\n');
    
    fprintf(fid, 'Bus │ Voltage(p.u.) │ Angle(deg) │ Load_P(MW) │ Load_Q(MVar)\n');
    fprintf(fid, '────┼───────────────┼────────────┼────────────┼──────────────\n');
    
    bus = mpc.bus;
    for i = 1:size(bus, 1)
        bus_num = bus(i, 1);
        P_load = bus(i, 3);
        Q_load = bus(i, 4);
        fprintf(fid, '%3d │    %8.4f    │  %8.2f  │  %8.2f  │   %8.2f\n', ...
                bus_num, solution.V(i), solution.theta(i)*180/pi, P_load, Q_load);
    end
    
    fprintf(fid, '\n');
    
    % ===== GENERATOR DATA =====
    fprintf(fid, '═══════════════════════════════════════════════════════════════\n');
    fprintf(fid, '6. GENERATOR DISPATCH\n');
    fprintf(fid, '═══════════════════════════════════════════════════════════════\n\n');
    
    fprintf(fid, 'Gen │ Bus │  Pg(MW)  │  Qg(MVar) │  Qmax  │  Qmin\n');
    fprintf(fid, '────┼─────┼──────────┼───────────┼────────┼────────\n');
    
    gen = mpc.gen;
    for i = 1:size(gen, 1)
        gen_bus = gen(i, 1);
        Pg = gen(i, 2);
        Qg = gen(i, 3);
        Qmax = gen(i, 4);
        Qmin = gen(i, 5);
        fprintf(fid, '%3d │ %3d │  %7.2f │  %7.2f  │ %6.2f │ %6.2f\n', ...
                i, gen_bus, Pg, Qg, Qmax, Qmin);
    end
    
    fprintf(fid, '\n');
    
    % ===== BRANCH DATA =====
    fprintf(fid, '═══════════════════════════════════════════════════════════════\n');
    fprintf(fid, '7. BRANCH POWER FLOWS\n');
    fprintf(fid, '═══════════════════════════════════════════════════════════════\n\n');
    
    fprintf(fid, 'Line │ From │ To │  P_ij(MW) │  Q_ij(MVar) │ Loss(MW)\n');
    fprintf(fid, '─────┼──────┼────┼───────────┼─────────────┼──────────\n');
    
    branch = mpc.branch;
    for k = 1:size(branch, 1)
        fbus = branch(k, 1);
        tbus = branch(k, 2);
        P_loss_k = P_flow.Pij(k) + P_flow.Pji(k);
        fprintf(fid, '%4d │ %4d │ %2d │  %7.2f  │   %7.2f   │  %7.4f\n', ...
                k, fbus, tbus, P_flow.Pij(k), P_flow.Qij(k), P_loss_k);
    end
    
    fprintf(fid, '\n');
    
    % ===== CONVERGENCE HISTORY =====
    if isfield(iter_info, 'convergence_history')
        fprintf(fid, '═══════════════════════════════════════════════════════════════\n');
        fprintf(fid, '8. CONVERGENCE HISTORY\n');
        fprintf(fid, '═══════════════════════════════════════════════════════════════\n\n');
        
        fprintf(fid, 'Iteration │ System Loss (MW)\n');
        fprintf(fid, '──────────┼─────────────────\n');
        
        for i = 1:length(iter_info.convergence_history)
            fprintf(fid, '%9d │    %12.6f\n', i, iter_info.convergence_history(i));
        end
        
        fprintf(fid, '\n');
    end
    
    % ===== FOOTER =====
    fprintf(fid, '═══════════════════════════════════════════════════════════════\n');
    fprintf(fid, 'End of Report\n');
    fprintf(fid, '═══════════════════════════════════════════════════════════════\n');
    
    fclose(fid);
    
    fprintf('Results exported to: %s\n', filename);
    
end
