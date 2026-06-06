%% SUCCESSIVE LINEAR APPROXIMATION OPF - MAIN PROGRAM
%
% This is the main entry point for the Successive Linear Approximation
% based Optimal Power Flow (SLA-OPF) solver.
%
% Reference:
%   Yang, L., et al. "Optimal Reactive Power Dispatch With Accurately 
%   Modeled Discrete Control Devices: A Successive Linear Approximation 
%   Approach." IEEE Transactions on Power Systems, vol. 32, no. 3, 
%   May 2017, pp. 2437-2447.
%
% Author: Successive Linear OPF Team
% Date: 2026

clear; close all; clc;

% Add paths for all modules
addpath('./core');
addpath('./validation');
addpath('./utils');
addpath('./data');

fprintf('\n');
fprintf('╔════════════════════════════════════════════════════════════╗\n');
fprintf('║  Successive Linear Approximation OPF Solver                ║\n');
fprintf('║  IEEE Transactions on Power Systems, Vol. 32, No. 3, 2017 ║\n');
fprintf('╚════════════════════════════════════════════════════════════╝\n');

% =========================================================================
% SECTION 1: LOAD CASE DATA
% =========================================================================

fprintf('\n>>> SECTION 1: LOADING POWER SYSTEM CASE\n');
fprintf('═══════════════════════════════════════════════════\n');

% Specify test case
case_name = 'ieee14bus_data';
fprintf('Loading case: %s\n', case_name);

% Load case data
mpc = load_case_data(case_name);

% Store case information
n_bus = size(mpc.bus, 1);
n_gen = size(mpc.gen, 1);
n_branch = size(mpc.branch, 1);
n_ltc = size(mpc.ltc, 1);
n_shunt = size(mpc.shunt, 1);

fprintf('\nCase Summary:\n');
fprintf('  Buses:        %d\n', n_bus);
fprintf('  Generators:   %d\n', n_gen);
fprintf('  Branches:     %d\n', n_branch);
fprintf('  LTC Trans.:   %d\n', n_ltc);
fprintf('  Shunt Caps:   %d\n', n_shunt);

% =========================================================================
% SECTION 2: CONFIGURE OPTIMIZATION OPTIONS
% =========================================================================

fprintf('\n>>> SECTION 2: CONFIGURING OPTIMIZATION OPTIONS\n');
fprintf('═══════════════════════════════════════════════════\n');

% Set optimization parameters
options = struct();
options.tol_convergence = 1e-2;      % Convergence tolerance (MW)
options.max_iterations = 20;          % Maximum iterations
options.verbose = true;               % Display iteration details
options.solver = 'intlinprog';        % Use intlinprog for MILP
options.objective = 'loss';           % Minimize active power loss

fprintf('Optimization Parameters:\n');
fprintf('  Convergence tolerance: %.1e MW\n', options.tol_convergence);
fprintf('  Maximum iterations:    %d\n', options.max_iterations);
fprintf('  Solver:                %s\n', options.solver);
fprintf('  Objective:             Minimize %s\n', options.objective);

% =========================================================================
% SECTION 3: INITIALIZE SOLUTION
% =========================================================================

fprintf('\n>>> SECTION 3: INITIALIZING SOLUTION\n');
fprintf('═══════════════════════════════════════════════════\n');

% Initial point (flat start)
V0 = mpc.bus(:, 8);                    % Initial voltage magnitude
theta0 = mpc.bus(:, 9) * pi/180;       % Initial voltage angle (convert to radians)

fprintf('Initial Solution (Flat Start):\n');
fprintf('  Voltage range: [%.4f, %.4f] p.u.\n', min(V0), max(V0));
fprintf('  Angle range:   [%.4f, %.4f] degrees\n', ...
        min(theta0)*180/pi, max(theta0)*180/pi);

% =========================================================================
% SECTION 4: BUILD ADMITTANCE MATRIX
% =========================================================================

fprintf('\n>>> SECTION 4: BUILDING ADMITTANCE MATRIX\n');
fprintf('═══════════════════════════════════════════════════\n');

[Ybus, Yf, Yt] = build_admittance_matrix(mpc);

% =========================================================================
% SECTION 5: LINEARIZE POWER FLOW EQUATIONS
% =========================================================================

fprintf('\n>>> SECTION 5: LINEARIZING POWER FLOW EQUATIONS\n');
fprintf('═══════════════════════════════════════════════════\n');

[Pij_lin, Qij_lin] = linearize_pf_equations(Ybus, V0, theta0, mpc.branch, mpc.baseMVA);

% =========================================================================
% SECTION 6: BUILD DISCRETE DEVICE MODELS
% =========================================================================

fprintf('\n>>> SECTION 6: BUILDING DISCRETE CONTROL DEVICE MODELS\n');
fprintf('═══════════════════════════════════════════════════\n');

% Build LTC model
[A_ltc, b_ltc_lb, b_ltc_ub, z_ltc, x_ltc] = build_ltc_model(mpc, V0);

% Build capacitor model
[A_cap, b_cap_lb, b_cap_ub, z_cap, x_cap] = build_capacitor_model(mpc, V0);

% =========================================================================
% SECTION 7: SOLVE OPTIMIZATION PROBLEM
% =========================================================================

fprintf('\n>>> SECTION 7: SOLVING OPF\n');
fprintf('═══════════════════════════════════════════════════\n');

[solution, iter_info] = solve_opf_model(mpc, options);

% =========================================================================
% SECTION 8: VALIDATE SOLUTION WITH AC POWER FLOW
% =========================================================================

fprintf('\n>>> SECTION 8: AC POWER FLOW VALIDATION\n');
fprintf('═══════════════════════════════════════════════════\n');

[P_flow, Q_flow, violation_info] = ac_power_flow(mpc, solution.V, solution.theta, Ybus);

% =========================================================================
% SECTION 9: DISPLAY FINAL RESULTS
% =========================================================================

fprintf('\n>>> SECTION 9: FINAL RESULTS\n');
fprintf('═══════════════════════════════════════════════════\n');

fprintf('\n╔═ CONVERGENCE INFORMATION ═════════════════════════════════╗\n');
fprintf('║ Convergence:           %-5s                              ║\n', ...
        num2str(solution.convergence));
fprintf('║ Number of iterations:  %-5d                              ║\n', ...
        iter_info.n_iterations);
fprintf('║ Computation time:      %-10.2f seconds                ║\n', ...
        iter_info.time_elapsed);
fprintf('╚═══════════════════════════════════════════════════════════╝\n');

fprintf('\n╔═ SYSTEM LOSSES ═══════════════════════════════════════════════╗\n');
fprintf('║ Active power loss (P):   %-10.4f MW                   ║\n', ...
        P_flow.loss_P);
fprintf('║ Reactive power loss (Q): %-10.4f MVar                  ║\n', ...
        P_flow.loss_Q);
fprintf('║ Total power loss (S):    %-10.4f MVA                  ║\n', ...
        P_flow.loss_S);
fprintf('╚═══════════════════════════════════════════════════════════════╝\n');

fprintf('\n╔═ VOLTAGE STATISTICS ══════════════════════════════════════╗\n');
fprintf('║ Min voltage:  %-8.4f p.u. (bus %d)                   ║\n', ...
        min(solution.V), find(solution.V == min(solution.V)));
fprintf('║ Max voltage:  %-8.4f p.u. (bus %d)                   ║\n', ...
        max(solution.V), find(solution.V == max(solution.V)));
fprintf('║ Avg voltage:  %-8.4f p.u.                            ║\n', ...
        mean(solution.V));
fprintf('╚═══════════════════════════════════════════════════════════╝\n');

fprintf('\n╔═ CONSTRAINT VIOLATIONS ═══════════════════════════════════╗\n');
fprintf('║ Total violations:     %-5d                            ║\n', ...
        violation_info.n_violations);
fprintf('║ Max violation:        %-10.6f                    ║\n', ...
        violation_info.max_violation);

if ~isempty(violation_info.voltage_violation)
    fprintf('║ Voltage violations:   %-5d buses                    ║\n', ...
            length(violation_info.voltage_violation));
end

if ~isempty(violation_info.thermal_violation)
    fprintf('║ Thermal violations:   %-5d branches                 ║\n', ...
            length(violation_info.thermal_violation));
end

if ~isempty(violation_info.reactive_violation)
    fprintf('║ Reactive violations:  %-5d generators                ║\n', ...
            length(violation_info.reactive_violation));
end

fprintf('╚═══════════════════════════════════════════════════════════╝\n');

% =========================================================================
% SECTION 10: CONVERGENCE HISTORY PLOT
% =========================================================================

fprintf('\n>>> SECTION 10: PLOTTING CONVERGENCE HISTORY\n');
fprintf('═══════════════════════════════════════════════════\n');

if length(iter_info.convergence_history) > 1
    figure('Name', 'Convergence History', 'NumberTitle', 'off');
    semilogy(1:length(iter_info.convergence_history), ...
             iter_info.convergence_history, 'b-o', 'LineWidth', 2, 'MarkerSize', 8);
    xlabel('Iteration', 'FontSize', 12);
    ylabel('System Loss (MW)', 'FontSize', 12);
    title('SLA-OPF Convergence History', 'FontSize', 14, 'FontWeight', 'bold');
    grid on;
    xlim([1 length(iter_info.convergence_history)]);
    
    fprintf('Convergence plot saved.\n');
end

% =========================================================================
% SECTION 11: DETAILED SOLUTION SUMMARY
% =========================================================================

fprintf('\n>>> SECTION 11: DETAILED SOLUTION SUMMARY\n');
fprintf('═══════════════════════════════════════════════════\n');

fprintf('\n╔═ BUS VOLTAGES AND ANGLES ══════════════════════════════════╗\n');
fprintf('║ Bus │ Voltage(p.u.) │ Angle(deg) │ Load_P(MW) │ Load_Q(MVar) ║\n');
fprintf('╠═════╪═══════════════╪════════════╪════════════╪══════════════╣\n');

for i = 1:min(n_bus, 14)  % Display first 14 buses
    bus_num = mpc.bus(i, 1);
    P_load = mpc.bus(i, 3);
    Q_load = mpc.bus(i, 4);
    fprintf('║ %3d │    %8.4f    │  %8.2f  │  %8.2f  │   %8.2f    ║\n', ...
            bus_num, solution.V(i), solution.theta(i)*180/pi, P_load, Q_load);
end

if n_bus > 14
    fprintf('║ ... │      ...      │    ...     │    ...     │     ...      ║\n');
end

fprintf('╚═════╧═══════════════╧════════════╧════════════╧══════════════╝\n');

fprintf('\n╔═ GENERATOR DISPATCH ══════════════════════════════════════╗\n');
fprintf('║ Gen │ Bus │  Pg(MW)  │  Qg(MVar) │  Qmax  │  Qmin  ║\n');
fprintf('╠═════╪═════╪══════════╪═══════════╪════════╪════════╣\n');

for i = 1:n_gen
    gen_bus = mpc.gen(i, 1);
    Pg = mpc.gen(i, 2);
    Qg = mpc.gen(i, 3);
    Qmax = mpc.gen(i, 4);
    Qmin = mpc.gen(i, 5);
    fprintf('║ %3d │ %3d │  %7.2f │  %7.2f  │ %6.2f │ %6.2f ║\n', ...
            i, gen_bus, Pg, Qg, Qmax, Qmin);
end

fprintf('╚═════╧═════╧══════════╧═══════════╧════════╧════════╝\n');

fprintf('\n╔═ BRANCH POWER FLOWS ══════════════════════════════════════╗\n');
fprintf('║ Line │ From │ To │  P_ij(MW) │  Q_ij(MVar) │  Loss(MW)  ║\n');
fprintf('╠══════╪══════╪════╪═══════════╪═════════════╪════════════╣\n');

for k = 1:min(n_branch, 20)  % Display first 20 branches
    fbus = mpc.branch(k, 1);
    tbus = mpc.branch(k, 2);
    P_loss_k = P_flow.Pij(k) + P_flow.Pji(k);
    fprintf('║ %4d │ %4d │ %2d │  %7.2f  │   %7.2f   │  %7.4f  ║\n', ...
            k, fbus, tbus, P_flow.Pij(k), P_flow.Qij(k), P_loss_k);
end

if n_branch > 20
    fprintf('║ ...  │ ...  │ .. │   ...    │    ...     │   ...     ║\n');
end

fprintf('╚══════╧══════╧════╧═══════════╧═════════════╧════════════╝\n');

% =========================================================================
% SECTION 12: EXPORT RESULTS
% =========================================================================

fprintf('\n>>> SECTION 12: EXPORTING RESULTS\n');
fprintf('═══════════════════════════════════════════════════\n');

% Save results to file
results_filename = sprintf('results_%s_%s.mat', case_name, datestr(now, 'yyyymmdd_HHMMSS'));
save(results_filename, 'mpc', 'solution', 'P_flow', 'Q_flow', 'violation_info', 'iter_info');
fprintf('Results saved to: %s\n', results_filename);

% =========================================================================
% FINAL SUMMARY
% =========================================================================

fprintf('\n');
fprintf('╔════════════════════════════════════════════════════════════╗\n');
fprintf('║           ✓ OPTIMIZATION COMPLETED SUCCESSFULLY            ║\n');
fprintf('╚════════════════════════════════════════════════════════════╝\n');
fprintf('\n');

% Display any warnings or recommendations
if violation_info.n_violations > 0
    fprintf('⚠  WARNING: Solution has %d constraint violations\n', ...
            violation_info.n_violations);
    fprintf('   Please review the violation details above.\n');
else
    fprintf('✓  All constraints satisfied. Solution is feasible.\n');
end

if ~solution.convergence
    fprintf('⚠  WARNING: Algorithm did not converge\n');
    fprintf('   Consider increasing max_iterations or adjusting tolerance.\n');
else
    fprintf('✓  Algorithm converged in %d iterations\n', iter_info.n_iterations);
end

fprintf('\n');
