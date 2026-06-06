% README - Successive Linear Approximation OPF Solver
%
% OVERVIEW
% ========
% This repository implements a Successive Linear Approximation (SLA) based
% Optimal Power Flow (OPF) solver with accurate modeling of discrete control
% devices including Load Tap Changer (LTC) transformers and switchable shunt
% capacitors.
%
% Reference Paper:
%   Yang, L., et al. "Optimal Reactive Power Dispatch With Accurately Modeled 
%   Discrete Control Devices: A Successive Linear Approximation Approach."
%   IEEE Transactions on Power Systems, vol. 32, no. 3, May 2017, pp. 2437-2447.
%
% PROJECT STRUCTURE
% =================
%
% SuccessiveLinearOPF/
% ├── core/                          Core optimization modules
% │   ├── build_ltc_model.m         LTC transformer constraint building
% │   ├── build_capacitor_model.m   Shunt capacitor constraint building
% │   └── solve_opf_model.m         Main three-loop iterative solver
% │
% ├── validation/                    Solution verification modules
% │   ├── ac_power_flow.m           AC power flow calculation
% │   └── verify_solution.m         Solution feasibility checker
% │
% ├── utils/                         Utility functions
% │   ├── build_admittance_matrix.m Ybus construction
% │   ├── plot_results.m            Result visualization
% │   └── export_results.m          Report generation
% │
% ├── data/                          Test case data
% │   └── load_case_data.m          IEEE test case loader
% │
% ├── main_SLA_OPF.m                Main program entry point
% └── README.txt                     This file
%
% QUICK START
% ===========
%
% 1. MATLAB Setup:
%    - Ensure MATPOWER is installed and on MATLAB path
%    - Add all directories to MATLAB path:
%      >> addpath('./core');
%      >> addpath('./validation');
%      >> addpath('./utils');
%      >> addpath('./data');
%
% 2. Run the solver:
%    >> main_SLA_OPF
%
%    This will:
%    - Load the IEEE 14-bus test case
%    - Execute the three-loop OPF algorithm
%    - Validate the solution with AC power flow
%    - Generate plots and export results
%
% KEY ALGORITHMS
% ==============
%
% Three-Loop Iterative Algorithm:
%
%   Loop 1: Solve continuous ORPD with fixed discrete variables
%   ─────────────────────────────────────────────────────────
%   - Linearize AC power flow around current operating point
%   - Formulate mixed-integer linear program (MILP)
%   - Solve for continuous variables (power, voltage)
%   - Obtain high-quality initial solution
%
%   Loop 2: Update discrete control variables
%   ──────────────────────────────────────────
%   - Extract fractional tap and capacitor values from Loop 1
%   - Round to nearest discrete position
%   - Heuristic rounding with monotonicity enforcement
%   - Maintain feasibility constraints
%
%   Loop 3: Verify AC feasibility
%   ────────────────────────────
%   - Run exact AC power flow on updated solution
%   - Check all constraint violations
%   - Recover feasible solution if needed
%   - Assess solution quality
%
% Convergence:
%   Algorithm converges when system loss change < tolerance (default: 1e-2 MW)
%   Maximum iterations: 20 (default)
%
% DISCRETE CONTROL DEVICES
% =========================
%
% 1. Load Tap Changer (LTC) Transformers
%    ─────────────────────────────────────
%    - Modeled as continuous tap ratio in [min_tap, max_tap]
%    - Discrete positions enforced via binary variables
%    - Tap range typically 0.9 to 1.1 pu with 11 positions
%    - Constraints:
%      * Voltage transformation: V_i = t_m * V_j + ΔV_m
%      * Only one tap position active: Σz_m = 1
%      * Monotonicity: z_m ≥ z_{m+1}
%
% 2. Shunt Capacitors
%    ─────────────────
%    - Reactive power injection Q_cap = B_cap * V^2
%    - Switchable with discrete MVar values
%    - Constraints:
%      * Reactive power bounds: 0 ≤ Q_cap ≤ Q_max
%      * Voltage dependency: Q_cap ≤ B * V_max^2
%      * Binary selection: 0 ≤ x_cap ≤ 1
%      * Monotonicity: x_i ≥ x_{i+1}
%
% INPUT DATA FORMAT (MATPOWER Case Structure)
% ============================================
%
% mpc.bus:
%   [bus_i type Pd Qd Gs Bs area Vm Va baseKV zone Vmax Vmin]
%   - bus_i: Bus number (1 to n_bus)
%   - type: 1=PQ, 2=PV, 3=Slack
%   - Vm: Voltage magnitude (p.u.)
%   - Va: Voltage angle (degrees)
%   - Vmin, Vmax: Voltage limits (p.u.)
%
% mpc.gen:
%   [bus Pg Qg Qmax Qmin Vg mBase status Pmax Pmin]
%   - bus: Generator bus number
%   - Pg: Active power setpoint (MW)
%   - Qg: Reactive power setpoint (MVar)
%   - Qmin, Qmax: Reactive power limits (MVar)
%   - Pmin, Pmax: Active power limits (MW)
%
% mpc.branch:
%   [fbus tbus r x b rateA rateB rateC ratio angle status]
%   - fbus, tbus: From and to bus
%   - r, x: Series resistance and reactance (p.u.)
%   - b: Shunt susceptance (p.u.)
%   - ratio: Transformer tap ratio (0 = line)
%   - rateA: Branch thermal limit (MVA)
%
% mpc.ltc (User-defined):
%   [from_bus to_bus n_taps min_tap max_tap nominal_tap]
%   - n_taps: Number of discrete tap positions
%   - min_tap, max_tap: Tap range (p.u.)
%
% mpc.shunt (User-defined):
%   [bus Qc_MVar]
%   - Qc_MVar: Capacitor reactive power (MVar)
%
% OUTPUT DATA
% ===========
%
% solution structure:
%   .V: Bus voltage magnitudes (p.u.)
%   .theta: Bus voltage angles (radians)
%   .Pg: Generator active power (MW)
%   .Qg: Generator reactive power (MVar)
%   .convergence: Boolean flag
%   .loss: Total system loss (MW)
%
% P_flow structure:
%   .Pij: Branch active power flow (MW)
%   .Qij: Branch reactive power flow (MVar)
%   .loss_P: Total active power loss (MW)
%   .loss_Q: Total reactive power loss (MVar)
%   .loss_S: Total apparent power loss (MVA)
%
% violation_info structure:
%   .voltage_violation: Buses exceeding voltage limits
%   .thermal_violation: Branches exceeding thermal limits
%   .reactive_violation: Generators exceeding reactive limits
%   .n_violations: Total number of violations
%   .max_violation: Maximum violation magnitude
%
% iter_info structure:
%   .n_iterations: Number of iterations to convergence
%   .convergence_history: Loss values per iteration
%   .time_elapsed: Total computation time (seconds)
%
% CONFIGURATION OPTIONS
% =====================
%
% options.tol_convergence:  Convergence tolerance (default: 1e-2 MW)
% options.max_iterations:   Maximum iterations (default: 20)
% options.verbose:          Print details (default: true)
% options.solver:           MILP solver: 'linprog' or 'intlinprog' (default: 'intlinprog')
% options.objective:        Objective: 'loss' or 'cost' (default: 'loss')
%
% Example:
%   options.tol_convergence = 1e-3;
%   options.max_iterations = 30;
%   [solution, iter_info] = solve_opf_model(mpc, options);
%
% VALIDATION AND VERIFICATION
% =============================
%
% The solver includes comprehensive validation:
%
% 1. AC Power Flow Calculation:
%    - Exact nonlinear power flow on converged solution
%    - Detects linearization errors
%    - Computes actual branch flows and losses
%
% 2. Constraint Violation Checking:
%    - Voltage magnitude limits (Vmin ≤ V ≤ Vmax)
%    - Thermal limits (S_flow ≤ rateA)
%    - Generator reactive limits (Qmin ≤ Qg ≤ Qmax)
%    - Power balance equations
%
% 3. Numerical Consistency:
%    - Checks for NaN/Inf values
%    - Validates voltage magnitude range
%    - Ensures power balance
%
% RESULTS AND REPORTING
% =====================
%
% The solver generates:
%
% 1. Console Output:
%    - Real-time iteration progress
%    - Convergence status
%    - Violation summary
%    - Computation time
%
% 2. Visualization (plot_results.m):
%    - Bus voltage magnitudes and angles
%    - Branch power flows
%    - Generator reactive power dispatch
%    - System losses
%
% 3. Text Report (export_results.m):
%    - Case information
%    - Convergence history
%    - Detailed bus/generator/branch data
%    - Constraint violation list
%
% 4. MATLAB Variables:
%    - Saved in results_*.mat file
%    - Contains all case data, solution, and metrics
%
% TEST CASES
% ==========
%
% Included test cases (via load_case_data.m):
%
% - ieee14bus_data:  IEEE 14-bus system
%   * 14 buses, 5 generators, 20 branches
%   * 1 LTC transformer, 2 shunt capacitors
%   * Small system for quick testing
%
% - ieee30bus_data:  IEEE 30-bus system (stubs)
%   * 30 buses, 6 generators
%   * Requires MATPOWER case30.m
%
% - ieee57bus_data:  IEEE 57-bus system (stubs)
%   * 57 buses, 7 generators
%   * Requires MATPOWER case57.m
%
% IMPLEMENTATION NOTES
% ====================
%
% 1. Power Flow Linearization:
%    - Linear approximation around flat start (V0=1.0, θ0=0)
%    - Updated at each iteration for better accuracy
%    - Validated against AC power flow
%
% 2. Binary Variable Formulation:
%    - z_m ∈ {0,1}: Tap/capacitor position selection
%    - Ordered binary variables enforce monotonicity
%    - Single-selection constraint: Σz_m = 1
%
% 3. Feasibility Recovery:
%    - AC power flow validation in Loop 3
%    - Violation-based feasibility restoration
%    - Prioritizes constraint satisfaction
%
% 4. Computational Performance:
%    - Sparse matrix representation (Ybus)
%    - MILP solver: intlinprog (MATLAB native)
%    - Typical convergence: 5-10 iterations
%    - Runtime: <1 second for IEEE 14-bus (typical)
%
% TROUBLESHOOTING
% ===============
%
% Issue: "Unknown case name" error
% → Solution: Use 'ieee14bus_data', 'ieee30bus_data', or 'ieee57bus_data'
%
% Issue: Algorithm doesn't converge
% → Increase max_iterations: options.max_iterations = 50
% → Relax tolerance: options.tol_convergence = 5e-2
%
% Issue: NaN values in solution
% → Check case data for invalid parameters
% → Ensure transformer tap ratios are in (0, 2)
%
% Issue: Large constraint violations
% → Increase linearization accuracy (more frequent updates)
% → Check numerical conditioning of admittance matrix
%
% REFERENCES
% ==========
%
% [1] Yang, L., et al. "Optimal Reactive Power Dispatch With Accurately 
%     Modeled Discrete Control Devices: A Successive Linear Approximation 
%     Approach." IEEE Transactions on Power Systems, vol. 32, no. 3, 
%     May 2017, pp. 2437-2447.
%
% [2] Zimmerman, R. D., et al. "MATPOWER: Steady-State Operations, Planning,
%     and Analysis Tools for Power Systems Research and Education."
%     IEEE Trans. Power Syst., vol. 26, no. 1, pp. 12-19, Feb 2011.
%
% [3] Boyd, S., & Parikh, N. "Distributed Optimization and Statistical 
%     Learning via the Alternating Direction Method of Multipliers."
%     Found. Trends Mach. Learning, vol. 3, no. 1, pp. 1-122, Jan 2011.
%
% CONTACT & SUPPORT
% =================
%
% For issues or questions:
% 1. Check this README first
% 2. Review code documentation in each .m file
% 3. Verify MATPOWER installation
% 4. Check GitHub issues: https://github.com/Yuejian-Wu/SuccessiveLinearOPF
%
% AUTHOR
% ======
% Successive Linear OPF Development Team
% Date: 2026
%
% LICENSE
% =======
% [Specify your license here, e.g., MIT, GPL, etc.]
%
% ========================================================================
