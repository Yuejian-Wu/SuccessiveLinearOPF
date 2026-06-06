function mpc = load_case_data(case_name)
% LOAD_CASE_DATA Load power system case data
%
% Syntax:
%   mpc = load_case_data(case_name)
%
% Description:
%   Loads power system case data in MATPOWER format.
%   Supports both built-in IEEE test systems and custom data files.
%
% Input:
%   case_name - Name of the test case or path to data file (string)
%              Examples: 'ieee14bus_data', 'ieee30bus_data', 'custom_case'
%
% Output:
%   mpc - MATPOWER case structure containing:
%         .baseMVA - Base power
%         .bus - Bus data matrix
%         .gen - Generator data matrix
%         .branch - Branch/line data matrix
%         .shunt - Shunt capacitor data (optional)
%         .ltc - Load tap changer data (optional)
%         .gencost - Generator cost coefficients (optional)
%
% Notes:
%   - Data structure is independent of specific test systems
%   - Compatible with various IEEE benchmark systems
%   - Supports extension for custom data formats
%
% Author: Successive Linear OPF Team
% Date: 2026

    narginchk(1, 1);
    
    % Remove file extension if provided
    if endsWith(case_name, '.m')
        case_name = case_name(1:end-2);
    end
    
    % Try to load the case
    try
        % First, try to call as a function (built-in cases)
        if exist(case_name, 'file') == 2
            % Function or file exists
            mpc = feval(case_name);
        else
            error('Case not found: %s', case_name);
        end
    catch ME
        error('Error loading case "%s": %s', case_name, ME.message);
    end
    
    % Validate and normalize the case structure
    mpc = validate_case_structure(mpc);
    
    % Display case information
    display_case_info(mpc);
    
end


function mpc = validate_case_structure(mpc)
% VALIDATE_CASE_STRUCTURE Validate and normalize MATPOWER case structure
%
% Ensures the case has all required fields with proper dimensions

    % Required fields
    required_fields = {'baseMVA', 'bus', 'gen', 'branch'};
    
    for i = 1:length(required_fields)
        field = required_fields{i};
        if ~isfield(mpc, field)
            error('Case structure missing required field: %s', field);
        end
    end
    
    % Extract dimensions
    n_bus = size(mpc.bus, 1);
    n_gen = size(mpc.gen, 1);
    n_branch = size(mpc.branch, 1);
    
    % Validate bus data structure
    % Expected: bus_i, type, Pd, Qd, Gs, Bs, area, Vm, Va, baseKV, zone, Vmax, Vmin
    if size(mpc.bus, 2) < 13
        warning('Bus matrix has fewer than 13 columns. Some data may be missing.');
    end
    
    % Validate generator data structure
    % Expected: bus, Pg, Qg, Qmax, Qmin, Vg, mBase, status, Pmax, Pmin, ...
    if size(mpc.gen, 2) < 10
        warning('Generator matrix has fewer than 10 columns.');
    end
    
    % Validate branch data structure
    % Expected: fbus, tbus, r, x, b, rateA, rateB, rateC, ratio, angle, status, ...
    if size(mpc.branch, 2) < 13
        warning('Branch matrix has fewer than 13 columns.');
    end
    
    % Optional fields: shunt, ltc, gencost
    if ~isfield(mpc, 'shunt')
        mpc.shunt = [];
    end
    
    if ~isfield(mpc, 'ltc')
        mpc.ltc = [];
    end
    
    if ~isfield(mpc, 'gencost')
        mpc.gencost = [];
    end
    
    % Validate bus numbers are unique
    bus_numbers = mpc.bus(:, 1);
    if length(unique(bus_numbers)) ~= length(bus_numbers)
        error('Duplicate bus numbers detected in case data.');
    end
    
    % Validate generator bus numbers
    gen_buses = mpc.gen(:, 1);
    if any(~ismember(gen_buses, bus_numbers))
        error('Generator connected to non-existent bus.');
    end
    
    % Validate branch connections
    branch_from = mpc.branch(:, 1);
    branch_to = mpc.branch(:, 2);
    if any(~ismember(branch_from, bus_numbers)) || any(~ismember(branch_to, bus_numbers))
        error('Branch connected to non-existent bus.');
    end
    
end


function display_case_info(mpc)
% DISPLAY_CASE_INFO Display information about the loaded case

    n_bus = size(mpc.bus, 1);
    n_gen = size(mpc.gen, 1);
    n_branch = size(mpc.branch, 1);
    n_shunt = size(mpc.shunt, 1);
    n_ltc = size(mpc.ltc, 1);
    
    total_load_p = sum(mpc.bus(:, 3));
    total_load_q = sum(mpc.bus(:, 4));
    
    fprintf('\n========== Power System Case Information ==========\n');
    fprintf('Base Power: %.2f MVA\n', mpc.baseMVA);
    fprintf('Number of Buses: %d\n', n_bus);
    fprintf('Number of Generators: %d\n', n_gen);
    fprintf('Number of Branches: %d\n', n_branch);
    fprintf('Number of Shunt Capacitors: %d\n', n_shunt);
    fprintf('Number of LTC Transformers: %d\n', n_ltc);
    fprintf('\nTotal Load: P = %.2f MW, Q = %.2f MVar\n', total_load_p, total_load_q);
    fprintf('====================================================\n\n');
    
end
