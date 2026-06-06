function mpc = load_case_data(case_name)
% LOAD_CASE_DATA Load power system test case data
%
% Syntax:
%   mpc = load_case_data(case_name)
%
% Description:
%   Loads MATPOWER case data and augments it with:
%   - LTC transformer data
%   - Shunt capacitor data
%   - Network parameters
%
% Input:
%   case_name - Name of the test case (e.g., 'ieee14bus_data')
%
% Output:
%   mpc - MATPOWER case structure with additional fields
%
% Author: Successive Linear OPF Team
% Date: 2026

    switch lower(case_name)
        
        case 'ieee14bus_data'
            % IEEE 14-bus test case with LTC and capacitors
            mpc = case14();  % Load base case from MATPOWER
            
            % Add LTC transformer data
            % Columns: [from_bus, to_bus, n_taps, min_tap, max_tap, nominal_tap]
            mpc.ltc = [
                4  7  11  0.900  1.100  1.000;  % Transformer 4-7 with 11 tap positions
            ];
            
            % Add shunt capacitor data
            % Columns: [bus, Qc (MVar)]
            mpc.shunt = [
                9   0.190;  % Bus 9: 0.19 MVar capacitor
                14  0.100;  % Bus 14: 0.10 MVar capacitor
            ];
            
        case 'ieee30bus_data'
            % IEEE 30-bus test case with LTC and capacitors
            mpc = case30();
            
            % Add LTC transformer data
            mpc.ltc = [
                6  31  11  0.900  1.100  1.000;
                10 20  11  0.900  1.100  1.000;
            ];
            
            % Add shunt capacitor data
            mpc.shunt = [
                10  0.190;
                24  0.043;
                29  0.080;
            ];
            
        case 'ieee57bus_data'
            % IEEE 57-bus test case
            mpc = case57();
            
            % Add LTC transformer data
            mpc.ltc = [
                4  18  11  0.900  1.100  1.000;
                8  32  11  0.900  1.100  1.000;
                21 20  11  0.900  1.100  1.000;
            ];
            
            % Add shunt capacitor data
            mpc.shunt = [
                3   0.100;
                25  0.060;
                53  0.060;
            ];
            
        otherwise
            error('Unknown case name: %s', case_name);
            
    end
    
    % Ensure required fields exist
    if ~isfield(mpc, 'ltc')
        mpc.ltc = [];
    end
    
    if ~isfield(mpc, 'shunt')
        mpc.shunt = [];
    end
    
    % Display case information
    fprintf('Case loaded: %s\n', case_name);
    fprintf('  Buses: %d, Generators: %d, Branches: %d\n', ...
            size(mpc.bus, 1), size(mpc.gen, 1), size(mpc.branch, 1));
    
end


function mpc = case14()
% IEEE 14-bus test case (simplified)

    mpc.version = '2';
    mpc.baseMVA = 100;
    
    % Bus data
    mpc.bus = [
        1  3  0.00  0.00  0.0  0.0  1  1.060  0.00  1.060  0.95  1.05  3;
        2  2  21.7 12.7  0.0  0.0  1  1.045  -4.98  1.045  0.95  1.05  3;
        3  2  94.2 19.0  0.0  0.0  1  1.010  -12.72 1.010  0.95  1.05  3;
        4  1  47.8 -3.9  0.0  0.0  1  1.019  -10.33 1.019  0.95  1.05  3;
        5  1  7.6  1.6  0.0  0.0  1  1.020  -8.78  1.020  0.95  1.05  3;
        6  2  11.2 7.5  0.0  0.0  1  1.070  -14.22 1.070  0.95  1.05  3;
        7  1  0.0  0.0  0.0  0.0  1  1.062  -13.37 1.062  0.95  1.05  3;
        8  2  0.0  0.0  0.0  0.0  1  1.090  -13.36 1.090  0.95  1.05  3;
        9  1  29.5 16.6  0.19 0.0  1  1.056  -14.94 1.056  0.95  1.05  3;
        10 1  9.0  5.8  0.0  0.0  1  1.051  -15.97 1.051  0.95  1.05  3;
        11 1  3.5  1.8  0.0  0.0  1  1.057  -14.79 1.057  0.95  1.05  3;
        12 1  6.1  1.6  0.0  0.0  1  1.055  -15.07 1.055  0.95  1.05  3;
        13 1  13.5 5.8  0.0  0.0  1  1.050  -15.16 1.050  0.95  1.05  3;
        14 1  14.9 5.0  0.1  0.0  1  1.036  -16.04 1.036  0.95  1.05  3;
    ];
    
    % Generator data
    mpc.gen = [
        1  232.4  -16.9  0.0  300.0  -300.0  1  100  300  50  0  0  0  0  0;
        2  40.0   43.6  0.0  100.0  -100.0  1  100  100  50  0  0  0  0  0;
        3  0.0    25.3  0.0  80.0   -80.0   1  100  80   50  0  0  0  0  0;
        6  0.0    12.2  0.0  120.0  -120.0  1  100  120  50  0  0  0  0  0;
        8  0.0    17.4  0.0  150.0  -150.0  1  100  150  50  0  0  0  0  0;
    ];
    
    % Branch data
    mpc.branch = [
        1  2  0.01938 0.05917 0.0528  0.0  0.0  0.0  0.0  0.0  1  0  0;
        1  5  0.05403 0.22304 0.0492  0.0  0.0  0.0  0.0  0.0  1  0  0;
        2  3  0.04699 0.19797 0.0438  0.0  0.0  0.0  0.0  0.0  1  0  0;
        2  4  0.05811 0.17632 0.0340  0.0  0.0  0.0  0.0  0.0  1  0  0;
        2  5  0.05695 0.17388 0.0346  0.0  0.0  0.0  0.0  0.0  1  0  0;
        3  4  0.06701 0.17103 0.0128  0.0  0.0  0.0  0.0  0.0  1  0  0;
        4  5  0.01335 0.04211 0.0000  0.0  0.0  0.0  0.0  0.0  1  0  0;
        4  7  0.00000 0.20912 0.0000  0.0  0.0  0.0  0.978  0.0  1  0  0;
        4  9  0.00000 0.55618 0.0000  0.0  0.0  0.0  0.969  0.0  1  0  0;
        5  6  0.01387 0.04623 0.0000  0.0  0.0  0.0  0.0  0.0  1  0  0;
        6  11 0.09498 0.19890 0.0000  0.0  0.0  0.0  0.0  0.0  1  0  0;
        6  12 0.12291 0.25581 0.0000  0.0  0.0  0.0  0.0  0.0  1  0  0;
        6  13 0.06615 0.13027 0.0000  0.0  0.0  0.0  0.0  0.0  1  0  0;
        7  9  0.00000 0.11001 0.0000  0.0  0.0  0.0  0.0  0.0  1  0  0;
        9  10 0.03181 0.08450 0.0000  0.0  0.0  0.0  0.0  0.0  1  0  0;
        9  14 0.12711 0.27038 0.0000  0.0  0.0  0.0  0.0  0.0  1  0  0;
        10 11 0.08205 0.19207 0.0000  0.0  0.0  0.0  0.0  0.0  1  0  0;
        12 13 0.22092 0.19988 0.0000  0.0  0.0  0.0  0.0  0.0  1  0  0;
        13 14 0.17093 0.34802 0.0000  0.0  0.0  0.0  0.0  0.0  1  0  0;
    ];
    
end


function mpc = case30()
% IEEE 30-bus test case (stub - would be loaded from MATPOWER)
    % This is a placeholder - in practice, call matpower directly
    mpc = struct();
    mpc.version = '2';
    mpc.baseMVA = 100;
    mpc.bus = [];
    mpc.gen = [];
    mpc.branch = [];
    warning('case30 is a stub. Load from MATPOWER using case30() directly.');
end


function mpc = case57()
% IEEE 57-bus test case (stub - would be loaded from MATPOWER)
    mpc = struct();
    mpc.version = '2';
    mpc.baseMVA = 100;
    mpc.bus = [];
    mpc.gen = [];
    mpc.branch = [];
    warning('case57 is a stub. Load from MATPOWER using case57() directly.');
end
