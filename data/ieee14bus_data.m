function mpc = ieee14bus_data()
% IEEE 14-Bus Test System Data
% 
% Data format compatible with MATPOWER
% Reference: IEEE PES Power System Test Case Archive
%
% This case represents a typical small to medium-sized power system
% with 14 buses, 20 lines, 5 generators, and 11 loads.

mpc.version = '2';
mpc.baseMVA = 100;

%% bus data
% bus_i type Pd Qd Gs Bs area Vm Va baseKV zone Vmax Vmin
mpc.bus = [
    1   3   0    0    0   0   1   1.060   0     345   1   1.06   0.94
    2   2   21.7 12.7 0   0   1   1.045  -4.98  345   1   1.06   0.94
    3   2   94.2 19   0   0   1   1.010 -12.72  345   1   1.06   0.94
    4   1   47.8 -3.9 0   0   1   1.019 -10.33  345   1   1.06   0.94
    5   1   7.6  1.6  0   0   1   1.020  -8.78  345   1   1.06   0.94
    6   2   11.2 7.5  0   0   1   1.070 -14.22  345   1   1.06   0.94
    7   1   0    0    0   0   1   1.062 -13.37  345   1   1.06   0.94
    8   2   0    0    0   0.19 1   1.090 -13.36  345   1   1.06   0.94
    9   1   29.5 16.6 0   0.19 1   1.056 -14.94  345   1   1.06   0.94
    10  1   9    5.8  0   0   1   1.051 -15.10  345   1   1.06   0.94
    11  1   3.5  1.8  0   0   1   1.057 -14.79  345   1   1.06   0.94
    12  1   6.1  1.6  0   0   1   1.055 -15.07  345   1   1.06   0.94
    13  1   13.5 5.8  0   0   1   1.050 -15.16  345   1   1.06   0.94
    14  1   14.9 5    0   0   1   1.036 -16.04  345   1   1.06   0.94
];

%% generator data
% bus Pg Qg Qmax Qmin Vg mBase status Pmax Pmin Pc1 Pc2 Qc1max Qc1min Qc2max Qc2min ramp_agc ramp_10 ramp_30 ramp_q apf
mpc.gen = [
    1   232.4  -16.9   40    -40    1.060  100  1  250   10   0   0   0   0   0   0   0   0   0   0   0
    2   40     42.4    50    -40    1.045  100  1  150   10   0   0   0   0   0   0   0   0   0   0   0
    3   0      23.4    40     0     1.010  100  1  100   10   0   0   0   0   0   0   0   0   0   0   0
    6   0      12.2    24    -6     1.070  100  1  100   10   0   0   0   0   0   0   0   0   0   0   0
    8   0      17.4    24    -6     1.090  100  1  100   10   0   0   0   0   0   0   0   0   0   0   0
];

%% branch data
% fbus tbus r x b rateA rateB rateC ratio angle status angmin angmax
mpc.branch = [
    1   2   0.01938  0.05917  0.0528   0     0     0    0      0   1   -360   360
    1   5   0.05403  0.22304  0.0492   0     0     0    0      0   1   -360   360
    2   3   0.04699  0.19797  0.0438   0     0     0    0      0   1   -360   360
    2   4   0.05811  0.17632  0.0374   0     0     0    0      0   1   -360   360
    2   5   0.05695  0.17388  0.0340   0     0     0    0      0   1   -360   360
    3   4   0.06701  0.17103  0.0128   0     0     0    0      0   1   -360   360
    4   5   0.01335  0.04211  0.0       0     0     0    0      0   1   -360   360
    4   7   0.0       0.20912  0.0     0.978  0     0  0.978   0   1   -360   360
    4   9   0.0       0.55618  0.0     0.969  0     0  0.969   0   1   -360   360
    5   6   0.0       0.25202  0.0     0.932  0     0  0.932   0   1   -360   360
    6   11  0.09498  0.1989   0.0       0     0     0    0      0   1   -360   360
    6   12  0.12291  0.25581  0.0       0     0     0    0      0   1   -360   360
    6   13  0.06615  0.13027  0.0       0     0     0    0      0   1   -360   360
    7   8   0.0       0.17615  0.0       0     0     0    0      0   1   -360   360
    7   9   0.0       0.11001  0.0       0     0     0    0      0   1   -360   360
    9   10  0.03181  0.0845   0.0       0     0     0    0      0   1   -360   360
    9   14  0.12711  0.27038  0.0       0     0     0    0      0   1   -360   360
    10  11  0.08205  0.19207  0.0       0     0     0    0      0   1   -360   360
    12  13  0.22092  0.19988  0.0       0     0     0    0      0   1   -360   360
    13  14  0.17093  0.34802  0.0       0     0     0    0      0   1   -360   360
];

%% Shunt capacitor data
% bus_id capacity_mvar_1 capacity_mvar_2 ... (可选多个电容器)
% 格式: [bus_id, num_capacitors, capacitor_1_value, capacitor_2_value, ...]
mpc.shunt = [
    9   1   0.19
    14  1   0.10
];

%% LTC (Load Tap Changer) transformer data
% 格式: [from_bus, to_bus, num_taps, min_tap, max_tap, nominal_tap, current_tap]
% tap ratio = 1 + (tap - nominal_tap) * step_size
mpc.ltc = [
    4   7   33   0.9   1.1   1.0   1.0
    4   9   33   0.9   1.1   1.0   1.0
    5   6   33   0.9   1.1   1.0   1.0
];

%% Generator cost data
% type startup shutdown n ... coefficients ...
% For quadratic cost: n=3, [c2 c1 c0]
mpc.gencost = [
    2   0   0   3   0.01124  5   150
    2   0   0   3   0.01124  5   150
    2   0   0   3   0.01124  5   150
    2   0   0   3   0.01124  5   150
    2   0   0   3   0.01124  5   150
];

end
