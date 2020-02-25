% Setup workspace
clear, clc
import('AE5224.quad_rotor.control.TrajPlan');

% Trajectory Plan
p_mat = [...
    [+0.0000e+0, +9.1440e+0, -1.6764e+1];
    [+1.4469e+1, +3.8769e+0, +7.8507e-1];
    [-1.0127e-1, -4.1309e+0, +3.1597e+0];
    [-1.1841e-4, +1.1727e+0, -1.0059e+0];
    [-7.6013e-4, -1.6510e-1, +1.2597e-1];
    [-3.4094e-3, +1.2217e-2, -6.9039e-3];
    [+2.9767e-4, -3.8280e-4, +1.2789e-4];
].';
plan = TrajPlan(p_mat);

% Plot
t_min = 0.00;
t_max = 7.25;
plan.plot(t_min, t_max);