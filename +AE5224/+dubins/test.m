% Test Script for Dubins Paths
clear, clc

% Imports
import('AE5224.dubins.path.RSR');
import('AE5224.dubins.path.LSL');

% Path conditions
pa = [0; 0]; ha = 0;
pb = 10*rand(2, 1)-5; hb = pi*rand();
r = 2.0;

% Generate paths
rsr = RSR(pa, ha, pb, hb, r);
lsl = LSL(pa, ha, pb, hb, r);

% Plot paths
fig_rsr = figure(1); fig_rsr.Name = 'RSR'; rsr.plot(fig_rsr);
fig_lsl = figure(2); fig_lsl.Name = 'LSL'; rsr.plot(fig_lsl);