% Test Script for Dubins Paths
clear, clc

% Imports
import('AE5224.dubins.RSR');
import('AE5224.dubins.LSL');

% Path conditions
pa = [0; 0]; ha = 0;
pb = 10*rand(2, 1)-5; hb = pi*rand();
r = 1;

% Generate paths
rsr = RSR(pa, ha, pb, hb, r);
lsl = LSL(pa, ha, pb, hb, r);

% Plot paths
rsr.plot(1); title(sprintf('RSR Path [d = %.2f]', rsr.dist()));
lsl.plot(2); title(sprintf('LSL Path [d = %.2f]', lsl.dist()));