% Test Script for Dubins Paths
clear, clc

% Imports
import('AE5224.dubins.path.LSL');
import('AE5224.dubins.path.LSR');
import('AE5224.dubins.path.RSL');
import('AE5224.dubins.path.RSR');

% Path conditions
pa = [0; 0];
ha = 0;
pb = 10*rand(2, 1)-5;
hb = pi*rand();
r = 1;

% Generate paths
fprintf('Generating Dubins paths...\n');
paths = {@LSL, @LSR, @RSL, @RSR};
names = {'LSL', 'LSR', 'RSL', 'RSR'};
for i = 1:4
    fig = figure(i); clf;
    try
        path = paths{i}(pa, ha, pb, hb, r);
        cls = class(path);
        fig.Name = cls(end-2:end);
        path.plot(fig);
        status = 'Generated';
    catch
        status = 'Failed';
    end
    fprintf('%s %s path.\n', status, names{i});
end