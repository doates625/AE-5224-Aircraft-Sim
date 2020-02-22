function path1()
%PATH1() Test Dubins Path 1
%   
%   Parameters:
%   - p(0) = [0, 0, -0.1] [km]
%   - h(0) = 0 deg
%   - p(T) = [1, 1, -0.1] [km]
%   - h(T) = 0 deg

% Printout
clc
fprintf('Fixed-Wing Dubins Path 1\n\n');

% Imports
import('AE5224.fixed_wing.plan.test');

% Parameters
pa = 1000 * [0; 0];
ha = deg2rad(0);
pb = 1000 * [1; 1];
hb = deg2rad(0);

% Test path
test(pa, ha, pb, hb);

end