function path2()
%PATH1() Test Dubins Path 2
%   
%   Parameters:
%   - p(0) = [0.50, 0.50, -0.10] [km]
%   - h(0) = 0 deg
%   - p(T) = [0.75, 0.75, -0.10] [km]
%   - h(T) = 90 deg

% Imports
import('AE5224.quad_rotor.plan.test');

% Parameters
pa = 1000 * [0.50; 0.50];
ha = deg2rad(0);
pb = 1000 * [0.75; 0.75];
hb = deg2rad(90);

% Test path
test(pa, ha, pb, hb);

end