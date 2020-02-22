function path1()
%PATH1() Test Dubins Path 1
%   
%   Parameters:
%   - p(0) = [0.0, 0.0, -0.1] [km]
%   - h(0) = 0 deg
%   - p(T) = [0.5, 0.5, -0.1] [km]
%   - h(T) = 0 deg

% Imports
import('AE5224.quad_rotor.plan.test');

% Parameters
pa = 1000 * [0.0; 0.0];
ha = deg2rad(0);
pb = 1000 * [0.5; 0.5];
hb = deg2rad(0);

% Test path
test(pa, ha, pb, hb);

end