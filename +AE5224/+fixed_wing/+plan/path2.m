function log = path2()
%PATH1() Test Dubins Path 2
%   
%   Parameters:
%   - p(0) = [1, 1, -0.1] [km]
%   - h(0) = 0 deg
%   - p(T) = [1.15, 1, -0.1] [km]
%   - h(T) = 90 deg

% Imports
import('AE5224.fixed_wing.plan.test');

% Parameters
p_a = [1.00; 1.00; -0.10];
h_a = 0;
p_b = [1.25; 1.00; -0.10];
h_b = 90;
p_i = p_a;
h_i = h_a;

% Test path
log = test(p_a, h_a, p_b, h_b, p_i, h_i);

end