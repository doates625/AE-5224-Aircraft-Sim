function log = path1(ic)
%PATH1(ic)
%   Test Dubins Path 1
%   
%   Inputs:
%   - ic = Initial condition ID [1...3]
%   
%   Constants:
%   - p(0) = [0, 0, -0.1] [km]
%   - h(0) = 0 deg
%   - p(T) = [1, 1, -0.1] [km]
%   - h(T) = 0 deg

% Imports
import('AE5224.fixed_wing.plan.test');

% Parameters
p_a = [0.0; 0.0; -0.10];
h_a = deg2rad(0);
p_b = [1.0; 1.0; -0.10];
h_b = deg2rad(0);

% Initial conditions
switch ic
    case 1, p_i = [0.1; 0.0; -0.11]; h_i = 00;
    case 2, p_i = [0.5; 0.5; -0.09]; h_i = 35;
    case 3, p_i = [0.1; 0.5; -0.10]; h_i = 90;
end

% Test path
log = test(p_a, h_a, p_b, h_b, p_i, h_i);

end