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
p_a = [0.0; 0.0; -0.1];
h_a = 0;
p_b = [0.5; 0.5; -0.1];
h_b = 0;

% Test path
test(p_a, h_a, p_b, h_b);

end