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
p_a = [0.50; 0.50; -0.10];
h_a = 0;
p_b = [0.75; 0.75; -0.10];
h_b = 90;

% Test path
test(p_a, h_a, p_b, h_b);

end