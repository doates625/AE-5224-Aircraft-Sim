function log = trim3(t_max, del_t)
%log = TRIM3(t_max, del_t)
%   Simulate trim3 condition
%   
%   Inputs:
%   - t_max = Sim duration [s, def = 120.0]
%   - del_t = Sim timestep [s, def = 0.01]
%   
%   Trim conditions:
%   - Airspeed V* = 30m/s
%   - Turn radius R* = 150m
%   - Climb angle gam* = 3deg
%   - Init altitude h = 100m

% Imports
import('AE5224.trim.Trim');
import('AE5224.fixed_wing.trim.test');

% Default args
if nargin < 1, t_max = 120.0; end
if nargin < 2, del_t = 0.01; end

% Trim test
trim = Trim(30, 150, deg2rad(3), 100);
log = test(trim, t_max, del_t);

end