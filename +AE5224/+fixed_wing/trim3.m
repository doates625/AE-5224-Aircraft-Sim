function log = trim3(t_max, del_t)
%log = TRIM3(t_max, del_t)
%   Simulate trim3 condition
%   
%   Trim conditions:
%   - Airspeed V* = 30m/s
%   - Turn radius R* = 150m
%   - Climb angle gam* = 3deg
%   - Init altitude h = 100m
%   
%   Inputs:
%   - t_max = Sim duration [s, def = 60.0]
%   - del_t = Sim timestep [s, def = 0.01]

% Imports
import('AE5224.trim.Trim');
import('AE5224.trim.sim');

% Default args
if nargin < 1, t_max = 60.0; end
if nargin < 2, del_t = 0.01; end

% Trim simulation
trim = Trim(30, 150, deg2rad(3), 100);
log = sim('Fixedwing', trim, t_max, del_t);

end