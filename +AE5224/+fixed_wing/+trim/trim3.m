function log = trim3(open_loop, sim_wind, t_max, del_t)
%log = TRIM3(open_loop, sim_wind, t_max, del_t)
%   Simulate trim3 condition
%   
%   Inputs:
%   - open_loop = Open loop control flag [logical, def = true]
%   - sim_wind = Wind simulation flag [logical, def = false]
%   - t_max = Sim duration [s, def = 120.0]
%   - del_t = Sim timestep [s, def = 0.01]
%   
%   Trim conditions:
%   - Airspeed V* = 30m/s
%   - Turn radius R* = 150m
%   - Climb angle gam* = 3deg
%   - Init altitude h = 100m

% Imports
import('AE5224.fixed_wing.trim.test');
import('AE5224.Trim');

% Default args
if nargin < 1, open_loop = true; end
if nargin < 2, sim_wind = false; end
if nargin < 3, t_max = 120.0; end
if nargin < 4, del_t = 0.01; end

% Trim test
trim = Trim(30, 150, deg2rad(3), 100);
log = test(trim, open_loop, sim_wind, t_max, del_t);

end