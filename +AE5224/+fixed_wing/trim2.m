function log = trim2(t_max, del_t)
%log = TRIM2(t_max, del_t)
%   Simulate trim2 condition
%   
%   Trim conditions:
%   - Airspeed V* = 30m/s
%   - Turn radius R* = 150m
%   - Altitude h* = 100m
%   
%   Inputs:
%   - t_max = Sim duration [s, def = 30.0]
%   - del_t = Sim timestep [s, def = 0.01]

% Imports
import('AE5224.Trim');
import('AE5224.sim_trim');

% Default args
if nargin < 1, t_max = 30.0; end
if nargin < 2, del_t = 0.01; end

% Trim simulation
trim = Trim(30, 150, 0, 100);
log = sim_trim('Fixedwing', trim, t_max, del_t);

end