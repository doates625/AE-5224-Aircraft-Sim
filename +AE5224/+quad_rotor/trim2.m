function log = trim2(t_max, del_t)
%log = TRIM2(t_max, del_t)
%   Simulate trim2 condition
%   
%   Trim conditions:
%   - Altitude h* = 100m
%   - Airspeed V* = 10m/s
%   
%   Inputs:
%   - t_max = Sim duration [s, def = 10.0]
%   - del_t = Sim timestep [s, def = 0.01]

% Imports
import('AE5224.Trim');
import('AE5224.sim_trim');

% Default args
if nargin < 1, t_max = 10.0; end
if nargin < 2, del_t = 0.01; end

% Trim simulation
trim = Trim(10, inf, 0, 100);
log = sim_trim('Quadrotor', trim, t_max, del_t);

end