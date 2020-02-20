function log = trim1(open_loop, sim_wind, ekf_fb, t_max, del_t)
%log = TRIM1(open_loop, sim_wind, ekf_fb, t_max, del_t)
%   Simulate trim1 condition
%   
%   Inputs:
%   - open_loop = Open loop control flag [logical]
%   - sim_wind = Wind simulation flag [logical]
%   - ekf_fb = EKF feedback flag [logical]
%   - t_max = Sim duration [s, def = 10.0]
%   - del_t = Sim timestep [s, def = 0.01]
%   
%   Trim conditions:
%   - Airspeed V* = 30m/s
%   - Altitude h* = 100m

% Imports
import('AE5224.fixed_wing.trim.test');
import('AE5224.Trim');

% Default args
if nargin < 4, t_max = 10.0; end
if nargin < 5, del_t = 0.01; end

% Trim test
trim = Trim(30, inf, 0, 100);
log = test(trim, open_loop, sim_wind, ekf_fb, t_max, del_t);

end