function log = trim2(open_loop, ekf_fb, t_max, del_t)
%log = TRIM2(open_loop, ekf_fb, t_max, del_t)
%   Simulate trim2 condition
%   
%   Inputs:
%   - open_loop = Open loop control flag [logical]
%   - ekf_fb = EKF feedback flag [logical]
%   - t_max = Sim duration [s, def = 10.0]
%   - del_t = Sim timestep [s, def = 0.01]
%   
%   Trim conditions:
%   - Altitude h* = 100m
%   - Airspeed V* = 10m/s

% Imports
import('AE5224.quad_rotor.trim.test');
import('AE5224.Trim');

% Default args
if nargin < 3, t_max = 10.0; end
if nargin < 4, del_t = 0.01; end

% Trim simulation
trim = Trim(10, inf, 0, 100);
log = test(trim, open_loop, ekf_fb, t_max, del_t);

end