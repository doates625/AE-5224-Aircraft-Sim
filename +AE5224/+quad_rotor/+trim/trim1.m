function log = trim1(t_max, del_t)
%log = TRIM1(t_max, del_t)
%   Simulate trim1 condition
%   
%   Inputs:
%   - t_max = Sim duration [s, def = 10.0]
%   - del_t = Sim timestep [s, def = 0.01]
%   
%   Trim conditions:
%   - Altitude h* = 100m

% Imports
import('AE5224.quad_rotor.trim.test');
import('AE5224.Trim');

% Default args
if nargin < 1, t_max = 10.0; end
if nargin < 2, del_t = 0.01; end

% Test trim condition
trim = Trim(0, inf, 0, 100);
log = test(trim, t_max, del_t);

end