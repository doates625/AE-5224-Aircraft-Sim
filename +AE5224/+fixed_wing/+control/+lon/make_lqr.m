function [x_lon_st, u_lon_st, K] = make_lqr(model, x_st, u_st, verbose)
%[x_lon_st, u_lat_st, K] = MAKE_LQR(mode, x_st, u_st, verbose)
%   Make LQR for lon flight controller
%   
%   Inputs:
%   - model = Aircraft model [AE5224.fixed_wing.Model]
%   - x_st = Trim states [p_e; q_e; v_e; w_b]
%   - u_st = Trim controls [d_e; d_a; d_r; d_p]
%   - verbose = Print flag [logical, def = true]
%   
%   Outputs:
%   - x_lon_st = Trim lon state vector
%   - u_lon_st = Trim lon input vector
%   - K = Lon feedback matrix

% Imports
import('AE5224.fixed_wing.control.lon.lin_model');
import('AE5224.fixed_wing.control.lon.u_to_ulon');
import('AE5224.fixed_wing.control.make_lqr');

% Default args
if nargin < 4, verbose = true; end

% Max deviations
dx_lon_max = [
    0.2;    % Velocity body x [m/s]
    0.2;    % Velocity Body z [m/s]
    0.1;    % Pitch rate [rad/s]
    0.1;    % Pitch angle [rad]
    10.0;   % Altitude [m]
];

% Linearization and LQR
[A, B, x_lon_st, u_lon_st] = lin_model(model, x_st, u_st, verbose);
u_lon_min = u_to_ulon(model.u_min);
u_lon_max = u_to_ulon(model.u_max);
K = make_lqr(A, B, dx_lon_max, u_lon_st, u_lon_min, u_lon_max, verbose);

end