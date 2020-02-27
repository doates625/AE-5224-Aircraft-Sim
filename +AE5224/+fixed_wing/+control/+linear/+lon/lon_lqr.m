function [A, B, x_lon, u_lon, K] = lon_lqr(model, x_st, u_st)
%[x_lon_st, u_lat_st, K] = LON_LQR(mode, x_st, u_st)
%   Make LQR for lon flight controller
%   
%   Inputs:
%   - model = Aircraft model [AE5224.fixed_wing.Model]
%   - x_st = Trim states [p_e; q_e; v_e; w_b]
%   - u_st = Trim controls [d_e; d_a; d_r; d_p]
%   
%   Outputs:
%   - A = Lon state matrix
%   - B = Lon input matrix
%   - x_lat = Lon trim state
%   - u_lat = Lon trim input
%   - K = Lon gain matrix

% Imports
import('AE5224.fixed_wing.control.linear.lon.lon_model');
import('AE5224.fixed_wing.control.linear.lon.u_to_ulon');
import('AE5224.fixed_wing.control.linear.make_lqr');

% Max deviations
dx_lon_max = [
    0.2;    % Velocity body x [m/s]
    0.2;    % Velocity Body z [m/s]
    0.1;    % Pitch rate [rad/s]
    0.1;    % Pitch angle [rad]
    10.0;   % Altitude [m]
];

% Linearization and LQR
[A, B, x_lon, u_lon] = lon_model(model, x_st, u_st);
u_lon_min = u_to_ulon(model.u_min);
u_lon_max = u_to_ulon(model.u_max);
K = make_lqr(A, B, dx_lon_max, u_lon, u_lon_min, u_lon_max);

end