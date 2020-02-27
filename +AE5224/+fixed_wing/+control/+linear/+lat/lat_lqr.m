function [A, B, x_lat, u_lat, K] = lat_lqr(model, x_st, u_st)
%[A, B, x_lat, u_lat, K] = LAT_LQR(model, x_st, u_st)
%   Make LQR for lat flight controller
%   
%   Inputs:
%   - model = Aircraft model [AE5224.fixed_wing.Model]
%   - x_st = Trim states [p_e; q_e; v_e; w_b]
%   - u_st = Trim controls [d_e; d_a; d_r; d_p]
%   
%   Outputs:
%   - A = Lat state matrix
%   - B = Lat input matrix
%   - x_lat = Lat trim state
%   - u_lat = Lat trim input
%   - K = Lat gain matrix

% Imports
import('AE5224.fixed_wing.control.linear.lat.lat_model');
import('AE5224.fixed_wing.control.linear.lat.u_to_ulat');
import('AE5224.fixed_wing.control.linear.make_lqr');

% Max deviations
dx_lat_max = [
    0.2;    % Velocity Body y [m/s]
    1.0;    % Roll rate [rad/s]
    1.0;    % Yaw rate [rad/s]
    0.1;    % Roll angle [rad]
    0.1;    % Yaw angle [rad]
];

% Linearization and LQR
[A, B, x_lat, u_lat] = lat_model(model, x_st, u_st);
u_lat_min = u_to_ulat(model.u_min);
u_lat_max = u_to_ulat(model.u_max);
K = make_lqr(A, B, dx_lat_max, u_lat, u_lat_min, u_lat_max);

end