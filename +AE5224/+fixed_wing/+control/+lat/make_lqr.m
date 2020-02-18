function [x_lat_st, u_lat_st, K] = make_lqr(model, x_st, u_st)
%[x_lat_st, u_lat_st, K] = MAKE_LQR(mode, x_st, u_st)
%   Make LQR for lat flight controller
%   
%   Inputs:
%   - model = Aircraft model [AE5224.fixed_wing.Model]
%   - x_st = Trim states [p_e; q_e; v_e; w_b]
%   - u_st = Trim controls [d_e; d_a; d_r; d_p]
%   
%   Outputs:
%   - x_lat_st = Trim lat state vector
%   - u_lat_st = Trim lat input vector
%   - K = Lat feedback matrix

% Imports
import('AE5224.fixed_wing.control.lat.lin_model');
import('AE5224.fixed_wing.control.lat.u_to_ulat');
import('AE5224.fixed_wing.control.make_lqr');

% Max deviations
dx_lat_max = [
    3.0;    % Velocity Body y [m/s]
    1.0;    % Roll rate [rad/s]
    0.5;    % Yaw rate [rad/s]
    0.1;    % Roll angle [rad]
    0.1;    % Yaw angle [rad]
];

% Linearization and LQR
[A, B, x_lat_st, u_lat_st] = lin_model(model, x_st, u_st);
u_lat_min = u_to_ulat(model.u_min);
u_lat_max = u_to_ulat(model.u_max);
K = make_lqr(A, B, dx_lat_max, u_lat_st, u_lat_min, u_lat_max);

end