function [A_lat, B_lat, x_lat, u_lat] = model(body, x_st, u_st)
%[A_lat, B_lat, x_lat, u_lat] = MODEL(body, x_st, u_st)
%   Linearized lateral (lat) flight model
%   
%   Inputs:
%   - body = Fixed-wing model [AE5224.fixed_wing.Body]
%   - x_st = Trim state vector [p_e; q_e; v_e; w_b]
%   - u_st = Trim control vector [d_e; d_a; d_r; d_p]
%   
%   Outputs:
%   - A_lat = Linearized lat state matrix
%   - B_lat = Linearized lat input matrix
%   - x_lat = Trim lat state vector
%   - u_lat = Trim lat control vector
%   
%   Lat state x_lat:
%   - v_by = Velocity body y [m/s]
%   - w_bx = Roll rate [rad/s]
%   - w_bz = Yaw rate [rad/s]
%   - th_x = Roll angle [rad]
%   - th_z = Yaw angle [rad]
%   
%   Lat input u_lat:
%   - d_a = Aileron angle [rad]
%   - d_r = Rudder angle [rad]
%   
%   Author: Dan Oates (WPI Class of 2020)

% Imports
import('AE5224.fixed_wing.control.lat.x_to_xlat');
import('AE5224.fixed_wing.control.lat.xlat_to_x');
import('AE5224.fixed_wing.control.lat.u_to_ulat');
import('AE5224.fixed_wing.control.lat.ulat_to_u');
import('AE5224.fixed_wing.control.sub_model');

% Numerical linearization
[A_lat, B_lat, x_lat, u_lat] = sub_model(...
    body, x_st, u_st, ...
    @x_to_xlat, @xlat_to_x, ...
    @u_to_ulat, @ulat_to_u);

% Correct known elements
A_lat(4:5, 1) = 0;
A_lat(4, 2) = 1;
A_lat(5, 2) = 0;
A_lat(2:3, 4) = 0;
A_lat(:, 5) = 0;
B_lat(4:5, :) = 0;

end