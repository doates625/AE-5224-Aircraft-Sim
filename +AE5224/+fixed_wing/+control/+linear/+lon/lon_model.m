function [A_lon, B_lon, x_lon, u_lon] = lon_model(model, x_st, u_st)
%[A_lon, B_lon, x_lon, u_lon] = LON_MODEL(model, x_st, u_st)
%   Linearized longitudinal flight model
%   
%   Inputs:
%   - mode = Fixed-wing model [AE5224.fixed_wing.Body]
%   - x_st = Trim state vector [p_e; q_e; v_e; w_b]
%   - u_st = Trim control vector [d_e; d_a; d_r; d_p]
%   
%   Outputs:
%   - A_lon = Linearized lon state matrix
%   - B_lon = Linearized lon input matrix
%   - x_lon = Trim lon state vector
%   - u_lon = Trim lon control vector
%   
%   Lon state x_lon:
%   - v_bx = Velocity Body x [m/s]
%   - v_bz = Velocity Body z [m/s]
%   - w_by = Pitch rate [rad/s]
%   - th_y = Pitch angle [rad]
%   - alt = Altitude [m]
%   
%   Lon input u_lon:
%   - d_e = Elevator angle [rad]
%   - d_p = Prop throttle [0-1]
%   
%   Author: Dan Oates (WPI Class of 2020)

% Imports
import('AE5224.fixed_wing.control.linear.lon.x_to_xlon');
import('AE5224.fixed_wing.control.linear.lon.xlon_to_x');
import('AE5224.fixed_wing.control.linear.lon.u_to_ulon');
import('AE5224.fixed_wing.control.linear.lon.ulon_to_u');
import('AE5224.fixed_wing.control.linear.sub_model');

% Numerical linearization
[A_lon, B_lon, x_lon, u_lon] = sub_model(...
    model, x_st, u_st, ...
    @x_to_xlon, @xlon_to_x, ...
    @u_to_ulon, @ulon_to_u);

% Correct known elements
A_lon(4, 1:2) = 0;
A_lon(4, 3) = 1;
A_lon(5, 3) = 0;
A_lon(3:4, 4) = 0;
A_lon(:, 5) = 0;
B_lon(4:5, 1) = 0;
B_lon(2:5, 2) = 0;

end