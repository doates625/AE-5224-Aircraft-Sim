function x_lat = x_to_xlat(x)
%x_lat = X_TO_XLAT(x)
%   Convert full state to lat state
%   
%   Inputs:
%   - x = Full state vector
%   
%   Outputs:
%   - x_lat = Lat state vector
%   
%   Author: Dan Oates (WPI Class of 2020)

% Imports
import('AE5224.rigid_body.Model.unpack_x');
import('quat.Quat');

% Function
x_lat = zeros(5, 1);
[~, q_e, v_e, w_b] = unpack_x(x);
[th_z, ~, th_x] = Quat(q_e).euler();
v_b = Quat(q_e).inv().rotate(v_e);
x_lat(1) = v_b(2);
x_lat(2) = w_b(1);
x_lat(3) = w_b(3);
x_lat(4) = th_x;
x_lat(5) = th_z;

end