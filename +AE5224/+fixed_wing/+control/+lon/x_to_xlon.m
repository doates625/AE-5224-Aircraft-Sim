function x_lon = x_to_xlon(x)
%x_lon = X_TO_XLON(x)
%   Convert full state to longitudinal state
%   
%   Inputs:
%   - x = Full state vector
%   
%   Outputs:
%   - x_lon = Lon state vector
%   
%   Author: Dan Oates (WPI Class of 2020)

% Imports
import('AE5224.rigid_body.Body.unpack');
import('quat.Quat');

% Function
x_lon = zeros(5, 1);
[p_e, q_e, v_e, w_b] = unpack(x);
[~, th_y, ~] = Quat(q_e).euler();
v_b = Quat(q_e).inv().rotate(v_e);
x_lon(1) = v_b(1);
x_lon(2) = v_b(3);
x_lon(3) = w_b(2);
x_lon(4) = th_y;
x_lon(5) = -p_e(3);

end