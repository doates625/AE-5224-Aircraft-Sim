function x = xlat_to_x(x_lat, x)
%x = XLAT_TO_X(x_lat, x)
%   Convert lat state to full state
%   
%   Inputs:
%   - x_lat = Lat state vector
%   - x = Full state reference
%   
%   Outputs:
%   - x = Full state vector
%   
%   Author: Dan Oates (WPI Class of 2020)

% Imports
import('AE5224.rigid_body.Model.unpack_x');
import('AE5224.rigid_body.Model.pack_x');
import('quat.Quat');

% Unpack reference state
[p_e, q_e, v_e, w_b] = unpack_x(x);

% Attitude
[~, th_y, ~] = Quat(q_e).euler();
th_x = x_lat(4);
th_z = x_lat(5);
q_z = Quat([0; 0; 1], th_z);
q_y = Quat([0; 1; 0], th_y);
q_x = Quat([1; 0; 0], th_x);
q_e = q_z * q_y * q_x;
q_e = q_e.vector();

% Linear velocity
v_b = Quat(q_e).inv().rotate(v_e);
v_b(2) = x_lat(1);
v_e = Quat(q_e).rotate(v_b);

% Angular velocity
w_b(1) = x_lat(2);
w_b(3) = x_lat(3);

% Pack state
x = pack_x(p_e, q_e, v_e, w_b);

end