function x = xlon_to_x(x_lon, x)
%x = XLON_TO_X(x_lon, x)
%   Convert lon state to full state
%   
%   Inputs:
%   - x_lon = Lon state vector
%   - x = Full state reference
%   
%   Outputs:
%   - x = Full state vector
%   
%   Author: Dan Oates (WPI Class of 2020)

% Imports
import('AE5224.rigid_body.Body.unpack');
import('AE5224.rigid_body.Body.pack');
import('quat.Quat');

% Unpack reference state
[p_e, q_e, v_e, w_b] = unpack(x);

% Attitude
[th_z, ~, th_x] = Quat(q_e).euler();
th_y = x_lon(4);
q_z = Quat([0; 0; 1], th_z);
q_y = Quat([0; 1; 0], th_y);
q_x = Quat([1; 0; 0], th_x);
q_e = q_z * q_y * q_x;
q_e = q_e.vector();

% Linear velocity
v_b = Quat(q_e).inv().rotate(v_e);
v_b(1) = x_lon(1);
v_b(3) = x_lon(2);
v_e = Quat(q_e).rotate(v_b);

% Angular velocity
w_b(2) = x_lon(3);

% Altitude
p_e(3) = x_lon(5);

% Pack state
x = pack(p_e, q_e, v_e, w_b);

end