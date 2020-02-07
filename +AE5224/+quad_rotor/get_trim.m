function [p_e, q_e, v_e, w_b, w_1, w_2, w_3, w_4] = ...
    get_trim(body, V, R, gam, h)
%[p_e, q_e, v_e, w_b, w_1, w_2, w_3, w_4] = GET_TRIM(body, V, R, gam, h)
%   Get trim conditions for quadrotor aircraft
%   
%   Inputs:
%   - body = Quadrotor model [AE5224.quad_rotor.Body]
%   - V = Trim airspeed [m/s]
%   - R = Trim turn radius [m]
%   - gam = Trim climb angle [rad]
%   - h = Trim altitude [m]
%   
%   Outputs:
%   - p_e = Init Earth position [m]
%   - q_e = Init Earth pose [quaternion]
%   - v_e = Init Earth velocity [m/s]
%   - w_b = Init Body angle rate [rad/s]
%   - w_1 = Prop 1 rate [rpm]
%   - w_2 = Prop 2 rate [rpm]
%   - w_3 = Prop 3 rate [rpm]
%   - w_4 = Prop 4 rate [rpm]

% Imports
import('AE5224.get_g');
import('quat.Quat');

% General trim solver
[p_e, v_e, w_e, F_c] = AE5224.rigid_body.get_trim(body, V, R, gam, h);

% Trim states
F_g = body.m*get_g();   % Gravity [N]
F_p = hypot(F_c, F_g);  % Force norm [N]
tx = atan2(F_c, F_g);   % Roll angle [rad]
x_hat = [1; 0; 0];      % X unit vector
q_e = Quat(x_hat, tx);  % Init attitude [quat]
R_be = q_e.mat_rot();   % Rotation matrix B-E
w_b = R_be \ w_e;       % Body angle rate [rad/s]
q_e = q_e.vector();     % Convert to vector

% Trim controls
M_b = cross(w_b, body.I_b*w_b);
k_F = body.k_F;
k_M = body.k_M;
Lk_F = body.L * body.k_F;
mat = [...
    [+k_F, +k_F, +k_F, +k_F]; ...
    [0, +Lk_F, 0, -Lk_F]; ...
    [-Lk_F, 0, +Lk_F, 0]; ...
    [+k_M, -k_M, +k_M, -k_M]];
w_p2 = mat \ [F_p; M_b];
w_1 = sqrt(w_p2(1));
w_2 = sqrt(w_p2(2));
w_3 = sqrt(w_p2(3));
w_4 = sqrt(w_p2(4));

end