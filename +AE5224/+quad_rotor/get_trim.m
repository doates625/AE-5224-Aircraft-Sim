function [q_e, w_b, w_1, w_2, w_3, w_4] = get_trim(body, trim)
%[q_e, w_b, w_1, w_2, w_3, w_4] = GET_TRIM(body, trim)
%   Get trim conditions for quadrotor aircraft
%   
%   Inputs:
%   - body = Quadrotor model [AE5224.quad_rotor.Body]
%   - trim = Trim conditions [AE5224.Trim]
%   
%   Outputs:
%   - q_e = Init Earth pose [quaternion]
%   - w_b = Init Body angle rate [rad/s]
%   - w_1 = Prop 1 rate [rpm]
%   - w_2 = Prop 2 rate [rpm]
%   - w_3 = Prop 3 rate [rpm]
%   - w_4 = Prop 4 rate [rpm]

% Imports
import('AE5224.get_g');
import('quat.Quat');

% Unpack trim
w_e = trim.w_e;
F_c = trim.get_F_c(body.m);

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