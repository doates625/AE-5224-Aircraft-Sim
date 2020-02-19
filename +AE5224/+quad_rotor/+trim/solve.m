function [x_st, u_st] = solve(model, trim)
%[x_st, u_st] = GET_TRIM(body, trim)
%   Get trim conditions for quadrotor aircraft
%   
%   Inputs:
%   - model = Quadrotor model [AE5224.quad_rotor.Model]
%   - trim = Trim conditions [AE5224.Trim]
%   
%   Outputs:
%   - x_st = Trim states [p_e; q_e; v_e; w_b]
%   - u_st = Trim controls [w_1; w_2; w_3; w_4]

% Imports
import('AE5224.rigid_body.Model.pack_x');
import('AE5224.const.get_g');
import('quat.Quat');

% Unpack trim
w_e = trim.w_e;         % Earth angle rate [rad/s]
m = model.m;            % Quadrotor mass [kg]
F_c = m * trim.a_e(2);  % Centripetal force [N]
F_g = m * get_g();      % Gravity [N]
F_p = hypot(F_c, F_g);  % Force norm [N]
tx = atan2(F_c, F_g);   % Roll angle [rad]
x_hat = [1; 0; 0];      % X unit vector
q_e = Quat(x_hat, tx);  % Init attitude [quat]
R_be = q_e.mat_rot();   % Rotation matrix B-E
w_b = R_be \ w_e;       % Body angle rate [rad/s]
q_e = q_e.vector();     % Convert to vector

% Trim controls
M_b = cross(w_b, model.I_b*w_b);
k_F = model.k_F;
k_M = model.k_M;
Lk_F = model.L * model.k_F;
mat = [...
    [+k_F, +k_F, +k_F, +k_F]; ...
    [0, +Lk_F, 0, -Lk_F]; ...
    [-Lk_F, 0, +Lk_F, 0]; ...
    [+k_M, -k_M, +k_M, -k_M]];
w_p2 = mat \ [F_p; M_b];
u_st = sqrt(w_p2);

% Pack trim state
x_st = pack_x(trim.p_e, q_e, trim.v_e, w_b);

end