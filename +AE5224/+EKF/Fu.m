function Fu_ = Fu(x, u)
%Fu_ = FU(x, u)
%   Get input Jacobian
%   - x = State vector
%   - u = Input vector
%   - Fu = State Jacobian

% Imports
import('AE5224.EKF.unpack_x');
import('AE5224.EKF.unpack_u');
import('AE5224.EKF.get_set_del_t');
import('quat.Quat');

% Unpack x, u, del_t
[q_e, ~, ~, ~, ~] = unpack_x(x);
[w_b, ~] = unpack_u(u);
del_t = get_set_del_t();

% Attitude from delta-q
q_e = Quat(q_e);
J_qe_dq = q_e.mat_ext();

% Delta-q from theta-b
norm_wb = norm(w_b);
norm_wb_inv = 1 / norm_wb;
del_th = 0.5 * norm_wb * del_t;
sin_th = sin(del_th);
cos_th = cos(del_th);
wh = w_b * norm_wb_inv;
J_dq_thb = zeros(4);
J_dq_thb(1, 1) = -0.5 * sin_th;
J_dq_thb(2:4, 1) = 0.5 * cos_th * wh;
J_dq_thb(2:4, 2:4) = sin_th * eye(3);

% Theta-b from w_b
J_th_wb = norm_wb_inv * (w_b.' * del_t);
J_wh_wb = norm_wb_inv * (eye(3) - wh * wh.');
J_thb_wb = [J_th_wb; J_wh_wb];

% Jacobian
Fu_ = zeros(16, 6);
Fu_(1:4, 1:3) = J_qe_dq * J_dq_thb * J_thb_wb;
Fu_(8:10, 4:6) = q_e.mat_rot() * del_t;

end