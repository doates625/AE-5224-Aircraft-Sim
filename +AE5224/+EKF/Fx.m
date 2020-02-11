function Fx_ = Fx(x, u)
%Fx_ = FX(x, u)
%   Get state Jacobian
%   - x = State vector
%   - u = Input vector
%   - Fx = State Jacobian

% Imports
import('AE5224.EKF.unpack_x');
import('AE5224.EKF.unpack_u');
import('AE5224.EKF.get_set_del_t');
import('quat.Quat');

% Unpack x, u, del_t
[q_e, ~, ~, ~, ~] = unpack_x(x);
[w_b, a_b] = unpack_u(u);
del_t = get_set_del_t();

% Jacobian
Fx_ = eye(16);
Fx_(1:4, 1:4) = Quat(w_b, norm(w_b) * del_t).mat_int();
Fx_(7:9, 8:10) = eye(3) * del_t;
Fx_(8:10, 1:4) = Quat(q_e).jac_rot(a_b) * del_t;

end