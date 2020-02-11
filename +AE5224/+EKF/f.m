function xn = f(x, u)
%xn = F(x, u)
%   State transition function
%   - x = State vector
%   - u = Input vector
%   - xn = Next state

% Imports
import('AE5224.EKF.unpack_x');
import('AE5224.EKF.unpack_u');
import('AE5224.EKF.get_set_del_t');
import('AE5224.EKF.pack_x');
import('AE5224.const.get_g');
import('quat.Quat');

% Unpack x and u
[q_e, p_e, v_e, w_e, b_e] = unpack_x(x);
[w_b, a_b] = unpack_u(u);
del_t = get_set_del_t();

% Attitude update
del_q = Quat(w_b, norm(w_b) * del_t);
q_e = Quat(q_e) * del_q;

% Position update
p_e = p_e + v_e * del_t;

% Velocity update
a_g = [0; 0; get_g()];
a_e = q_e.rotate(a_b) + a_g;
v_e = v_e + a_e * del_t;

% Re-pack x
q_e = q_e.vector();
xn = pack_x(q_e, p_e, v_e, w_e, b_e);

end