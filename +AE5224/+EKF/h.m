function z = h(x)
%z = H(x)
%   Output function
%   - x = State vector
%   - z = Output vector

% Imports
import('AE5224.EKF.unpack_x');
import('AE5224.EKF.pack_z');

% Function
[~, p_e, v_e, ~, ~] = unpack_x(x);
z = pack_z(p_e, v_e);

end