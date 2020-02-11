function [w_b, a_b] = unpack_u(u)
%[w_b, a_b] = UNPACK_U(u)
%   Get components from input vector
%   
%   Inputs:
%   - u = Input vector
%   
%   Outputs:
%   - w_b = Angular velocity Body [rad/s]
%   - a_b = Acceleration Body [m/s^2]

w_b = u(1:3);
a_b = u(4:6);

end