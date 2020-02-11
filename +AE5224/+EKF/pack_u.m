function u = pack_u(w_b, a_b)
%u = PACK_U(w_b, a_b)
%   Make input vector from components
%   
%   Inputs:
%   - w_b = Angular velocity Body [rad/s]
%   - a_b = Acceleration Body [m/s^2]
%   
%   Outputs:
%   - u = Input vector

u = [w_b; a_b];

end