function u_lat = u_to_ulat(u)
%u_lat = U_TO_ULAT(u)
%   Convert full input to lateral input
%   
%   Inputs:
%   - u = Full input vector
%   
%   Outputs:
%   - u_lat = Lat input vector
%   
%   Author: Dan Oates (WPI Class of 2020)

d_a = u(2);
d_r = u(3);
u_lat = [d_a; d_r];

end