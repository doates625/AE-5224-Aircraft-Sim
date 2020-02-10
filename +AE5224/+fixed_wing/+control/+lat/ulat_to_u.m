function u = ulat_to_u(u_lat, u)
%u = ULAT_TO_U(u_lat, u)
%   Convert lat input to full input
%   
%   Inputs:
%   - u_lat = Lat input vector
%   - u = Full input reference
%   
%   Outputs:
%   - u = Full input vector
%   
%   Author: Dan Oates (WPI Class of 2020)

d_a = u_lat(1);
d_r = u_lat(2);
u(2) = d_a;
u(3) = d_r;

end