function u = ulon_to_u(u_lon, u)
%u = ULON_TO_U(u_lon, u)
%   Convert lon input to full input
%   
%   Inputs:
%   - u_lon = Lon input vector
%   - u = Full input reference
%   
%   Outputs:
%   - u = Full input vector

d_e = u_lon(1);
d_p = u_lon(2);
u(1) = d_e;
u(4) = d_p;

end