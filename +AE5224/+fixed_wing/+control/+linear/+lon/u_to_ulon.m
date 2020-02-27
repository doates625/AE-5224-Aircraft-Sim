function u_lon = u_to_ulon(u)
%u_lon = U_TO_ULON(u)
%   Convert full input to lon input
%   
%   Inputs:
%   - u = Full input vector
%   
%   Outputs:
%   - u_lon = Lon input vector
%   
%   Author: Dan Oates (WPI Class of 2020)

d_e = u(1);
d_p = u(4);
u_lon = [d_e; d_p];

end