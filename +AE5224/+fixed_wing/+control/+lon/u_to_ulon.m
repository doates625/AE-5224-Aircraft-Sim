function u_lon = u_to_ulon(u)
%u_lon = U_TO_ULON(u)
%   Convert full input to longitudinal input
%   
%   Inputs:
%   - u = Full input vector
%   
%   Outputs:
%   - u_lon = Lon input vector
%   
%   Full input u:
%   - d_e = Elevator angle [rad]
%   - d_a = Aileron angle [rad]
%   - d_r = Rudder angle [rad]
%   - d_p = Prop throttle [0-1]
%   
%   Lon input u_lon:
%   - d_e = Elevator angle [rad]
%   - d_p = Prop throttle [0-1]
%   
%   Author: Dan Oates (WPI Class of 2020)

d_e = u(1);
d_p = u(4);
u_lon = [d_e; d_p];

end