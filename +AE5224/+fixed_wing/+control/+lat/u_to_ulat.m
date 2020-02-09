function u_lat = u_to_ulat(u)
%u_lat = U_TO_ULAT(u)
%   Convert full input to lateral input
%   
%   Inputs:
%   - u = Full input vector
%   
%   Outputs:
%   - u_lon = Lat input vector
%   
%   Full input u:
%   - d_e = Elevator angle [rad]
%   - d_a = Aileron angle [rad]
%   - d_r = Rudder angle [rad]
%   - d_p = Prop throttle [0-1]
%   
%   Lon input u_lon:
%   - d_a = Aileron angle [rad]
%   - d_r = Rudder angle [rad]
%   
%   Author: Dan Oates (WPI Class of 2020)

d_a = u(2);
d_r = u(3);
u_lat = [d_a; d_r];

end