function [p_e, v_e, w_e, F_c] = get_trim(body, V, R, gam, h)
%[p_e, v_e, w_e, F_c] = GET_TRIM(body, V, R, gam, h, p)
%   Get trim conditions for rigid body
%   
%   Inputs:
%   - body = Body model [AE5224.rigid_body.Body]
%   - V = Trim airspeed [m/s]
%   - R = Trim turn radius [m]
%   - gam = Trim climb angle [rad]
%   - h = Trim altitude [m]
%   
%   Outputs:
%   - p_e = Init Earth position [m]
%   - v_e = Init Earth velocity [m/s]
%   - w_e = Init Earth angle rate [rad/s]
%   - F_c = Centripetal force [N]

p_e = [0; 0; -h];
v_ex = V*cos(gam);
v_ey = 0;
v_ez = -V*sin(gam);
v_e = [v_ex; v_ey; v_ez];
w_ex = 0;
w_ey = 0;
w_ez =  V*cos(gam)/R;
w_e = [w_ex; w_ey; w_ez];
F_c = body.m * v_e(1)^2 / R;

end