function [p_e, q_e, v_e, w_b, de, da, dr, dp] = trim(body, p, h, V, R, gam)
%[p_e, q_e, v_e, w_b, de, da, dr, dp] = TRIM(body, p, h, V, R, gam)
%   Get trim conditions for fixed-wing aircraft
%   
%   Inputs:
%   - body = Fixed-wing model [AE5224.fixed_wing.Body]
%   - p = Air density [kg/m^3]
%   - h = Trim altitude [m]
%   - V = Trim airspeed [m/s]
%   - R = Trim turn radius [m]
%   - gam = Trim climb angle [rad]
%   
%   Outputs:
%   - p_e = Init Earth position [m]
%   - q_e = Init Earth pose [quaternion]
%   - v_e = Init Earth velocity [m/s]
%   - w_b = Init Body angle rate [rad/s]
%   - de = Trim elevator angle [rad]
%   - da = Trim aileron angle [rad]
%   - dr = Trim rudder angle [rad]
%   - dp = Trim prop throttle [0-1]
%   
%   Currently only supports R = inf, gam = 0

% Imports
import('AE5224.get_g');
import('quat.Quat');

% Default args
if nargin < 6, gam = 0; end
if nargin < 5, R = inf; end

% Trim constraints
p_e = [0; 0; -h];
v_ex = V*cos(gam);
v_ey = 0;
v_ez = -V*sin(gam);
v_e = [v_ex; v_ey; v_ez];
w_ex = 0;
w_ey = 0;
w_ez = V*cos(gam)/R;
w_e = [w_ex; w_ey; w_ez];
F_c = body.m * v_ex^2 / R;

% Symbolic unknowns
qw = sym('qw');     % Attitude w [quat]
qx = sym('qx');     % Attitude x [quat]
qy = sym('qy');     % Attitude y [quat]
qz = sym('qz');     % Attitude z [quat]
w_bx = sym('wx');   % Body angle rate x [rad/s]
w_by = sym('wy');   % Body angle rate y [rad/s]
w_bz = sym('wz');   % Body angle rate z [rad/s]
de = sym('de');     % Elevator angle [rad]
da = sym('da');     % Aileron angle [rad]
dr = sym('dr');     % Rudder angle [rad]
dp = sym('dp');     % Prop thrust [0-1]

% Combined symbols
q_e = [qw; qx; qy; qz];
Reb = Quat(q_e).conj().mat_rot();
w_b = [w_bx; w_by; w_bz];

% Airspeed angles
v_a = Reb * v_e;
v_ax = v_a(1);
v_ay = v_a(2);
v_az = v_a(3);
al = atan(v_az / v_ax);
be = asin(v_ay / V);

% Gravitational forces
F_gz = body.m * get_g();
F_g = Reb * [0; 0; F_gz];

% Longitudinal AFMs
F_air = 0.5 * p * V^2 * body.S_wn;
M_lon = F_air * body.c_wn;
C_c = body.c_wn / (2 * V);
C_Fl = ...
    body.C_Fl_of + ...
    body.C_Fl_al * al + ...
    body.C_Fl_wy * C_c * w_by + ...
    body.C_Fl_de * de;
C_Fd = ...
    body.C_Fd_of + ...
    body.C_Fd_al * al + ...
    body.C_Fd_wy * C_c * w_by + ...
    body.C_Fd_de * de;
C_My = ...
    body.C_My_of + ...
    body.C_My_al * al + ...
    body.C_My_wy * C_c * w_by + ...
    body.C_My_de * de;
Fl = F_air * C_Fl;
Fd = F_air * C_Fd;
c_al = cos(al);
s_al = sin(al);
F_ax = +Fl*s_al - Fd*c_al;
F_az = -Fl*c_al - Fd*s_al;
M_ay = M_lon * C_My;

% Lateral AFMs
M_lat = F_air * body.b_wn;
C_b = body.b_wn / (2 * V);
C_Fy = ...
    body.C_Fy_of + ...
    body.C_Fy_be * be + ...
    body.C_Fy_wx * C_b * w_bx + ...
    body.C_Fy_wz * C_b * w_bz + ...
    body.C_Fy_da * da + ...
    body.C_Fy_dr * dr;
C_Mx = ...
    body.C_Mx_of + ...
    body.C_Mx_be * be + ...
    body.C_Mx_wx * C_b * w_bx + ...
    body.C_Mx_wz * C_b * w_bz + ...
    body.C_Mx_da * da + ...
    body.C_Mx_dr * dr;
C_Mz = ...
    body.C_Mz_of + ...
    body.C_Mz_be * be + ...
    body.C_Mz_wx * C_b * w_bx + ...
    body.C_Mz_wz * C_b * w_bz + ...
    body.C_Mz_da * da + ...
    body.C_Mz_dr * dr;
F_ay = F_air * C_Fy;
M_ax = M_lat * C_Mx;
M_az = M_lat * C_Mz;

% Prop FMs
S_pr = body.S_pr;
C_pr = body.C_pr;
F_px = 0.5 * p * S_pr * C_pr * ((body.k_v * dp)^2 - V^2);
M_px = -body.k_t * (body.k_w * dp)^2;

% Sum forces and moments
F_a = [F_ax; F_ay; F_az];
F_p = [F_px; 0; 0];
F_b = F_g + F_a + F_p;
M_a = [M_ax; M_ay; M_az];
M_p = [M_px; 0; 0];
M_b = M_a + M_p;

% Solve equations
eqs = [...
    F_b == Reb * [0; F_c; 0]; ...
    M_b == cross(w_b, body.I_b*w_b); ...
    w_b == Reb * w_e;
    norm(q_e) == 1];
sol = vpasolve(eqs, ...
    [qw, qx, qy, qz, w_bx, w_by, w_bz, de, da, dr, dp], ...
    [ 1,  0,  0,  0, w_ex, w_ey, w_ez,  0,  0,  0, .5]);

% Initial conditions
q_e = double([sol.qw; sol.qx; sol.qy; sol.qz]);
w_b = double([sol.wx; sol.wy; sol.wz]);

% Trim controls
de = double(sol.de);
da = double(sol.da);
dr = double(sol.dr);
dp = double(sol.dp);

end