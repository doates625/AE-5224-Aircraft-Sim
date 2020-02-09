function [q_e, w_b, u] = get_trim(body, trim)
%[q_e, w_b, u] = GET_TRIM(body, trim)
%   Get trim conditions for fixed-wing aircraft
%   
%   Inputs:
%   - body = Fixed-wing model [AE5224.fixed_wing.Body]
%   - trim = Trim conditions [AE5224.Trim]
%   
%   Outputs:
%   - q_e = Init Earth pose [quaternion]
%   - w_b = Init Body angle rate [rad/s]
%   - u = Trim controls [d_e; d_a; d_r; d_p]

% Imports
import('AE5224.get_g');
import('AE5224.get_p');
import('quat.Quat');

% Constants
g = get_g();
p = get_p();

% Unpack trim
V = trim.V;
v_e = trim.v_e;
w_e = trim.w_e;
w_ex = w_e(1);
w_ey = w_e(2);
w_ez = w_e(3);
F_c = trim.get_F_c(body.m);

% Symbolic unknowns
q_ew = sym('q_ew'); % Attitude w [quat]
q_ex = sym('q_ex'); % Attitude x [quat]
q_ey = sym('q_ey'); % Attitude y [quat]
q_ez = sym('q_ez'); % Attitude z [quat]
w_bx = sym('w_ex'); % Body angle rate x [rad/s]
w_by = sym('w_ey'); % Body angle rate y [rad/s]
w_bz = sym('w_ez'); % Body angle rate z [rad/s]
d_e = sym('d_e');   % Elevator angle [rad]
d_a = sym('d_a');   % Aileron angle [rad]
d_r = sym('d_r');   % Rudder angle [rad]
d_p = sym('d_p');   % Prop thrust [0-1]

% Combined symbols
q_e = [q_ew; q_ex; q_ey; q_ez];
R_eb = Quat(q_e).conj().mat_rot();
w_b = [w_bx; w_by; w_bz];

% Airspeed angles
v_a = R_eb * v_e;
v_ax = v_a(1);
v_ay = v_a(2);
v_az = v_a(3);
al = atan(v_az / v_ax);
be = asin(v_ay / V);

% Gravitational forces
F_gz = body.m * g;
F_g = R_eb * [0; 0; F_gz];

% Longitudinal AFMs
F_air = 0.5 * p * V^2 * body.S_wn;
M_lon = F_air * body.c_wn;
C_c = body.c_wn / (2 * V);
C_Fl = ...
    body.C_Fl_of + ...
    body.C_Fl_al * al + ...
    body.C_Fl_wy * C_c * w_by + ...
    body.C_Fl_de * d_e;
C_Fd = ...
    body.C_Fd_of + ...
    body.C_Fd_al * al + ...
    body.C_Fd_wy * C_c * w_by + ...
    body.C_Fd_de * d_e;
C_My = ...
    body.C_My_of + ...
    body.C_My_al * al + ...
    body.C_My_wy * C_c * w_by + ...
    body.C_My_de * d_e;
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
    body.C_Fy_da * d_a + ...
    body.C_Fy_dr * d_r;
C_Mx = ...
    body.C_Mx_of + ...
    body.C_Mx_be * be + ...
    body.C_Mx_wx * C_b * w_bx + ...
    body.C_Mx_wz * C_b * w_bz + ...
    body.C_Mx_da * d_a + ...
    body.C_Mx_dr * d_r;
C_Mz = ...
    body.C_Mz_of + ...
    body.C_Mz_be * be + ...
    body.C_Mz_wx * C_b * w_bx + ...
    body.C_Mz_wz * C_b * w_bz + ...
    body.C_Mz_da * d_a + ...
    body.C_Mz_dr * d_r;
F_ay = F_air * C_Fy;
M_ax = M_lat * C_Mx;
M_az = M_lat * C_Mz;

% Prop FMs
S_pr = body.S_pr;
C_pr = body.C_pr;
F_px = 0.5 * p * S_pr * C_pr * ((body.k_v * d_p)^2 - V^2);
M_px = -body.k_t * (body.k_w * d_p)^2;

% Sum forces and moments
F_a = [F_ax; F_ay; F_az];
F_p = [F_px; 0; 0];
F_b = F_g + F_a + F_p;
M_a = [M_ax; M_ay; M_az];
M_p = [M_px; 0; 0];
M_b = M_a + M_p;

% Solve equations
L_b = body.I_b * w_b;
eqs = [
    F_b == R_eb * [0; F_c; 0];  % Net force
    M_b == cross(w_b, L_b);     % Net moment
    w_b == R_eb * w_e;          % Body angle rate
    R_eb(2, 1) == 0;            % Point in +x
    norm(q_e) == 1;             % Unit quaternion
];
sol = vpasolve(eqs, ...
    [q_ew, q_ex, q_ey, q_ez, w_bx, w_by, w_bz, d_e, d_a, d_r, d_p], ...
    [   1,    0,    0,    0, w_ex  w_ey, w_ez,   0,   0,   0, 0.5]);

% Trim angular conditions
q_e = double([sol.q_ew; sol.q_ex; sol.q_ey; sol.q_ez]);
w_b = double([sol.w_ex; sol.w_ey; sol.w_ez]);

% Trim controls
d_e = double(sol.d_e);
d_a = double(sol.d_a);
d_r = double(sol.d_r);
d_p = double(sol.d_p);
u = [d_e; d_a; d_r; d_p];

end