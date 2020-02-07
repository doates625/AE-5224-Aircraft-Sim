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

% Lateral solver
F_air = 0.5 * p * V^2 * body.S_wn;
M_lon = F_air * body.c_wn;
K_pr = 0.5 * p * body.S_pr * body.C_pr;
syms Fp Fl Fd al de dp
Fp = K_pr * ((body.k_v*dp)^2 - V^2);
Fl = F_air * (body.C_Fl_of + body.C_Fl_al*al + body.C_Fl_de*de);
Fd = F_air * (body.C_Fd_of + body.C_Fd_al*al + body.C_Fd_de*de);
My = M_lon * (body.C_My_of + body.C_My_al*al + body.C_My_de*de);
eqns = [...
     0 == Fp*cos(al) - Fd; ...
     0 == Fp*sin(al) + Fl - body.m*get_g(); ...
     0 == My];
sol = vpasolve(eqns, [al, de, dp], [0, 0, 0.5]);
al = double(sol.al);
de = double(sol.de);
dp = double(sol.dp);

% Longitudinal solver (TODO)
da = 0;
dr = 0;

% Initial conditions
p_e = [0; 0; -h];
if R < inf
    p_e(2) = -R;
end
q_e = Quat([0; 1; 0], al).vector(); % Initial pose (quaternion)
v_e = [V; 0; 0];    % Initial velocity
w_b = [0; 0; 0];    % Initial angle rate

end

