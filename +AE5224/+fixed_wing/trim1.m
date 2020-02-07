function log = trim1(t_max, del_t)
%log = TRIM1(t_max, del_t) Run trim 1 simulation test
%   
%   Inputs:
%   - t_max = Sim duration [s, def = 60.0]
%   - del_t = Sim time delta [s, def = 0.01]
%   
%   Outputs:
%   - log = Sim log file [AE5224.rigid_body.Log]
%   
%   Trim conditions:
%   - Constant altitude h* = 100m
%   - Constant airspeed V* = 30m/s
%   
%   Author: Dan Oates (WPI Class of 2020)

% Imports
import('AE5224.fixed_wing.Body');
import('AE5224.fixed_wing.Sim');
import('AE5224.rigid_body.Log');
import('AE5224.get_g');
import('quat.Quat');

% Default args
if nargin < 1, t_max = 10.0; end
if nargin < 2, del_t = 0.01; end

% Trim conditions
h = 100;    % Altitude [m]
V = 30;     % Airspeed [m/s]

% Other conditions
w_e = zeros(3, 1);  % Wind-speed [m/s]
p = 1.2682;         % Air density [kg/m^3]

% Numerical solution
body = Body();
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

% Initial conditions
p_e = [0; 0; -h];   % Initial position
q_e = Quat([0; 1; 0], al).vector(); % Initial pose (quaternion)
v_e = [V; 0; 0];    % Initial velocity
w_b = [0; 0; 0];    % Initial angle rate

% Other trim controls
da = 0;     % Aileron angle [rad] (no lateral motion)
dr = 0;     % Rudder [rad] (no lateral motion)

% Simulation
sim = Sim(body, del_t, p_e, q_e, v_e, w_b);
log = Log(sim);
while sim.t < t_max
    sim.update(de, da, dr, dp, w_e, p);
    log.update();
end
log.plot();

end