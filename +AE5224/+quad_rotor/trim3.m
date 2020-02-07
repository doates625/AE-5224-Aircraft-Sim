function log = trim3(t_max, del_t)
%log = TRIM3(t_max, del_t) Run trim 3 simulation test
%   
%   Inputs:
%   - t_max = Sim duration [s, def = 10.0]
%   - del_t = Sim time delta [s, def = 0.01]
%   
%   Outputs:
%   - log = Sim log file [AE5224.rigid_body.Log]
%   
%   Trim conditions:
%   - Constant altitude h* = 100m
%   - Constant airspeed V* = 10m/s
%   - Turning radius R* = 20m
%   
%   Author: Dan Oates (WPI Class of 2020)

% Imports
import('AE5224.quad_rotor.Body');
import('AE5224.quad_rotor.Sim');
import('AE5224.rigid_body.Log');
import('AE5224.get_g');
import('quat.Quat');

% Default args
if nargin < 1, t_max = 10.0; end
if nargin < 2, del_t = 0.01; end

% Trim Conditions
h = 100;    % Altitude [m]
v = 10;     % Airspeed [m/s]
R = 20;     % Turn radius [m]

% Quadrotor rigid body model
body = Body();

% Trim states
wz = v/R;               % Yaw-rate [rad/s]
F_c = body.m*v*wz;      % Centripetal force [N]
F_g = body.m*get_g();   % Gravity [N]
F_p = hypot(F_c, F_g);  % Force norm [N]
tx = atan2(F_c, F_g);   % Roll angle [rad]
x_hat = [1; 0; 0];      % X unit vector
q_e = Quat(x_hat, tx);  % Init attitude [quat]
p_e = [0; -R; -h];      % Init position [m]
v_e = [v; 0; 0];        % Init velocity [m/s]
Rbe = q_e.mat_rot();    % Rotation matrix B-E
w_e = [0; 0; wz];       % Earth angle rate [rad/s]
w_b = Rbe \ w_e;        % Body angle rate [rad/s]
q_e = q_e.vector();     % Convert quat to vector

% Trim controls
M_b = cross(w_b, body.I_b*w_b);
k_F = body.k_F;
k_M = body.k_M;
Lk_F = body.L * body.k_F;
mat = [...
    [+k_F, +k_F, +k_F, +k_F]; ...
    [0, +Lk_F, 0, -Lk_F]; ...
    [-Lk_F, 0, +Lk_F, 0]; ...
    [+k_M, -k_M, +k_M, -k_M]];
wp2 = mat \ [F_p; M_b];
w1 = sqrt(wp2(1));
w2 = sqrt(wp2(2));
w3 = sqrt(wp2(3));
w4 = sqrt(wp2(4));

% Simulation
sim = Sim(body, del_t, p_e, q_e, v_e, w_b);
log = Log(sim);
while sim.t < t_max
    sim.update(w1, w2, w3, w4);
    log.update();
end
log.plot();

end