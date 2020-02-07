function log = trim3(t_max, del_t)
%log = TRIM3(t_max, del_t) Run trim 3 simulation test
%   
%   Inputs:
%   - t_max = Sim duration [s, def = 25.0]
%   - del_t = Sim time delta [s, def = 0.01]
%   
%   Outputs:
%   - log = Sim log file [AE5224.rigid_body.Log]
%   
%   Trim conditions:
%   - Initial altitude h* = 100m
%   - Airspeed V* = 30m/s
%   - Turn radius R* = 150m
%   - Climb angle gam* = 3deg
%   
%   Author: Dan Oates (WPI Class of 2020)

% Imports
import('AE5224.fixed_wing.Body');
import('AE5224.fixed_wing.Sim');
import('AE5224.fixed_wing.trim');
import('AE5224.rigid_body.Log');

% Default args
if nargin < 1, t_max = 25.0; end
if nargin < 2, del_t = 0.01; end

% Trim conditions
h = 100;    % Altitude [m]
V = 30;     % Airspeed [m/s]
R = 150;    % Turn radius [m]
gam = ...   % Climb angle [rad]
    deg2rad(3);   
p = 1.2682; % Air density [kg/m^3]

% Trim solver
body = Body();
[p_e, q_e, v_e, w_b, de, da, dr, dp] = trim(body, p, h, V, R, gam);

% Simulation
sim = Sim(body, del_t, p_e, q_e, v_e, w_b);
log = Log(sim);
while sim.t < t_max
    w_e = zeros(3, 1);
    sim.update(de, da, dr, dp, w_e, p);
    log.update();
end
log.plot();

end