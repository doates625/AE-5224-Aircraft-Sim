function log = trim1(t_max, del_t)
%log = TRIM1(t_max, del_t) Run trim 1 simulation test
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
%   - Constant airspeed V* = 30m/s
%   
%   Author: Dan Oates (WPI Class of 2020)

% Imports
import('AE5224.fixed_wing.Body');
import('AE5224.fixed_wing.Sim');
import('AE5224.fixed_wing.trim');
import('AE5224.rigid_body.Log');

% Default args
if nargin < 1, t_max = 10.0; end
if nargin < 2, del_t = 0.01; end

% Trim conditions
h = 100;    % Altitude [m]
V = 30;     % Airspeed [m/s]
p = 1.2682; % Air density [kg/m^3]

% Trim solver
body = Body();
[p_e, q_e, v_e, w_b, de, da, dr, dp] = trim(body, p, h, V);

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