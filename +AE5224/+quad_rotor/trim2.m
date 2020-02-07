function log = trim2(t_max, del_t)
%log = TRIM2(t_max, del_t) Run trim 2 simulation test
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
%   
%   Author: Dan Oates (WPI Class of 2020)

% Imports
import('AE5224.quad_rotor.Body');
import('AE5224.quad_rotor.Sim');
import('AE5224.rigid_body.Log');
import('AE5224.get_g');

% Default args
if nargin < 1, t_max = 10.0; end
if nargin < 2, del_t = 0.01; end

% Trim Conditions
h = 100;    % Altitude [m]
v = 10;     % Airspeed [m/s]

% Trim states
p_e = [0; 0; -h];
q_e = [1; 0; 0; 0];
v_e = [v; 0; 0];
w_b = [0; 0; 0];

% Trim controls
body = Body();
Fg = body.m * get_g();
w = sqrt(Fg / (4 * body.k_F));
w_1 = w;    % Prop 1 [rpm]
w_2 = w;    % Prop 2 [rpm]
w_3 = w;    % Prop 3 [rpm]
w_4 = w;    % Prop 4 [rpm]

% Simulation
sim = Sim(body, del_t, p_e, q_e, v_e, w_b);
log = Log(sim);
while sim.t < t_max
    sim.update(w_1, w_2, w_3, w_4);
    log.update();
end
log.plot();

end