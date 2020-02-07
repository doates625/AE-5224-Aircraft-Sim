function log = sim_trim(V, R, gam, h, t_max, del_t)
%log = SIM_TRIM(V, R, gam, h, t_max, del_t)
%   Simulate and plot trim condition
%   
%   Inputs:
%   - V = Airspeed [m/s, def = 0.0]
%   - R = Turn radius [m, def = inf]
%   - gam = Climb angle [rad, def = 0.0]
%   - h = Altitude [m, def = 100.0]
%   - t_max = Sim duration [s, def = 10.0]
%   - del_t = Sim timestep [s, def = 0.01]
%   
%   Outputs:
%   - log = Sim log file [AE5224.rigid_body.Log]
%   
%   Author: Dan Oates (WPI Class of 2020)
clc

% Imports
import('AE5224.quad_rotor.Body')
import('AE5224.quad_rotor.Sim')
import('AE5224.quad_rotor.get_trim')
import('AE5224.rigid_body.Log')
import('timing.ProgDisp')

% Default args
if nargin < 1, V = 0.0; end
if nargin < 2, R = inf; end
if nargin < 3, gam = 0.0; end
if nargin < 4, h = 100.0; end
if nargin < 5, t_max = 10.0; end
if nargin < 6, del_t = 0.01; end

% Initial printout
fprintf('Quadrotor Trim Simulator\n\n')

% Trim solver
fprintf('Numerical trim solver...\n')
body = Body();
[p_e, q_e, v_e, w_b, w_1, w_2, w_3, w_4] = get_trim(body, V, R, gam, h);

% Simulation
fprintf('Simulating flight...\n')
sim = Sim(body, del_t, p_e, q_e, v_e, w_b);
log = Log(sim);
prog = ProgDisp();
prog.start();
while sim.t < t_max
    sim.update(w_1, w_2, w_3, w_4);
    log.update();
    prog.update(sim.t / t_max);
end

% Plot trajectory
fprintf('Plotting trajectory...\n')
log.plot_path();

end