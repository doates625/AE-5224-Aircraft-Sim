function log = sim_trim(V, R, gam, h, p, t_max, del_t)
%log = SIM_TRIM(V, R, gam, h, p, t_max, del_t)
%   Simulate and plot trim condition
%   
%   Inputs:
%   - V = Airspeed [m/s, def = 30.0]
%   - R = Turn radius [m, def = inf]
%   - gam = Climb angle [rad, def = 0.0]
%   - h = Altitude [m, def = 100.0]
%   - p = Air density [kg/m^3, def = 1.2682]
%   - t_max = Sim duration [s, def = 60.0]
%   - del_t = Sim timestep [s, def = 0.01]
%   
%   Outputs:
%   - log = Sim log file [AE5224.rigid_body.Log]
%   
%   Author: Dan Oates (WPI Class of 2020)
clc

% Imports
import('AE5224.fixed_wing.Body');
import('AE5224.fixed_wing.Sim');
import('AE5224.fixed_wing.get_trim');
import('AE5224.rigid_body.Log');
import('timing.ProgDisp');

% Default args
if nargin < 1, V = 30.0; end
if nargin < 2, R = inf; end
if nargin < 3, gam = 0.0; end
if nargin < 4, h = 100.0; end
if nargin < 5, p = 1.2682; end
if nargin < 6, t_max = 60.0; end
if nargin < 7, del_t = 0.01; end

% Initial printout
fprintf('Fixed-Wing Trim Simulator\n\n')

% Trim solver
fprintf('Numerical trim solver...\n')
body = Body();
[p_e, q_e, v_e, w_b, d_e, d_a, d_r, d_p] = get_trim(body, V, R, gam, h, p);

% Simulation
fprintf('Simulating flight...\n')
sim = Sim(body, del_t, p_e, q_e, v_e, w_b, p);
log = Log(sim);
prog = ProgDisp();
prog.start();
while sim.t < t_max
    sim.update(d_e, d_a, d_r, d_p);
    log.update();
    prog.update(sim.t / t_max);
end

% Plot trajectory
fprintf('Plotting trajectory...\n')
log.plot_path();

end