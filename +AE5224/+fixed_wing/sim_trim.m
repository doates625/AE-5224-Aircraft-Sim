function log = sim_trim(trim, t_max, del_t)
%log = SIM_TRIM(trim, t_max, del_t)
%   Simulate and plot trim condition
%   
%   Inputs:
%   - trim = Trim conditions [AE5224.Trim]
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
if nargin < 2, t_max = 60.0; end
if nargin < 3, del_t = 0.01; end

% Initial printout
fprintf('Fixed-Wing Trim Simulator\n\n')

% Trim solver
fprintf('Numerical trim solver...\n')
body = Body();
p_e = trim.p_e;
v_e = trim.v_e;
[q_e, w_b, d_e, d_a, d_r, d_p] = get_trim(body, trim);

% Simulation
fprintf('Simulating flight...\n')
sim = Sim(body, del_t, p_e, q_e, v_e, w_b);
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