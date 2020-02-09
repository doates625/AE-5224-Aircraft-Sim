function log = sim_trim(type_, trim, t_max, del_t)
%log = SIM_TRIM(type_, trim, t_max, del_t)
%   Simulate and plot trim condition
%   
%   Inputs:
%   - type_ = Aircraft type ['Fixedwing', 'Quadrotor']
%   - trim = Trim conditions [AE5224.Trim]
%   - t_max = Sim duration [s, def = 10.0]
%   - del_t = Sim timestep [s, def = 0.01]
%   
%   Outputs:
%   - log = Sim log file [AE5224.rigid_body.Log]
%   
%   Author: Dan Oates (WPI Class of 2020)
clc

% Imports
import('AE5224.rigid_body.Log')
import('timing.ProgDisp')
switch type_
    case 'Fixedwing'
        import('AE5224.fixed_wing.Body')
        import('AE5224.fixed_wing.Sim')
        import('AE5224.fixed_wing.get_trim')
    case 'Quadrotor'
        import('AE5224.quad_rotor.Body')
        import('AE5224.quad_rotor.Sim')
        import('AE5224.quad_rotor.get_trim')
    otherwise
        error('Invalid type: ''%s''', type_)
end

% Default args
if nargin < 3, t_max = 10.0; end
if nargin < 4, del_t = 0.01; end

% Initial printout
fprintf('%s Trim Simulator\n\n', type_)

% Trim solver
fprintf('Numerical trim solver...\n')
body = Body();
p_e = trim.p_e;
v_e = trim.v_e;
[q_e, w_b, u] = get_trim(body, trim);

% Simulation
fprintf('Simulating flight...\n')
sim = Sim(body, del_t, p_e, q_e, v_e, w_b);
log = Log(sim);
prog = ProgDisp();
prog.start();
while sim.t < t_max
    sim.update(u);
    log.update();
    prog.update(sim.t / t_max);
end

% Plot trajectory
fprintf('Plotting trajectory...\n')
log.plot_path();

end