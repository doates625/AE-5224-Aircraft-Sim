function log = sim(body, x_st, u_st, t_max, del_t)
%log = SIM(body, x_st, u_st, t_max, del_t)
%   Simulate and plot trim condition
%   
%   Inputs:
%   - body = Aircraft model [AE5224.rigid_body.Body]
%   - x_st = Trim state [p_e; q_e; v_e; w_b]
%   - u_st = Trim control vector
%   - t_max = Simulation duration [s]
%   - del_t = Simulation timestep [s]

% Imports
import('AE5224.rigid_body.Sim');
import('AE5224.rigid_body.Log');
import('timing.ProgDisp');

% Simulator
fprintf('Simulating trim...\n');
sim = Sim(body, x_st, del_t);
log = Log(sim);
prog = ProgDisp();
prog.start();
while sim.t < t_max
    sim.update(u_st);
    log.update();
    prog.update(sim.t / t_max);
end

% Plot trajectory
fprintf('Plotting trajectory...\n')
log.plot_path();
drawnow

end