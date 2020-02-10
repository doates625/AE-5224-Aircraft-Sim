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
import('AE5224.sensors.GPS');
import('AE5224.sensors.Accel');
import('timing.ProgDisp');

% Setup simulator
sim = Sim(body, x_st, del_t);
log = Log(sim);

acc = Accel(100);
p_acc = zeros(3, 0);

gps = GPS();
n_gps = round(1 / sim.del_t);
p_gps = zeros(3, 0);

% Run simulator
fprintf('Simulating trim...\n');
prog = ProgDisp();
prog.start();
i_sim = 1;
while sim.t < t_max
    % Run simulator
    sim.update(u_st);
    log.update();
    
    % Sim accel
    z_acc = acc.measure(sim.x);
    p_acc = [p_acc, z_acc];
    
    % Simulate GPS
    if ~mod(i_sim, n_gps)
        z_gps = gps.measure(sim.x);
        p_gps = [p_gps, z_gps(1:3)];
    end
    
    % Print progress
    i_sim = i_sim + 1;
    prog.update(sim.t / t_max);
end

% Plot trajectory
fprintf('Plotting trajectory...\n')
log.plot_path();
drawnow

% Plot GPS readings
plot3(p_gps(1, :), p_gps(2, :), p_gps(3, :), 'kx');

figure;
for i = 1:3
    subplot(3, 1, i)
    plot(p_acc(i, :))
    grid on
end

end