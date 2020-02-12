function log_sim = sim(body, x_st, u_st, t_max, del_t)
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
import('AE5224.const.get_b');
import('AE5224.rigid_body.Sim');
import('AE5224.rigid_body.Model');
import('AE5224.sensors.Gyro');
import('AE5224.sensors.Acc');
import('AE5224.sensors.Mag');
import('AE5224.sensors.GPS');
import('AE5224.EKF.EKF');
import('AE5224.EKF.EKF.pack_x');
import('AE5224.EKF.Log');
import('timing.ProgDisp');

% Create simulator
sim = Sim(body, x_st, del_t);
log_sim = AE5224.rigid_body.Log(sim);

% Create IMU
f_imu = 1 / del_t;
gyro = Gyro(f_imu);
accel = Acc(f_imu);
mag = Mag();

% Create GPS
f_gps = 1.0;
gps = GPS();
n_gps = round(f_gps / sim.del_t);

% Create EKF
[p_e, q_e, v_e, ~] = Model.unpack_x(x_st);
w_e = zeros(3, 1);
b_e = get_b();
x_est = pack_x(q_e, p_e, v_e, w_e, b_e);
cov_w = gyro.cov_z;
cov_a = accel.cov_z;
cov_p = gps.cov_z(1:3, 1:3);
cov_v = gps.cov_z(4:6, 4:6);
cov_x = zeros(16);
cov_x(05:07, 05:07) = cov_p;
cov_x(08:10, 08:10) = cov_v;
cov_x(11:13, 11:13) = cov_v;
cov_x(14:16, 14:16) = mag.cov_z;
ekf = EKF(x_est, cov_x, cov_w, cov_a, cov_p, cov_v, del_t);
log_ekf = AE5224.EKF.Log(sim, ekf);

% Run simulator
fprintf('Simulating trim...\n');
prog = ProgDisp();
prog.start();
i_sim = 1;
while sim.t < t_max
    % Run simulator
    sim.update(u_st)
    x = sim.x;
    
    % Simulate IMU
    w_b = gyro.measure(x);
    a_b = accel.measure(x);
    u_ekf = [w_b; a_b];
    ekf.predict(u_ekf);
    
    % Simulate GPS
    if ~mod(i_sim, n_gps)
        z_ekf = gps.measure(x);
        ekf.correct(z_ekf);
    end
    
    % Update logs
    log_sim.update();
    log_ekf.update();
    prog.update(sim.t / t_max);
    i_sim = i_sim + 1;
end

% Plot results
fprintf('Plotting...\n')

% Trajectories
figure;
log_sim.plot_path();
log_ekf.plot_path();
drawnow

end