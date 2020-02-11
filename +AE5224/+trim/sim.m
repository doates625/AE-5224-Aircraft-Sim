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
import('AE5224.rigid_body.Body.unpack');
import('AE5224.sensors.Gyroscope');
import('AE5224.sensors.Accelerometer');
import('AE5224.sensors.Magnetometer');
import('AE5224.sensors.GPS');
import('AE5224.EKF.get_set_del_t');
import('AE5224.EKF.EKF');
import('AE5224.EKF.Log');
import('AE5224.EKF.pack_x');
import('timing.ProgDisp');

% Create simulator
sim = Sim(body, x_st, del_t);
log_sim = AE5224.rigid_body.Log(sim);

% Create IMU
f_imu = 1 / del_t;
gyr = Gyroscope(f_imu);
acc = Accelerometer(f_imu);
mag = Magnetometer();

% Create GPS
f_gps = 1.0;
gps = GPS();
n_gps = round(f_gps / sim.del_t);

% Create EKF
get_set_del_t(del_t);
[p_e, q_e, v_e, ~] = unpack(x_st);
w_e = zeros(3, 1);
b_e = get_b();
xh = pack_x(q_e, p_e, v_e, w_e, b_e);
Ew = gyr.cov_z;
Ea = acc.cov_z;
Ep = gps.cov_z(1:3, 1:3);
Ev = gps.cov_z(4:6, 4:6);
Ex = zeros(16);
Ex(05:07, 05:07) = Ep;
Ex(08:10, 08:10) = Ev;
Ex(11:13, 11:13) = Ev;
Ex(14:16, 14:16) = mag.cov_z;
ekf = EKF(xh, Ex, Ew, Ea, Ep, Ev);
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
    w_b = gyr.measure(x);
    a_b = acc.measure(x);
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
log_sim.plot_path();
log_ekf.plot_path();
drawnow

end