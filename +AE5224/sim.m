function log = sim(model, ctrl, sim_wind, ekf_fb, x_st, t_max, del_t, log_cls)
%log = sim(model, ctrl, sim_wind, ekf_fb, x_st, t_max, del_t, log_cls)
%   Simulate and plot UAV flight
%   
%   Inputs:
%   - model = Aircraft model [AE5224.rigid_body.Model]
%   - ctrl = Control system [AE5224.control.Controller]
%   - sim_wind = Wind simulation flag [logical]
%   - ekf_fb = EKF feedback flag [logical]
%   - x_st = Trim state [p_e; q_e; v_e; w_b]
%   - t_max = Simulation duration [s]
%   - del_t = Simulation timestep [s]
%   - log_cls = Log class [@AE5224.Log]
%   
%   Outputs:
%   - log = Log object [AE5224.Log]

% Imports
import('AE5224.const.get_b');
import('AE5224.rigid_body.Sim');
import('AE5224.rigid_body.Model');
import('AE5224.Wind');
import('AE5224.sensors.Gyro');
import('AE5224.sensors.Acc');
import('AE5224.sensors.Mag');
import('AE5224.sensors.GPS');
import('AE5224.EKF');
import('timing.ProgDisp');

% Create simulators
f_sim = 1 / del_t;
[p_e, q_e, v_e, ~] = Model.unpack_x(x_st);
body_sim = Sim(model, x_st, del_t);
wind_sim = Wind(norm(v_e), del_t);

% Create sensors
f_imu = 100.0;
gyro = Gyro(f_imu);
accel = Acc(f_imu);
mag = Mag();

% Create GPS
f_gps = 1.0;
gps = GPS();
n_gps = round(f_sim / f_gps);

% Create EKF
b_e = get_b();
x_est = EKF.pack_x(q_e, p_e, v_e, b_e);
cov_x = zeros(13);
cov_x(05:07, 05:07) = gps.cov_p;
cov_x(08:10, 08:10) = gps.cov_v;
cov_x(11:13, 11:13) = mag.cov_z;
ekf = EKF(x_est, cov_x, ...
    gyro.cov_z, ...
    accel.cov_z, ...
    mag.cov_z, ...
    gps.cov_p, ...
    gps.cov_v, ...
    del_t);

% Create logger
n_log = ceil(t_max / del_t) + 1;
log = log_cls(body_sim, ekf, n_log);

% Run simulator
fprintf('Simulating trim...\n');
prog = ProgDisp();
prog.start();
i_sim = 1;
while body_sim.t < t_max && ~ctrl.finished()
    
    % State and time
    x = body_sim.x;
    t = body_sim.t;
    
    % Simulate IMU
    z_gyr = gyro.measure(x);
    z_acc = accel.measure(x);
    z_mag = mag.measure(x);
    u_ekf = EKF.pack_u(z_gyr, z_acc);
    ekf.predict(u_ekf);
    ekf.correct_mag(z_mag);
    
    % Simulate GPS
    if ~mod(i_sim, n_gps)
        z_gps = gps.measure(x);
        ekf.correct_gps(z_gps);
    else
        z_gps = nan(6, 1);
    end
    
    % Simulate dynamics and control
    va_b = wind_sim.update();
    if ekf_fb
        [q_e, p_e, v_e, ~] = EKF.unpack_x(ekf.x_est);
        x_est = Model.pack_x(p_e, q_e, v_e, z_gyr);
    else
        x_est = x;
    end
    u_ctrl = ctrl.update(x_est, t);
    body_sim.update(u_ctrl, va_b);
    if sim_wind; wind_sim.update(); end
    
    % Logging and progress
    log.update(z_gyr, z_mag, z_gps, u_ctrl);
    prog.update(body_sim.t / t_max);
    i_sim = i_sim + 1;
end

% Plot results
fprintf('Plotting...\n')
log.plot();

end