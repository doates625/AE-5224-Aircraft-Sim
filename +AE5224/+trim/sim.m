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
%   
%   Outputs:
%   - log = Log object [AE5224.Log]

% Imports
import('AE5224.const.get_b');
import('AE5224.rigid_body.Sim');
import('AE5224.rigid_body.Model');
import('AE5224.sensors.Gyro');
import('AE5224.sensors.Acc');
import('AE5224.sensors.Mag');
import('AE5224.sensors.Air');
import('AE5224.sensors.GPS');
import('AE5224.EKF');
import('AE5224.Log');
import('timing.ProgDisp');

% Create simulator
f_sim = 1 / del_t;
sim = Sim(body, x_st, del_t);

% Create sensors
f_imu = 100.0;
gyro = Gyro(f_imu);
accel = Acc(f_imu);
mag = Mag();
air = Air();

% Create GPS
f_gps = 1.0;
gps = GPS();
n_gps = round(f_sim / f_gps);

% Create EKF
[p_e, q_e, v_e, ~] = Model.unpack_x(x_st);
w_e = zeros(3, 1);
b_e = get_b();
x_est = EKF.pack_x(q_e, p_e, v_e, w_e, b_e);
cov_x = zeros(16);
cov_x(05:07, 05:07) = gps.cov_p;
cov_x(08:10, 08:10) = gps.cov_v;
cov_x(11:13, 11:13) = air.cov_z;
cov_x(14:16, 14:16) = mag.cov_z;
ekf = EKF(x_est, cov_x, ...
    gyro.cov_z, ...
    accel.cov_z, ...
    mag.cov_z, ...
    air.cov_z, ...
    gps.cov_p, ...
    gps.cov_v, ...
    del_t);

% Create logger
log = Log(sim, ekf, t_max);

% Run simulator
fprintf('Simulating trim...\n');
prog = ProgDisp();
prog.start();
i_sim = 1;
while sim.t < t_max
    % Simulate dynamics
    sim.update(u_st)
    x = sim.x;
    
    % Simulate IMU
    z_gyr = gyro.measure(x);
    z_acc = accel.measure(x);
    z_mag = mag.measure(x);
    u_ekf = EKF.pack_u(z_gyr, z_acc);
    ekf.predict(u_ekf);
    ekf.correct_mag(z_mag);
    
    % Simulate airspeed sensor
    z_air = air.measure(x);
    % ekf.correct_air(z_air);
    
    % Simulate GPS
    if ~mod(i_sim, n_gps)
        z_gps = gps.measure(x);
        ekf.correct_gps(z_gps);
        log.update(z_gyr, z_mag, z_air, z_gps);
    else
        log.update(z_gyr, z_mag, z_air);
    end
    
    % Progress tracker
    prog.update(sim.t / t_max);
    i_sim = i_sim + 1;
end

% Plot results
fprintf('Plotting...\n')
log.save();
log.plot();
drawnow

end