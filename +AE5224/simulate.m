function log = simulate(model, ctrl, x_st, t_max, del_t)
%log = SIMULATE(body, ctrl, x_st, t_max, del_t)
%   Simulate and plot UAV flight
%   
%   Inputs:
%   - model = Aircraft model [AE5224.rigid_body.Model]
%   - ctrl = Control system [AE5224.control.Controller]
%   - x_st = Trim state [p_e; q_e; v_e; w_b]
%   - t_max = Simulation duration [s]
%   - del_t = Simulation timestep [s]
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
import('AE5224.sensors.Air');
import('AE5224.sensors.GPS');
import('AE5224.EKF');
import('timing.ProgDisp');

% Create simulators
f_sim = 1 / del_t;
[p_e, q_e, vb_e, ~] = Model.unpack_x(x_st);
sim_body = Sim(model, x_st, del_t);
sim_wind = Wind(norm(vb_e), del_t);

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
va_e = zeros(3, 1);
b_e = get_b();
x_est = EKF.pack_x(q_e, p_e, vb_e, va_e, b_e);
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
n_log = ceil(t_max / del_t) + 1;
switch class(model)
    case 'AE5224.fixed_wing.Model'
        log = AE5224.fixed_wing.Log(sim_body, sim_wind, ekf, n_log);
    case 'AE5224.quad_rotor.Model'
        log = AE5224.quad_rotor.Log(sim_body, sim_wind, ekf, n_log);
end

% Run simulator
fprintf('Simulating trim...\n');
prog = ProgDisp();
prog.start();
i_sim = 1;
while sim_body.t < t_max
    
    % Simulate IMU
    x = sim_body.x;
    z_gyr = gyro.measure(x);
    z_acc = accel.measure(x);
    z_mag = mag.measure(x);
    u_ekf = EKF.pack_u(z_gyr, z_acc);
    ekf.predict(u_ekf);
    ekf.correct_mag(z_mag);
    
    % Simulate airspeed sensor
    va_b = sim_wind.va_b;
    z_air = air.measure(x, va_b);
    % ekf.correct_air(z_air);   % Currently unstable
    
    % Simulate GPS
    if ~mod(i_sim, n_gps)
        z_gps = gps.measure(sim_body.x);
        ekf.correct_gps(z_gps);
    else
        z_gps = nan(6, 1);
    end
    
    % Simulate dynamics and control
    [q_e, p_e, vb_e, ~, ~] = EKF.unpack_x(ekf.x_est);
    x_hat = Model.pack_x(p_e, q_e, vb_e, z_gyr);
    u = ctrl.update(x_hat);
    sim_body.update([u; va_b]);
    sim_wind.update();
    
    % Logging and progress
    log.update(u, z_gyr, z_mag, z_air, z_gps);
    prog.update(sim_body.t / t_max);
    i_sim = i_sim + 1;
end

% Plot results
fprintf('Plotting...\n')
log.plot();

end