function log = test(del_t)
%log = TEST(del_t)
%   Test polynomial trajectory tracking controller
%   - del_t = Simulation time step [s, def = 0.01]
%   - log = Simulation log [AE5224.rigid_body.Log] 

% Imports
import('AE5224.quad_rotor.Model');
import('AE5224.quad_rotor.control.Poly');
import('AE5224.quad_rotor.Log');
import('AE5224.rigid_body.Model.pack_x');
import('AE5224.Trim');
import('AE5224.sim');
import('quat.Quat');

% Default args
if nargin < 1, del_t = 0.01; end

% Initial printout
fprintf('Quadrotor Polynomial Tracking Test\n\n');

% Modeling and control
model = Model();
p_mat = [...
    [+0.0000e+0, +9.1440e+0, -1.6764e+1];
    [+1.4469e+1, +3.8769e+0, +7.8507e-1];
    [-1.0127e-1, -4.1309e+0, +3.1597e+0];
    [-1.1841e-4, +1.1727e+0, -1.0059e+0];
    [-7.6013e-4, -1.6510e-1, +1.2597e-1];
    [-3.4094e-3, +1.2217e-2, -6.9039e-3];
    [+2.9767e-4, -3.8280e-4, +1.2789e-4];
].';
ctrl = Poly(model, p_mat, del_t);
t_max = 7.25;

% Initial conditions
trim = Trim(15, inf, deg2rad(-3), 0);
p_e = [0.0; 9.0; -17.0];
q_e = Quat([0; 0; 1], deg2rad(15));
v_e = q_e.rotate(trim.v_e);
w_b = q_e.rotate(trim.w_e);
q_e = q_e.vector();
x_init = pack_x(p_e, q_e, v_e, w_b);

% Run simulator
sim_wind = false;
ekf_fb = false;
log = sim(model, ctrl, sim_wind, ekf_fb, x_init, t_max, del_t, @Log);

% Tracking plot
figure;
lbs = 'XYZ';
p_e_cmd = ctrl.get(log.log_t);
for i = 1:3
    subplot(4, 1, i)
    hold on; grid on;
    title(['Position ' lbs(i)])
    xlabel('Time [s]')
    ylabel('Pos [m]')
    plot(log.log_t, log.p_e_act(i, :), 'b-');
    plot(log.log_t, p_e_cmd(i, :), 'k--');
    legend('Act', 'Cmd');
end

% Error plot
err = vecnorm(p_e_cmd - log.p_e_act);
subplot(4, 1, 4)
hold on; grid on;
title('Position Error Norm');
xlabel('Time [s]')
ylabel('Err [m]')
plot(log.log_t, err, 'r-');

end