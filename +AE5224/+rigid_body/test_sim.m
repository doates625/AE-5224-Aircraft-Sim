function test_sim()
%TEST_SIM() Tests simulator using simple turn and climb
%   
%   Author: Dan Oates (WPI Class of 2020)

% Imports
import('AE5224.rigid_body.Body');
import('AE5224.rigid_body.Sim');
import('AE5224.rigid_body.Log');

% Inertial params
m = 1;          % Mass [kg]
I_b = eye(3);   % Inertia [kg*m^2]
body = Body(m, I_b);

% Motion params
R = 10;         % Turn radius [m]
v_r = 5;        % Airspeed [m/s]
v_h = 1;        % Climb rate [m/s]
w = v_r/R;      % Yaw velocity [rad/s]
F = m*v_r*w;    % Lateral force [N]

% Initial conditions
p_e = [0; -R; 0];
q_e = [1; 0; 0; 0];
v_e = [v_r; 0; -v_h];
w_b = [0; 0; w];

% Sim and log setup
del_t = 0.1;
sim = Sim(body, del_t, p_e, q_e, v_e, w_b);
log = Log(sim);

% Simulation
t_max = 30;
while sim.t < t_max
    F_b = [0; F; 0];
    M_b = [0; 0; 0];
    sim.update(F_b, M_b);
    log.update();
end

% Log save and plot
log.save('test_sim.mat');
log.plot_path();

end

