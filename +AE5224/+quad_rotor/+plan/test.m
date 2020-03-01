function log = test(p_a, h_a, p_b, h_b)
%log = TEST(p_a, h_a, p_b, h_b)
%   Test Dubins path
%   
%   Inputs:
%   - p_a = Point A [x; y] [km]
%   - h_a = Heading A [deg]
%   - p_b = Point B [x; y] [km]
%   - h_b = Heading B [deg]
%   
%   Outputs:
%   - log = Simulation log [AE5224.rigid_body.Log]
%   
%   Constants:
%   - Turn radius 20 [m]
%   - Airspeed 10 [m/s]
%   - Altitude 100 [m]

% Imports
import('AE5224.quad_rotor.Model');
import('AE5224.quad_rotor.Log');
import('AE5224.quad_rotor.control.Dubins');
import('AE5224.rigid_body.Model.pack_x');
import('AE5224.dubins.get_path');
import('AE5224.sim');
import('quat.Quat');

% Constants
rad = 20;
vel = 10;
alt = 100;

% Display Title
clc
fprintf('Quadrotor Dubins Path 1\n\n');

% Display problem
fprintf('Finding Dubins path...\n');
fprintf('A: p = (%+.2f, %+.2f, %+.2f) [km], h = %.0f [deg]\n', p_a, h_a);
fprintf('B: p = (%+.2f, %+.2f, %+.2f) [km], h = %.0f [deg]\n', p_b, h_b);
fprintf('\n');

% Dubins path
p_a = p_a(1:2) * 1000;
p_b = p_b(1:2) * 1000;
h_a = deg2rad(h_a);
h_b = deg2rad(h_b);
path = get_path(p_a, h_a, p_b, h_b, rad);
cls = class(path);
cls = cls(end-2:end);
dist = path.dist();
time = dist / vel;

% Display solution
fprintf('Solution:\n');
fprintf('Type: %s\n', cls);
fprintf('Dist: %.0f [m]\n', dist);
fprintf('Time: %.1f [s]\n', time);
fprintf('\n');
path.plot(figure(1));
xlabel('Pos-Y [m]')
ylabel('Pos-X [m]')
drawnow

% Initial conditions
p_e = [p_a; -alt] + 10*(2*rand(3, 1) - 1);
q_e = Quat().vector();
v_e = zeros(3, 1);
w_b = zeros(3, 1);
x_init = pack_x(p_e, q_e, v_e, w_b);

% Controller
model = Model();
del_t = 0.01;
ctrl = Dubins(model, path, vel, alt, del_t);

% Simulation
sim_wind = false;
ekf_fb = true;
log = sim(model, ctrl, sim_wind, ekf_fb, x_init, time, del_t, @Log);

end