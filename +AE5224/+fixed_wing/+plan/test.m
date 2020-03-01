function log = test(p_a, h_a, p_b, h_b, p_i, h_i)
%log = TEST(p_a, h_a, p_b, h_b, p_i, h_i)
%   Test Dubins path
%   
%   Inputs:
%   - p_a = Position A [km]
%   - h_a = Heading A [deg]
%   - p_b = Position B [km]
%   - h_b = Heading B [deg]
%   - p_i = Position init [km]
%   - h_i = Heading init [deg]
%   
%   Outputs:
%   - log = Simulation log [AE5224.rigid_body.Log]
%   
%   Constants:
%   - Turn radius 150 [m]
%   - Airspeed 30 [m/s]
%   - Altitude 100 [m]

% Imports
import('AE5224.fixed_wing.Model');
import('AE5224.fixed_wing.Log');
import('AE5224.fixed_wing.control.Dubins');
import('AE5224.rigid_body.Model.pack_x');
import('AE5224.dubins.get_path');
import('AE5224.sim');
import('quat.Quat');

% Constants
rad = 150;
vel = 30;
alt = 100;

% Display Title
clc
fprintf('Fixed-Wing Dubins Path\n\n');

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
p_e = p_i * 1000;
h_e = deg2rad(h_i);
q_e = Quat([0; 0; 1], h_e);
v_e = q_e.rotate([vel; 0; 0]);
q_e = q_e.vector();
w_b = zeros(3, 1);
x_init = pack_x(p_e, q_e, v_e, w_b);

% Controller
model = Model();
ctrl = Dubins(model, path, vel, alt);

% Simulation
sim_wind = true;
ekf_fb = true;
del_t = 0.01;
log = sim(model, ctrl, sim_wind, ekf_fb, x_init, time, del_t, @Log);

end