function log = test(pa, ha, pb, hb)
%log = TEST(pa, ha, pb, hb)
%   Test Dubins path
%   
%   Inputs:
%   - pa = Point A [x; y] [m]
%   - ha = Heading A [rad]
%   - pb = Point B [x; y] [m]
%   - hb = Heading B [rad]
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
import('AE5224.dubins.get_path');
import('AE5224.sim');

% Constants
R = 150;
V = 30;
h = 100;

% Display Title
clc
fprintf('Fixed-Wing Dubins Path 1\n\n');

% Display problem
fprintf('Finding Dubins path...\n');
fprintf('A: p = (%+.2f, %+.2f, %+.2f) [km], h = %.0f [deg]\n', ...
    pa/1000, -h/1000, rad2deg(ha));
fprintf('B: p = (%+.2f, %+.2f, %+.2f) [km], h = %.0f [deg]\n', ...
    pb/1000, -h/1000, rad2deg(hb));
fprintf('\n');

% Dubins path
path = get_path(pa, ha, pb, hb, R);
cls = class(path);
cls = cls(end-2:end);
dist = path.dist();
time = dist / V;

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

% Controller
model = Model();
ctrl = Dubins(model, path, V, h);
sim_wind = true;
ekf_fb = true;
x_st = ctrl.x_st;
log = sim(model, ctrl, sim_wind, ekf_fb, x_st, time, 0.01, @Log);

end