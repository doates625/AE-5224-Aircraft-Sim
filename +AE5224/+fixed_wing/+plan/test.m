function test(pa, ha, pb, hb)
%TEST(pa, ha, pb, hb)
%   Test Dubins path
%   
%   Inputs:
%   - pa = Point A [x; y] [m]
%   - ha = Heading A [rad]
%   - pb = Point B [x; y] [m]
%   - hb = Heading B [rad]
%   
%   Constants:
%   - Turn radius 150 [m]
%   - Airspeed 30 [m/s]
%   - Altitude 100 [m]

% Imports
import('AE5224.dubins.get_path');

% Constants
r = 150;
v = 30;
h = 100;

% Display problem
fprintf('Finding Dubins path...\n');
fprintf('A: p = (%+.1f, %+.1f, %+.1f) [km], h = %.0f [deg]\n', ...
    pa/1000, h/1000, rad2deg(ha));
fprintf('B: p = (%+.1f, %+.1f, %+.1f) [km], h = %.0f [deg]\n', ...
    pb/1000, h/1000, rad2deg(hb));
fprintf('\n');

% Dubins path
path = get_path(pa, ha, pb, hb, r);
cls = class(path);
cls = cls(end-2:end);
dist = path.dist();
time = dist / v;

% Display solution
fprintf('Solution:\n');
fprintf('Type: %s\n', cls);
fprintf('Dist: %.0f [m]\n', dist);
fprintf('Time: %.1f [s]\n', time);
fprintf('\n');
path.plot(figure(1));

end