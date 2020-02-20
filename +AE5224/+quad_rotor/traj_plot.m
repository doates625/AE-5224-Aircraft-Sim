function traj_plot()
%TRAJ_PLOT() Plot trajectory from project assignment

% Trajectory generation
C = [...
    [+0.0000e+0, +1.4469e+1, -1.0127e-1, -1.1841e-4, -7.6013e-4, -3.4094e-3, +2.9767e-4];
    [+9.1440e+0, +3.8769e+0, -4.1309e+0, +1.1727e+0, -1.6510e-1, +1.2217e-2, -3.8280e-4];
    [-1.6764e+1, +7.8507e-1, +3.1597e+0, -1.0059e+0, +1.2597e-1, -6.9039e-3, +1.2789e-4]];
t = 0:0.01:7.25;
T = (t.'.^(0:6)).';
P = C*T;

% 3D Plotting
figure;
hold on, grid on
title('Trajectory')
xlabel('Pos-x [m]')
ylabel('Pos-y [m]')
zlabel('Pos-z [m]')
plot3(P(1, :), P(2, :), P(3,:), 'b-')
set(gca, 'YDir','reverse')
set(gca, 'ZDir','reverse')
view(-105, +25)
axis equal
camproj perspective

end