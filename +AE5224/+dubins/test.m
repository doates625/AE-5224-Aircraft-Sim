% Test Script for Dubins Paths
clear, clc

% Imports
import('AE5224.dubins.path.LSL');
import('AE5224.dubins.path.LSR');
import('AE5224.dubins.path.RSL');
import('AE5224.dubins.path.RSR');
import('AE5224.dubins.get_path');

% Path conditions
pa = 5*(2*rand(2, 1) - 1);
ha = pi*(2*rand()-1);
pb = 5*(2*rand(2, 1) - 1);
hb = pi*(2*rand()-1);
r = 1;

% Generate paths
fprintf('Generating Dubins paths...\n\n');
paths = {@LSL, @LSR, @RSL, @RSR};
names = {'LSL', 'LSR', 'RSL', 'RSR'};
for i_p = 1:4
    try
        path = paths{i_p}(pa, ha, pb, hb, r);
        cls = class(path);
        for i_s = 1:3
            fig = figure;
            fig.Name = [cls(end-2:end) ' Section ' int2str(i_s)];
            path.plot(fig);
            x = -10:0.5:+10;
            y = -10:0.5:+10;
            p_sect = zeros(2, 0);
            for i_x = 1:length(x)
                for i_y = 1:length(y)
                    p = [x(i_x); y(i_y)];
                    if path.in_sect(p, i_s)
                        p_sect(:, end+1) = p;
                    end
                end
            end
            plot(p_sect(2, :), p_sect(1, :), 'rx', 'DisplayName', 'Sect')
            drawnow
        end
        status = 'Generated';
        fprintf('Generated %s path (dist = %.1f).\n', names{i_p}, path.dist());
    catch
        fprintf('Failed %s path.\n', names{i_p});
    end
end
fprintf('\n');

% Best path
best_path = get_path(pa, ha, pb, hb, r);
cls = class(best_path);
fprintf('Best path: %s\n', cls(end-2:end))